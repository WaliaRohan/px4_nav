#include "rclcpp/rclcpp.hpp"

// Include necessary message types from px4_msgs

// Following offboard_control.cpp to navigate drone
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/distance_sensor.hpp>

#include <chrono>
#include <iostream>

#include <cmath>

// Define any required enums or structs to represent flight states or other constants
enum class FlightState {
	NAVIGATE,
	AVOID_OBSTACLE,
	TRANSITION_TO_NAVIGATE
};

class DroneControl : public rclcpp::Node {
public:
    DroneControl() : Node("drone_control") {

        initializeWaypoints();

        // Initialize publishers
        offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
		// attitude_setpoint_publisher = this->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>("fmu/in/vehi")

		// Initialize subscribers with appropriate topics and QoS settings
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		vehicle_odometry_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", qos,
					 std::bind(&DroneControl::odometry_callback, this, std::placeholders::_1));
		distance_sensor_subscriber_ = this->create_subscription<px4_msgs::msg::DistanceSensor>("/fmu/out/distance_sensor", qos, 
					 std::bind(&DroneControl::distance_sensor_callback, this, std::placeholders::_1));

		// Offboard Control
        offboard_setpoint_counter_ = 0;

		auto timer_callback = [this]() -> void {

			if (offboard_setpoint_counter_ == 10) {
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

				// Arm the vehicle
				this->arm();
			}

			calculateNextWaypoint();

			// offboard_control_mode needs to be paired with trajectory_setpoint
			publish_offboard_control_mode();
			publish_trajectory_setpoint(next_waypoint_);

			// stop the counter after reaching 11
			if (offboard_setpoint_counter_ < 11) {
				offboard_setpoint_counter_++;
			}
		};

		timer_ = this->create_wall_timer(std::chrono::milliseconds(100), timer_callback);
    }

    void arm();
	void disarm();

private:

	// Data members
    std::vector<std::array<float, 3>> square_waypoints_;

	int last_requested_waypoint_index_ = -1; // index value (-1 means no waypoints requested)
	float POSITION_ERROR_TOLERANCE = 1.0; // m
	float DISTANCE_ERROR_THRESHOLD = 5.0; // m
	float DISTANCE_SIGNAL_QUALITY_THRESHOLD = 50; // approximate distance sensor quality value at DISTANCE_ERROR_THRESHOLD -> empirically calculated
	float STOP_VELOCITY_THRESHOLD = 0.1; // m/s
	float CLEAR_DISTANCE = 1.0; // m -> Distance to clear in forward direction before switching from AVOID_OBSTACLE to NAVIGATE mode. Should be atleast the length of the drone
	bool DRONE_WAS_STOPPED = false; // flag to check if drone was stopped after encountering obstacle
	bool set_zero_velocity_ = false; // flag to set zero velocity for trajectory
	std::array<float, 3> next_waypoint_;

	FlightState state = FlightState::NAVIGATE;

	px4_msgs::msg::VehicleOdometry latest_odometry_;
	px4_msgs::msg::DistanceSensor latest_distance_data_;

	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_subscriber_;
	rclcpp::Subscription<px4_msgs::msg::DistanceSensor>::SharedPtr distance_sensor_subscriber_;

	rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;

	rclcpp::TimerBase::SharedPtr timer_;
	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

	// Helper methods
	void initializeWaypoints();
	bool hasWaypointReached();
	bool isNearObstacle();
	void calculateNextWaypoint();
    
	// Subscriber callbacks
	void odometry_callback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg);
	void distance_sensor_callback(const px4_msgs::msg::DistanceSensor::UniquePtr msg);

	// Publisher callbacks
	void publish_offboard_control_mode();
	void publish_trajectory_setpoint(std::array<float, 3> waypoint);
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);	
};

void DroneControl::initializeWaypoints()
{
    // Initialize waypoints forming a square trajectory at the specified takeoff height
    square_waypoints_ = {
        {0.0, 0.0, -15.0}, // Takeoff position (assuming NED coordinates)
        {50.0, 0.0, -15.0}, // First waypoint
        {50.0, 50.0, -15.0}, // Second waypoint
        {0.0, 50.0, -15.0} // Third waypoint
    };

	next_waypoint_ = {0, 0, 0};
}

bool DroneControl::hasWaypointReached()
{
	bool result = false;

	// check if drone reached last waypoint

	float dx = next_waypoint_[0] - latest_odometry_.position[0];
	float dy = next_waypoint_[1] - latest_odometry_.position[1];
	float dz = next_waypoint_[2] - latest_odometry_.position[2];

	float error = sqrt(dx*dx + dy*dy + dz*dz);
	
	result = error <= POSITION_ERROR_TOLERANCE;

	std::cout << "Target waypoint reached? " << error << std::endl;

	// If drone reached last waypoint, and drone is in NAVIGATE mode, go to next square waypoint
	if (result == true && state == FlightState::NAVIGATE)
	{
		// Check if drone reached last requested square waypoint
		dx = square_waypoints_[last_requested_waypoint_index_][0] - latest_odometry_.position[0];
		dy = square_waypoints_[last_requested_waypoint_index_][1] - latest_odometry_.position[1];
		dz = square_waypoints_[last_requested_waypoint_index_][2] - latest_odometry_.position[2];

		error = sqrt(dx*dx + dy*dy + dz*dz);
		
		result = error <= POSITION_ERROR_TOLERANCE;

		// select next square waypoint if drone reached last square waypoint
		if (result == true && last_requested_waypoint_index_ != -1)
		{
			std::cout << "------------ Waypoint " << last_requested_waypoint_index_ << " reached with " << error << " error -----------\n"; 

			if (last_requested_waypoint_index_ < square_waypoints_.size() - 1)
				last_requested_waypoint_index_++; // move to next waypoint
			else
			{
				last_requested_waypoint_index_ = 0; // restart lap
				std::cout << "------------ Starting next lap ------------\n";
			}
		}
	}

	return result;
}

bool DroneControl::isNearObstacle()
{
	bool result = false;

	if (latest_distance_data_.current_distance <= DISTANCE_ERROR_THRESHOLD && static_cast<int>(latest_distance_data_.signal_quality) >= DISTANCE_SIGNAL_QUALITY_THRESHOLD)
	{
		result = true;
		state = FlightState::AVOID_OBSTACLE;
		set_zero_velocity_ = DRONE_WAS_STOPPED ? false : true; // If drone was stopped after encountering obstacle, don't set velocity to zero. Otherwise, set it to zero.
		std::cout << "------------ Obstacle Avoidance Mode ------------\n";

	}
	else
	{
		// IF drone is transitioning from AVOID_OBSTACLE -> NAVIGATE, 
		// fly forward for value set by CLEAR_DISTANCE to avoid getting
		// stuck behind obstacle in case drone pitches down before moving forward.	
		if (state == FlightState::AVOID_OBSTACLE)
		{
			state = FlightState::TRANSITION_TO_NAVIGATE;
			std::cout << "------------ Transition Mode ------------\n";
			
		}
		else if (state == FlightState::TRANSITION_TO_NAVIGATE)
		{
			if (hasWaypointReached()) // has drone cleared last seen obstacle
			{
				state = FlightState::NAVIGATE; // set state to navigate
			}
		}
		else 
		{
			state = FlightState::NAVIGATE;
			set_zero_velocity_ = false;
			DRONE_WAS_STOPPED = false;
			std::cout << "------------ Navigation Mode ------------\n";
		}
	}

	return result;
}

void DroneControl::calculateNextWaypoint()
{
	if (state == FlightState::NAVIGATE)
	{
		if (last_requested_waypoint_index_ == -1) // no waypoints have been requested since this node was running
			last_requested_waypoint_index_ = 0;

		next_waypoint_ = square_waypoints_[last_requested_waypoint_index_];
		std::cout << "Updated waypoint to " << last_requested_waypoint_index_ << std::endl;
	}
	else if (state == FlightState::AVOID_OBSTACLE)
	{
		next_waypoint_ = latest_odometry_.position;

		if(set_zero_velocity_ == true)
		{
			// zero velocity will automatically be published in publish_trajectory_setpoint method

			// check if NORTH velocity has become zero
			if (latest_odometry_.velocity[0] <= STOP_VELOCITY_THRESHOLD)
				DRONE_WAS_STOPPED = true;
		}
		else // this clause assumes drone was stopped -> now fly up to avoid obstacle
		{
			next_waypoint_[2] = next_waypoint_[2] - 2.0; // fly up to avoid current obstacle
		}
	}
	else if (state == FlightState::TRANSITION_TO_NAVIGATE)
	{
		if (hasWaypointReached())
		{
			next_waypoint_ = latest_odometry_.position;
			next_waypoint_[0] = next_waypoint_[0] + latest_distance_data_.current_distance + CLEAR_DISTANCE; // Move forward by "CLEAR_DISTANCE" to clear the obstacle
			next_waypoint_[2] = next_waypoint_[2] - 2.0; // move 1m up
		}
	}
	
}

void DroneControl::odometry_callback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg)
{
	latest_odometry_ = *msg;

	std::cout << "Current xyz: " << msg->position[0] << " " << msg->position[1] << " " << msg->position[2] << " "
			//   << "Current acc: " << msg->acceleration[0] << " " << msg->acceleration[1] << " " << msg->acceleration[2] << " "
			  << std::endl;

	hasWaypointReached();
}

void DroneControl::distance_sensor_callback(const px4_msgs::msg::DistanceSensor::UniquePtr msg)
{
	latest_distance_data_ = *msg;

	std::cout << "---- Current distance: " << msg->current_distance << " & Signal Quality: " << static_cast<int>(msg->signal_quality) << "%" << std::endl;

	isNearObstacle();
}

void DroneControl::arm()
{
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void DroneControl::disarm()
{
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void DroneControl::publish_offboard_control_mode()
{
	px4_msgs::msg::OffboardControlMode msg{};
	msg.position = true;
	// set_zero_velocity_ ? msg.velocity = true : msg.velocity = false; // control velocity if required
	msg.velocity = true;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void DroneControl::publish_trajectory_setpoint(std::array<float, 3> waypoint)
{

	std::cout << "Target xyz: " << waypoint[0] << " " << waypoint[1] << " " << waypoint[2] << std::endl;
	px4_msgs::msg::TrajectorySetpoint msg{};
	msg.position = waypoint;
	if (set_zero_velocity_)
	{
		msg.velocity = {0, 0, 0};
		std::cout << "Target vel: " << msg.velocity[0] << " " << msg.velocity[1] << " " << msg.velocity[2] << std::endl;
	}
	// msg.velocity = {0, 0, 0};
	// std::cout << "Target vel: " << msg.velocity[0] << " " << msg.velocity[1] << " " << msg.velocity[2] << std::endl;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void DroneControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	px4_msgs::msg::VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DroneControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
