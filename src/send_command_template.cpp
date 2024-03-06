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

// Define any required enums or structs to represent flight states or other constants
enum class FlightState {


};

class DroneControl : public rclcpp::Node {
public:
    DroneControl() : Node("drone_control") {

        initializeWaypoints();

        // Initialize publishers
        offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

		// Initialize subscribers with appropriate topics and QoS settings
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		vehicle_odometry_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", qos,
					 std::bind(&DroneControl::odometry_callback, this, std::placeholders::_1));
		distance_sensor_subscriber_ = this->create_subscription<px4_msgs::msg::DistanceSensor>("/fmu/out/distance_sensor", qos, 
					 std::bind(&DroneControl::distance_sensor_callback, this, std::placeholders::_1));

        offboard_setpoint_counter_ = 0;

		auto timer_callback = [this]() -> void {

			if (offboard_setpoint_counter_ == 10) {
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

				// Arm the vehicle
				this->arm();
			}

			// offboard_control_mode needs to be paired with trajectory_setpoint
			publish_offboard_control_mode();
			publish_trajectory_setpoint(square_waypoints_[0]);

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
    std::vector<std::array<float, 3>> square_waypoints_;
    rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;

	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_subscriber_;
	rclcpp::Subscription<px4_msgs::msg::DistanceSensor>::SharedPtr distance_sensor_subscriber_;
	
	int last_completed_waypoint_ = -1;
	float POSITION_ERROR_TOLERANCE = 1.5; //m

	px4_msgs::msg::VehicleOdometry latest_odometry_;
	px4_msgs::msg::DistanceSensor latest_distance_data_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

    void initializeWaypoints();
	void publish_offboard_control_mode();
	void publish_trajectory_setpoint(std::array<float, 3> waypoint);
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

	void odometry_callback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg);
	void distance_sensor_callback(const px4_msgs::msg::DistanceSensor::UniquePtr msg);

	bool hasWaypointReached(int last_requested_waypoint_index_);
	bool isNearObstacle();
};

void DroneControl::odometry_callback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg)
{
	latest_odometry_ = *msg;

	std::cout << "Current xyz: " << msg->position[0] << " " << msg->position[1] << " " << msg->position[2] << " " << std::endl;
}

void DroneControl::distance_sensor_callback(const px4_msgs::msg::DistanceSensor::UniquePtr msg)
{
	latest_distance_data_ = *msg;

	std::cout << "Current distance: " << msg->current_distance << std::endl;
}

bool DroneControl::hasWaypointReached(int last_requested_waypoint_index_)
{
	bool result = false;

	return result;
}

bool DroneControl::isNearObstacle()
{
	bool result = false;

	return result;
}

void DroneControl::initializeWaypoints()
{
    // Initialize waypoints forming a square trajectory at the specified takeoff height
    square_waypoints_ = {
        {0.0, 0.0, -15.0}, // Takeoff position (assuming NED coordinates)
        {50.0, 0.0, -15.0}, // First waypoint
        {50.0, 50.0, -15.0}, // Second waypoint
        {0.0, 50.0, -15.0} // Third waypoint
    };
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
	msg.velocity = false;
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
	px4_msgs::msg::TrajectorySetpoint msg{};
	msg.position = waypoint;
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