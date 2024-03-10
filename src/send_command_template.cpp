#include "DroneControl.hpp"
#include <cmath>

DroneControl::DroneControl() : Node("drone_control")
{
	initializeWaypoints();

	// Initialize publishers
	offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
	trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
	vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

	// Initialize subscribers with appropriate topics and QoS settings
	rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

	vehicle_odometry_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", qos,
					std::bind(&DroneControl::odometryCallback, this, std::placeholders::_1));
	distance_sensor_subscriber_ = this->create_subscription<px4_msgs::msg::DistanceSensor>("/fmu/out/distance_sensor", qos, 
					std::bind(&DroneControl::distanceSensorCallback, this, std::placeholders::_1));

	// Offboard Control
	offboard_setpoint_counter_ = 0;

	auto timer_callback = [this]() -> void {

		if (offboard_setpoint_counter_ == 10) {
			// Change to Offboard mode after 10 setpoints
			this->publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

			// Arm the vehicle
			this->arm();
		}

		if (!DISARM)
		{
			calculateNextWaypoint();

			// offboard_control_mode needs to be paired with trajectory_setpoint
			publishOffboardControlMode();
			publishTrajectorySetpoint(next_waypoint_);
		}
		else
			this->disarm(); // Completed TOTAL_LAPS, disarm drone

		// stop the counter after reaching 11
		if (offboard_setpoint_counter_ < 11) {
			offboard_setpoint_counter_++;
		}
	};

	timer_ = this->create_wall_timer(std::chrono::milliseconds(100), timer_callback);
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

	std::cout << state_name_[static_cast<int>(state_)] << " :: Error to target " << error << std::endl;

	// If drone reached last waypoint, and drone is in NAVIGATE mode, update square waypoint index
	if (result == true && state_ == FlightState::NAVIGATE)
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
			std::cout << state_name_[static_cast<int>(state_)] << " :: ------------ Lap " << lap_ << " Waypoint " 
			          << last_requested_waypoint_index_ + 1 << " reached with " << error << " error -----------\n"; 

			if (last_requested_waypoint_index_ < static_cast<int>(square_waypoints_.size() - 1))
				last_requested_waypoint_index_++; // move to next waypoint
			else // drone has reached end of a lap
			{
				if (lap_ < TOTAL_LAPS)
				{
					lap_++;
					last_requested_waypoint_index_ = 0; // restart lap
					std::cout << state_name_[static_cast<int>(state_)] << " :: ------------ Starting lap " << lap_ << " ------------\n";
				}
				else
				{
					state_ = FlightState::RETURN_TO_BASE;
				}
					
			}
		}
	}
	if (state_ == FlightState::RETURN_TO_BASE && next_waypoint_ == square_waypoints_[0] && result == true)
	{
		std::cout << state_name_[static_cast<int>(state_)] 
				  << " :: Target xyz: " << next_waypoint_[0] << " " << next_waypoint_[1] << " " << next_waypoint_[2] << std::endl;
		DISARM = true;
	}

	return result;
}

bool DroneControl::isNearObstacle()
{
	bool result = false;

	// latest_distance_data_.current_distance <= DISTANCE_ERROR_THRESHOLD && 
	if (static_cast<int>(latest_distance_data_.signal_quality) >= DISTANCE_SIGNAL_QUALITY_THRESHOLD)
	{
		result = true;
		state_ = FlightState::AVOID_OBSTACLE;
		set_zero_velocity_ = DRONE_WAS_STOPPED ? false : true; // If drone was stopped after encountering obstacle, 
															   // don't set velocity to zero. Otherwise, set it to zero.

		if(latest_distance_data_.current_distance > 0)
			last_positive_obstacle_distance_ = latest_distance_data_.current_distance;

	}
	else
	{
		if (state_ == FlightState::AVOID_OBSTACLE)
		{
			if (hasWaypointReached())
			{
				state_ = FlightState::CLEAR_OBSTACLE;
			}
		}
		else if (state_ == FlightState::CLEAR_OBSTACLE)
		{
			if (hasWaypointReached() && request_clear_obstacle_ == true) // has drone cleared last seen obstacle
			{
				state_ = FlightState::RETURN_TO_PATH;
				request_clear_obstacle_ = false;
			}	
		}
		else if (state_ == FlightState::RETURN_TO_PATH)
		{
			last_positive_obstacle_distance_ = 0; // since drone has cleared obstacle, set this tracker to 0 for next AVOID_OBSTACLE cycle
			bool is_mission_complete = lap_ == TOTAL_LAPS && last_requested_waypoint_index_ == static_cast<int>(square_waypoints_.size() - 1);
			if (hasWaypointReached()) // has drone returned to path
			{
				state_ = is_mission_complete ? FlightState::RETURN_TO_BASE : FlightState::NAVIGATE; // set state to navigate if mission is not complete
			}
		}
		else if (state_ != FlightState::RETURN_TO_BASE)
		{
			state_ = FlightState::NAVIGATE;
			set_zero_velocity_ = false;
			DRONE_WAS_STOPPED = false;
		}

	}

	return result;
}

void DroneControl::calculateNextWaypoint()
{
	if (state_ == FlightState::NAVIGATE)
	{
		if (last_requested_waypoint_index_ == -1) // no waypoints have been requested since this node was running
			last_requested_waypoint_index_ = 0;

		next_waypoint_ = square_waypoints_[last_requested_waypoint_index_];
		std::cout << state_name_[static_cast<int>(state_)] << " :: Updated waypoint to " << last_requested_waypoint_index_ + 1<< std::endl;
	}
	else if (state_ == FlightState::AVOID_OBSTACLE)
	{
		next_waypoint_ = latest_odometry_.position;

		if(set_zero_velocity_ == true)
		{
			// zero velocity will automatically be published in publishTrajectorySetpoint method

			// check if NORTH velocity has become zero
			if (latest_odometry_.velocity[0] <= STOP_VELOCITY_THRESHOLD)
				DRONE_WAS_STOPPED = true;
		}
		else // this clause assumes drone was stopped -> now fly right to avoid obstacle
		{
			if (isNearObstacle())
				next_waypoint_[1] = next_waypoint_[1] + CLEAR_DISTANCE; // fly right to avoid current obstacle
		}
	}
	else if (state_ == FlightState::CLEAR_OBSTACLE)
	{
		if (hasWaypointReached()) // check if AVOID_OBSTACLE waypoint was reached
		{
			next_waypoint_ = latest_odometry_.position;
			next_waypoint_[0] += last_positive_obstacle_distance_ + CLEAR_DISTANCE; // Move forward by "CLEAR_DISTANCE" to clear the obstacle
			std::cout << state_name_[static_cast<int>(state_)] << " :: Moving forward by " << last_positive_obstacle_distance_ + CLEAR_DISTANCE << std::endl;
			request_clear_obstacle_ = true;
		}
	}
	else if (state_ == FlightState::RETURN_TO_PATH)
	{
		if (hasWaypointReached()) // check if drone has cleared obstacle
		{
			int next_square_waypoint_index = last_requested_waypoint_index_;
			int previous_waypoint_index =  next_square_waypoint_index == 0
										   ? previous_waypoint_index = static_cast<int>(square_waypoints_.size() - 1) 
										   : next_square_waypoint_index - 1;

			waypointNED closest_waypoint_on_path = projectionPointOnLine(square_waypoints_[previous_waypoint_index],
																		 square_waypoints_[next_square_waypoint_index],
																		 latest_odometry_.position);

			closest_waypoint_on_path[0] += CLEAR_DISTANCE; // move forward to avoid hitting obstacle
			std::cout << state_name_[static_cast<int>(state_)] << " :: Closest waypoint between waypoints " 
					  << previous_waypoint_index+1 << " and " << next_square_waypoint_index+1 << " is "
			          << closest_waypoint_on_path[0] << " "
					  << closest_waypoint_on_path[1] << " "
					  << closest_waypoint_on_path[2] << std::endl; 
			next_waypoint_ = closest_waypoint_on_path;
		}
	}
	else if (state_ == FlightState::RETURN_TO_BASE)
	{
		next_waypoint_ = square_waypoints_[0];
	}
}

float DroneControl::dotProduct(const waypointNED& a, const waypointNED& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

waypointNED DroneControl::projectionPointOnLine(const waypointNED& waypoint_1, const waypointNED& waypoint_2, const waypointNED& waypoint_3) {
    
	waypointNED v = {waypoint_2[0] - waypoint_1[0], waypoint_2[1] - waypoint_1[1], waypoint_2[2] - waypoint_1[2]}; // p2.x - p1.x, p2.y - p1.y, p2.z - p1.z
    waypointNED w = {waypoint_3[0] - waypoint_1[0], waypoint_3[1] - waypoint_1[1], waypoint_3[2] - waypoint_1[2]}; //p3.x - p1.x, p3.y - p1.y, p3.z - p1.z

    float vDotV = dotProduct(v, v);
    float projFactor = dotProduct(w, v) / vDotV;

    waypointNED proj = {projFactor * v[0], projFactor * v[1], projFactor * v[2]};
    waypointNED intersection = {waypoint_1[0] + proj[0], waypoint_1[1] + proj[1], waypoint_1[2] + proj[2]};

    return intersection;
}

void DroneControl::odometryCallback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg)
{
	latest_odometry_ = *msg;

	std::cout << state_name_[static_cast<int>(state_)] << " :: Current xyz: " << msg->position[0] << " " << msg->position[1] << " " << msg->position[2] << " "
			  << std::endl;

	hasWaypointReached();
}

void DroneControl::distanceSensorCallback(const px4_msgs::msg::DistanceSensor::UniquePtr msg)
{
	latest_distance_data_ = *msg;

	std::cout << state_name_[static_cast<int>(state_)] << " :: ---- Current distance: " << msg->current_distance 
			  << " & Signal Quality: " << static_cast<int>(msg->signal_quality) << "%" << std::endl;

	isNearObstacle();
}

void DroneControl::arm()
{
    publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void DroneControl::disarm()
{
    publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

void DroneControl::publishOffboardControlMode()
{
	px4_msgs::msg::OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = true;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

void DroneControl::publishTrajectorySetpoint(waypointNED waypoint)
{


	std::cout << state_name_[static_cast<int>(state_)] << " :: Target xyz: " << waypoint[0] << " " << waypoint[1] << " " << waypoint[2] << std::endl;
	px4_msgs::msg::TrajectorySetpoint msg{};
	msg.position = waypoint;
	msg.velocity = {0.0, 0, 0};
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

void DroneControl::publishVehicleCommand(uint16_t command, float param1, float param2)
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
