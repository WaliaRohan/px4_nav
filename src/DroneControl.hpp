#include "rclcpp/rclcpp.hpp"
#include <stdint.h>

// Include necessary message types from px4_msgs
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/distance_sensor.hpp>

#include <chrono>
#include <iostream>

using waypointNED = std::array<float, 3>;

/**
 * @brief Flight States for defining drone behavior
 * 
 * NAVIGATE:
 * 			 Drone moves to next square waypoint. This is the default state
 * 			 and the drone starts in this state. Drone exits this state when
 * 			 an obstacle is seen. Drone enters this state again from
 * 			 "CLEAR_OBSTACLE" after an obstacle has been cleared.
 * 
 * AVOID_OBSTACLE: 
 * 			
 * 			 When the distance sensor quality threshold is crossed, the drone
 * 			 brakes. Once the drone is stopped, it starts moving right until
 * 			 the distance sensor signal quality goes below the threshold.
 * 
 * CLEAR_OBSTACLE: 
 * 
 * 			 After the drone stops detecting obstacles in "AVOID_OBSTACLE" state, it
 * 			 enters this state. The drone now tries to "clear" the obstacle by moving
 * 			 ahead. Ideally, the distance the drone moves to clear the obstacle should
 * 			 be greater than the length of the drone.
 * 
 * RETURN_TO_PATH:
 * 
 * 			 Drone enters this state from CLEAR_OBSTACLE. Once the obstacle is
 * 			 cleared, the drone tries to find the nearest point to return to 
 * 			 the original path between the last completed square waypoint and 
 * 			 the next square waypoint.
 * 
 * RETURN_TO_BASE:
 * 
 * 			 The drone enters this state from NAVIGATE once it has visited all waypoints
 * 			 for the number of laps set by TOTAL_LAPS. Once in this state, the drone 
 * 			 travels to the first waypoint and then disarms to land.
 */
enum class FlightState {
	NAVIGATE,
	AVOID_OBSTACLE,
	CLEAR_OBSTACLE,
	RETURN_TO_PATH,
	RETURN_TO_BASE
};

/**
 * @brief This class defines the obstacle avoidance behavior for a px4 iris with
 * 	      a single front mounted lidar. 
 */
class DroneControl : public rclcpp::Node {
public:
    DroneControl();
    void arm();
	void disarm();

private:

	// Data members
    std::vector<waypointNED> square_waypoints_;
	waypointNED next_waypoint_; // drone is always following this waypoint. It can be set by any state.
	int last_requested_waypoint_index_ = -1; // for square waypoints only (zero based indexing, -1 means no waypoints requested since beginning of node)
	int lap_ = 1; // lap counter
	float last_positive_obstacle_distance_ = 0.0; // keeps track of non-zero distance from last scene obstacle in AVOID_OBSTACLE mode
	bool request_clear_obstacle_ = false; // flag for checking if waypoint for clearing obstacle has been set

	std::vector<std::string> state_name_ = {"NAVIGATE", "AVOID OBSTACLE", "CLEAR OBSTACLE", "RETURN TO PATH", "RETURN TO BASE"};

	// Flags and constants
	float POSITION_ERROR_TOLERANCE = 1.0; // m
	float DISTANCE_SIGNAL_QUALITY_THRESHOLD = 1; // decrease this threshold with increase in MPC_XY_VEL_MAX param
	float STOP_VELOCITY_THRESHOLD = 0.1; // In m/s. If odometry reported velocity is less than this value, drone was stopped.
	float CLEAR_DISTANCE = 2.0; // In m -> Distance to clear in forward direction before switching from AVOID_OBSTACLE to NAVIGATE mode.
								// Should be atleast the length of the drone
	bool DRONE_WAS_STOPPED = false; // flag to check if drone was stopped after encountering obstacle
	int TOTAL_LAPS = 3;
	bool set_zero_velocity_ = false; // flag to set zero velocity for trajectory
	bool DISARM = false; // flag to tell the drone whether to 

	FlightState state_ = FlightState::NAVIGATE; // Start node in navigatio mode

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

	/**
	 * @brief Create predefined square waypoints
	 * 
	 */
	void initializeWaypoints();

	/**
	 * @brief Check if drone is within POSITION_ERROR_TOLERANCE of next_waypoint_
	 * 		  Update last_request_waypoint_index_ if drone reached last requested
	 * 		  square waypoint.
	 * 
	 * @return true If last_waypoint_ was reached within POSITION_ERROR_TOLERANCE
	 * @return false otherwise
	 */
	bool hasWaypointReached();

	/**
	 * @brief Read distance sensor data to check if drone is near an obstacle. This
	 * 	      method handles state transitions.
	 * 
	 * @return true If drone is near obstacle
	 * @return false otherwise
	 */
	bool isNearObstacle();

	/**
	 * @brief Calculate and update next_waypoint_ based on state and obstacle information
	 * 
	 */
	void calculateNextWaypoint();

	/**
	 * @brief Calculate dot product of two waypoints vectors. Helper method for "projectionPointOnLine"
	 * 
	 * @param a Waypoint vector 1
	 * @param b Waypoint vector 2
	 * @return float 
	 */
	float dotProduct(const waypointNED& a, const waypointNED& b);

	/**
	 * @brief This function calculates the nearest point to the original line 
	 * 		  between two square points. For instance, if the drone deviates 
	 * 		  from its intended path due to avoiding an obstacle and ends up at 
	 * 		  (4, 1, 0) ("stray waypoint"), while it was supposed to travel from
	 * 		  square waypoint (2, 0, 0) to square waypoint (5, 0, 0), this function
	 *        determines the "projectionPointOnLine" as the perpendicular intersection
	 *        of (4, 1, 0) and the line defined by (2, 0, 0) and (5, 0, 0).
	 * 		  The resulting intersection point is (4, 0, 0).
	 *       
	 * @param waypoint_1 Stray waypoint
	 * @param waypoint_2 Square waypoint 1
	 * @param waypoint_3 Square waypoint 2 
	 * @return waypointNED 
	 */
	waypointNED projectionPointOnLine(const waypointNED& waypoint_1,
									 const waypointNED& waypoint_2,
									 const waypointNED& waypoint_3);
    
	// Subscriber callbacks
	void odometryCallback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg);
	void distanceSensorCallback(const px4_msgs::msg::DistanceSensor::UniquePtr msg);

	// Publisher callbacks
	void publishOffboardControlMode();
	void publishTrajectorySetpoint(waypointNED waypoint);
	void publishVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0);	
};