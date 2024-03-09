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
 * 			 an obstacle is seen. Drone enters this state again from "TRANSITION_TO_NAVIGATE"
 * 			 after an obstacle has been cleared.
 * 
 * AVOID_OBSTACLE: 
 * 			
 * 			 Drone moves up until obstacle is cleared. This state is triggered when
 * 			 distance and quality of signal data from distance sensor cross the
 * 		     values set by DISTANCE_ERROR_THRESHOLD and DISTANCE_SIGNAL_QUALITY_THRESHOLD.
 * 
 * TRANSITION_TO_NAVIGATE: 
 * 
 * 			 After the drone stops detection obstacles in "AVOID_OBSTACLE" state, it
 * 			 enters this state. The drone now tries to "clear" the obstacle by moving
 * 			 up and forward by preset distance values. Ideally, the distances moved up
 * 			 forward should be greater than the size of the bounding box of the drone.
 */
enum class FlightState {
	NAVIGATE,
	AVOID_OBSTACLE,
	TRANSITION_TO_NAVIGATE
};

/**
 * @brief This class defines the obstacle avoidance behavior for a px4 iris with
 * 	      a single front mounted lidar. 
 * 
 * 	Subscribers:
 * 			px4_msgs::msg::VehicleOdometry latest_odometry_;
 * 			px4_msgs::msg::DistanceSensor latest_distance_data_;
 * 			rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_subscriber_ ()
 * 			rclcpp::Subscription<px4_msgs::msg::DistanceSensor>::SharedPtr distance_sensor_subscriber_ ()
 * 
 * Publishers:
 * 			rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_ ()
 * 			rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_ ()
 * 			rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_ ()
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
	int lap_ = 0;
	

	// Flags and constants
	float POSITION_ERROR_TOLERANCE = 1.0; // m
	float DISTANCE_ERROR_THRESHOLD = 5.0; // m
	float DISTANCE_SIGNAL_QUALITY_THRESHOLD = 1; // decrease this threshold with increase in MPC_XY_VEL_MAX param
	float STOP_VELOCITY_THRESHOLD = 0.1; // In m/s. If odometry reported velocity is less than this value, drone was stopped.
	float CLEAR_DISTANCE = 1.0; // In m -> Distance to clear in forward direction before switching from AVOID_OBSTACLE to NAVIGATE mode.
								// Should be atleast the length of the drone
	bool DRONE_WAS_STOPPED = false; // flag to check if drone was stopped after encountering obstacle
	int TOTAL_LAPS = 3;
	bool set_zero_velocity_ = false; // flag to set zero velocity for trajectory
	bool DISARM = false;

	FlightState state = FlightState::NAVIGATE; // Start node in navigatio mode

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
	 * @param waypoint_1 Square waypoint 1
	 * @param waypoint_2 Square waypoint 2
	 * @param waypoint_3 Stray waypoint
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