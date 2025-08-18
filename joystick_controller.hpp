/**
 * @file joystick_controller.hpp
 * 
 * @brief Joystick controller for robot simulation
 * 
 * @author: t-tang-rfc
 * 
 * @details:
 * - Left trigger (LT) -- throttle, move along X axis, i.e., forward thrust
 * - Right bumpper (RB) -- 
 * - Right stick left/right -- move along Z axis, i.e., left/right movement
 * - Left stick left/right -- yaw (stick left for yaw left)
 * - Right stick forward/backward -- pitch (stick forward for pitch up)
 * 
 * @date: [created: 2025-06-06, updated: 2025-08-18]
 **/

#ifndef JOYSTICK_CONTROLLER_HPP
#define JOYSTICK_CONTROLLER_HPP

#include <QObject>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

namespace rf {

class JoystickController : public QObject
{
	Q_OBJECT

	public:
		explicit JoystickController(QObject* parent = nullptr);
		~JoystickController();

		Q_SIGNAL void applyTransform(Eigen::Matrix4f transform);

	private:
		// ---- State ----
		float throttle_;          // throttle value from joystick, [0, 1]
		float xdd_;               // acceleration of the robot, [m/s^2]
		float xd_;                // velocity of the robot, [m/s]
		float torque_;            // torque applied to roll, [N*m]
		float rdd_;               // angular acceleration around local x-axis, [rad/s^2]
		float rd_;                // angular velocity around local x-axis, [rad/s]
		float r_;                 // roll angle, [rad]
		float yaw_torque_;        // yaw torque, [N*m]
		float yaw_dd_;            // yaw angular acceleration, [rad/s^2]
		float yaw_d_;             // yaw angular velocity, [rad/s]
		float pitch_ref_;         // target pitch angle, [rad]
		float pitch_;             // current pitch angle, [rad]
		float pitch_rate_;        // angular velocity, [rad/s]

		// --- Parameters ---
		// @note: the object to be controlled is modeled as a rod stick, with longitudinal axis being the roll axis.
		float mode_volume_;       // [cm^3]
		float mode_density_;      // [g/cm^3]
		float mode_mass_;         // [kg]
		float max_thrust_;        // [N]
		float static_drag_;       // static friction, [N]
		float quadratic_drag_k_;  // dynamic friction parameter, [N/(m/s)^2]
		float roll_inertia_;      // moment of inertia around roll axis (longitudinal), [kg*m^2]
		float pitch_inertia_;     // moment of inertia around pitch axis (transverse), [kg*m^2]
		float yaw_inertia_;       // moment of inertia around yaw axis (transverse), [kg*m^2]
		float max_roll_torque_;   // maximum torque around local x-axis, [N*m]
		float static_friction_;   // static friction, [N]
		float quadratic_friction_k_; // dynamic friction parameter, [N/(m/s)^2]
		float max_yaw_torque_;    // maximum torque around local z-axis, [N*m]
		float yaw_static_friction_;   // static yaw friction, [N*m]
		float yaw_quadratic_friction_k_; // dynamic yaw friction parameter, [N*m/(rad/s)^2]
		float pitch_Kp_;          // proportional gain for pitch control, [rad/(rad/s)]
		float pitch_Kd_;          // derivative gain for pitch control, [rad/(rad/s)^2]
		float max_pitch_rate_;    // maximum pitch rate, [rad/s]
		float dt_;                // time step for the physics model, [s]

		// ---- ROS ----
		ros::NodeHandle nh_;
		ros::Subscriber joy_sub_;
		ros::Timer ctrl_timer_;

		// ---- Callbacks ----
		void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
		void updateModel(const ros::TimerEvent& event);		

		// --- Utility functions ---
		float deadzone(float in, float tol) const;
		float sign(float x, float threshold = 1e-5) const;
		float clamp(float x, float min_val, float max_val) const;
};

} // namespace rf

#endif // JOYSTICK_CONTROLLER_HPP
