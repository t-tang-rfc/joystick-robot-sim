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
 * - Left stick left/right -- rotate around X axis, i.e., rolling left/right
 * - Right stick forward/backward -- rotate around Y axis, i.e., pitching up/down
 * 
 * @@note:
 * - The controller class should have the FULL model of the object to be controlled.
 * - It is the core of the robot simulation, and it is *physics driven*.
 * 
 * @date: [created: 2025-06-06, updated: 2025-08-18]
 **/

#ifndef JOYSTICK_CONTROLLER_HPP
#define JOYSTICK_CONTROLLER_HPP

#include <QObject>
#include <QtGlobal>
#include <QList>
#include <QMatrix4x4>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <algorithm>

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
		float x_;                 // position of the robot along local x-axis, [m]
		float torque_;            // torque applied to roll, [N*m]
		float rdd_;               // angular acceleration around local x-axis, [rad/s^2]
		float rd_;                // angular velocity around local x-axis, [rad/s]
		float r_;                 // roll angle, [rad]
		float yaw_torque_;        // torque applied to yaw, [N*m]
		float ydd_;               // angular acceleration around local z-axis, [rad/s^2]
		float yd_;                // angular velocity around local z-axis, [rad/s]
		float y_;                 // yaw angle, [rad]
		float pitch_ref_;         // target pitch angle, [rad]
		float pitch_;             // current pitch angle, [rad]
		float pitch_rate_;        // angular velocity, [rad/s]
		float yaw_ref_;           // target heading direction, [rad] (kept for compatibility)
		float yaw_;               // heading direction, [rad] (kept for compatibility)
		float yaw_rate_;          // angular velocity, [rad/s] (kept for compatibility)

		QList<float> robot_pose_; // (x, y, z, roll, pitch, yaw) in [cm] and [degrees]

		// --- Parameters ---
		// @note: the object to be controlled is modeled as a rod stick, with diameter 40 cm and length 160 cm (the unit [cm] is chosen to be compatible with Qt Quick 3D)
		float mode_volume_;       // [cm^3]
		float mode_density_;      // [g/cm^3]
		float mode_mass_;         // [kg]
		float max_thrust_;        // [N]
		float static_drag_;       // static friction, [N]
		float quadratic_drag_k_;  // dynamic friction parameter, [N/(m/s)^2]
		float model_inertia_;     // moment of inertia, [kg*m^2]
		float max_roll_torque_;   // maximum torque around local x-axis, [N*m]
		float static_friction_;   // static friction, [N]
		float quadratic_friction_k_; // dynamic friction parameter, [N/(m/s)^2]
		float max_yaw_torque_;    // maximum torque around local z-axis, [N*m]
		float yaw_static_friction_;   // static yaw friction, [N*m]
		float yaw_quadratic_friction_k_; // dynamic yaw friction parameter, [N*m/(rad/s)^2]
		float yaw_Kp_;            // proportional gain for yaw control, [rad/(rad/s)]
		float yaw_Kd_;            // derivative gain for yaw control, [rad/(rad/s)^2]
		float max_yaw_rate_;      // maximum yaw rate, [rad/s]
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
		float deadzone(float in, float tol) const
		{
			if (std::abs(in) < tol)
				return 0.0;
			else
				return in;
		}

		float sign(float x, float threshold = 1e-5) const
		{
			if (x > threshold)
				return 1.0;
			else if (x < -threshold)
				return -1.0;
			else
				return 0.0;
		}

		float clamp(float x, float min_val, float max_val) const
		{
			return std::max(min_val, std::min(max_val, x));
		}
};

} // namespace rf

#endif // JOYSTICK_CONTROLLER_HPP
