/**
 * @file joystick_controller.hpp
 * 
 * @brief Joystick controller for robot simulation
 * 
 * @author: t-tang-rfc
 * 
 * @details:
 * - Right trigger -- roll (w/ left bumper for reverse)
 * - Left trigger -- pitch (w/ right bumper for reverse)
 * - Left stick left/right -- yaw (stick left for yaw left)
 * - Right stick forward/backward -- translation in heading direction (stick forward for forward motion)
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
		// ---- Parameters ----
		float dt_;                     // time step for the physics model, [s]
		// Roll
		float roll_max_torque_;        // maximum torque around roll axis [N*m]		
		float roll_torque_;            // torque applied to roll [N*m]
		float roll_inertia_;           // moment of inertia of roll axis [kg*m^2]
		float roll_static_friction_;   // static friction for roll [N*m]
		float roll_2nd_friction_k_;    // dynamic friction parameter [N*m/(rad/s)^2]			
		float roll_dd_;                // angular acceleration around roll axis [rad/s^2]
		float roll_d_;                 // angular velocity around roll axis [rad/s]		
		// Pitch
		float pitch_max_torque_;       // maximum torque around pitch axis [N*m]
		float pitch_torque_;           // pitch torque [N*m]
		float pitch_inertia_;          // moment of inertia of pitch axis [kg*m^2]		
		float pitch_static_friction_;  // static pitch friction [N*m]
		float pitch_2nd_friction_k_;   // dynamic pitch friction parameter [N*m/(rad/s)^2]
		float pitch_dd_;               // pitch angular acceleration [rad/s^2]
		float pitch_d_;                // pitch angular velocity [rad/s]		
		// Yaw
		float yaw_max_torque_;         // maximum torque around yaw axis [N*m]
		float yaw_torque_;             // yaw torque [N*m]
		float yaw_inertia_;            // moment of inertia of yaw [kg*m^2]		
		float yaw_static_friction_;    // static yaw friction [N*m]
		float yaw_2nd_friction_k_;     // dynamic yaw friction parameter [N*m/(rad/s)^2]		
		float yaw_dd_;                 // yaw angular acceleration [rad/s^2]
		float yaw_d_;                  // yaw angular velocity [rad/s]
		// Translation
		float x_max_force_;            // maximum linear force applied to the robot [N]
		float x_force_;                // force applied to translate the robot [N]
		float x_mode_mass_;            // mass of the robot [kg]
		float x_static_friction_;      // static friction for translation [N]
		float x_2nd_friction_k_;       // dynamic friction parameter [N/(m/s)^2]
		float x_dd_;                   // acceleration of the robot, [m/s^2]
		float x_d_;                    // velocity of the robot, [m/s]

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
