/**
 * @file joystick_controller.hpp
 * 
 * @brief implementation of the JoystickController class
 * 
 * @author: t-tang-rfc
 * 
 * @details:
 * All joystick control logic is implemented here.
 * This class sends pose information to the GUI thread for rendering.
 * 
 * @date: [created: 2025-06-06, updated: 2025-08-18]
 **/

#include "joystick_controller.hpp"
#include <cmath>
#include <iostream>

namespace rf {

JoystickController::JoystickController(QObject* parent)
	: QObject(parent)
	, throttle_(0.0)
	, xdd_(0.0)
	, xd_(0.0)
	, torque_(0.0) // Initialize roll torque
	, rdd_(0.0) // Initialize roll angular acceleration
	, rd_(0.0) // Initialize roll angular velocity
	, r_(0.0) // Initialize roll angle
	, yaw_torque_(0.0)
	, yaw_dd_(0.0)
	, yaw_d_(0.0)
	, pitch_ref_(0.0) // Initialize pitch reference
	, pitch_(0.0) // Initialize pitch angle
	, pitch_rate_(0.0) // Initialize pitch rate
	, mode_volume_(M_PI * (20.0 * 20.0) * 160.0) // V = π * r^2 * h, r = 20 cm, h = 160 cm
	, mode_density_(0.01) // Density in g/cm^3
	, mode_mass_(mode_volume_ * mode_density_ * 0.001) // Convert to kg
	, max_thrust_(12.0)
	, static_drag_(4.0)
	, quadratic_drag_k_(0.5)
	, model_inertia_(0.1) // Moment of inertia for a rod: I = (1/12) * m * L^2, approximated
	, max_roll_torque_(5.0) // Maximum roll torque, [N*m]
	, static_friction_(2.0) // Static rotational friction, [N*m]
	, quadratic_friction_k_(0.3) // Dynamic rotational friction parameter, [N*m/(rad/s)^2]
	, max_yaw_torque_(5.0) // Maximum yaw torque, [N*m]
	, yaw_static_friction_(2.0) // Static yaw friction, [N*m]
	, yaw_quadratic_friction_k_(0.3) // Dynamic yaw friction parameter, [N*m/(rad/s)^2]
	, pitch_Kp_(4.0)
	, pitch_Kd_(-0.3)
	, max_pitch_rate_(2.0)
	, dt_(1.0 / 50.0)
	, nh_(ros::NodeHandle())
	, joy_sub_(nh_.subscribe("joy", 10, &JoystickController::joyCallback, this))
	, ctrl_timer_(nh_.createTimer(ros::Duration(dt_), &JoystickController::updateModel, this))	
{
	// Print out parameters
	std::cout << "Mode volume: " << mode_volume_ << " cm^3"
			  << ", Density: " << mode_density_ << " g/cm^3"
			  << ", Mass: " << mode_mass_ << " kg"
			  << ", Max force: " << max_thrust_ << " N"
			  << ", Static drag: " << static_drag_ << " N"
			  << ", Quadratic drag k: " << quadratic_drag_k_ << " N/(m/s)^2"
			  << ", Model inertia: " << model_inertia_ << " kg*m^2"
			  << ", Max roll torque: " << max_roll_torque_ << " N*m"
			  << ", Static friction: " << static_friction_ << " N*m"
			  << ", Quadratic friction k: " << quadratic_friction_k_ << " N*m/(rad/s)^2"
			  << ", Max yaw torque: " << max_yaw_torque_ << " N*m"
			  << ", Yaw static friction: " << yaw_static_friction_ << " N*m"
			  << ", Yaw quadratic friction k: " << yaw_quadratic_friction_k_ << " N*m/(rad/s)^2"
			  << ", Pitch Kp: " << pitch_Kp_ << ", Pitch Kd: " << pitch_Kd_ << std::endl;
}

JoystickController::~JoystickController() = default;

void JoystickController::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
	// [1] Read joystick control
	float left_stick_hoz = msg->axes[0]; // suppose from Left stick horizontal axis
	float left_stick_vet = msg->axes[1]; // suppose from Left stick vertical axis
	float right_stick_vet = msg->axes[5]; // suppose from Right stick vertical axis
	float left_trigger_val = msg->axes[3]; // suppose from L2
	int right_bumper_val = msg->buttons[5]; // suppose from R1
	float right_trigger_val = msg->axes[4]; // suppose from R2
	int left_bumper_val = msg->buttons[4]; // suppose from L1
	
	// [2] Derive control values
	// Forward/backward throttle: right bumper (forward/reverse) + left trigger (throttle amount)
	throttle_ = (right_bumper_val < 1 ? 1.0 : -1.0) * (1.0 - left_trigger_val) * 0.5;

	// Roll torque control: left bumper (roll direction) + right trigger (torque amount)
	float roll_throttle = (left_bumper_val < 1 ? 1.0 : -1.0) * (1.0 - right_trigger_val) * 0.5;
	torque_ = roll_throttle * max_roll_torque_; // Roll torque, [N*m]

	// Yaw torque control: left stick horizontal (positive = counter-clockwise around Z-axis)
	float yaw_input = deadzone(left_stick_hoz, 0.05);
	yaw_torque_ = yaw_input * max_yaw_torque_; // Yaw torque, [N*m]	

	// derive pitch reference from right stick vertical axis
	// Map right stick vertical (-1.0 to 1.0) to pitch (-180° to +180°)
	float right_stick_vertical = deadzone(right_stick_vet, 0.05);
	pitch_ref_ = right_stick_vertical * M_PI / 2.0; // Scale to [-π/2, π/2] radians
}

void JoystickController::updateModel(const ros::TimerEvent& event)
{
	// --- Update state ---

	// [1] Update linear motion dynamics (in local coordinates)

	// Compute current velocity by the *last* acceleration
	float vel_x = xd_ + xdd_ * dt_; // [m/s]
	// Update velocity
	xd_ = vel_x;

	// [2] Update physics-based linear motion

	// Compute motor force
	float F_mot = throttle_ * max_thrust_; // Motor force, [N]

	// Compute static friction
	float f_static = (sign(xd_) == 0.0) ? clamp(-F_mot, -static_drag_, static_drag_) : (-sign(xd_) * static_drag_);

	// Compute dynamic friction
	float f_dynamic = -sign(xd_) * (std::pow(std::abs(xd_), 2) * quadratic_drag_k_);

	// Compute & Update instant acceleration
	xdd_ = (F_mot + f_static + f_dynamic) / mode_mass_; // [m/s^2]

	// [3] Update roll dynamics (physics-based model)
	
	// Compute current angular velocity by the *last* angular acceleration
	float ang_vel_r = rd_ + rdd_ * dt_; // [rad/s]
	// Update angular velocity
	rd_ = ang_vel_r;

	// Compute static rotational friction
	float f_static_rot = (sign(rd_) == 0.0) ? clamp(-torque_, -static_friction_, static_friction_) : (-sign(rd_) * static_friction_);

	// Compute dynamic rotational friction
	float f_dynamic_rot = -sign(rd_) * (std::pow(std::abs(rd_), 2) * quadratic_friction_k_);

	// Compute & Update instant angular acceleration
	rdd_ = (torque_ + f_static_rot + f_dynamic_rot) / model_inertia_; // [rad/s^2]

	// [3.5] Update yaw dynamics (physics-based model)
	
	// Compute current yaw angular velocity by the *last* angular acceleration
	float yaw_ang_vel = yaw_d_ + yaw_dd_ * dt_; // [rad/s]
	// Update yaw angular velocity
	yaw_d_ = yaw_ang_vel;

	// Compute static yaw rotational friction
	float f_static_yaw = (sign(yaw_d_) == 0.0) ? clamp(-yaw_torque_, -yaw_static_friction_, yaw_static_friction_) : (-sign(yaw_d_) * yaw_static_friction_);

	// Compute dynamic yaw rotational friction
	float f_dynamic_yaw = -sign(yaw_d_) * (std::pow(std::abs(yaw_d_), 2) * yaw_quadratic_friction_k_);

	// Compute & Update instant yaw angular acceleration
	yaw_dd_ = (yaw_torque_ + f_static_yaw + f_dynamic_yaw) / model_inertia_; // [rad/s^2]

	// [5] Update pitch rate (attitude control)
	float pitch_diff = pitch_ref_ - pitch_;
	while (pitch_diff > M_PI) pitch_diff -= 2 * M_PI; // Normalize to [-π, π]
	while (pitch_diff < -M_PI) pitch_diff += 2 * M_PI;

	float pitch_cmd_rate = clamp(pitch_Kp_ * pitch_diff + pitch_Kd_ * pitch_rate_, -max_pitch_rate_, max_pitch_rate_);
	pitch_ += pitch_cmd_rate * dt_; // Update pitch, [rad]
	pitch_rate_ = pitch_cmd_rate;   // Update pitch rate, [rad/s]

	// [6] Build incremental transform for this timestep
	Eigen::Matrix4f step = Eigen::Matrix4f::Identity();

	// 6a) Translate along local +X by velocity increment (not absolute position!)
	// float delta_x = xd_ * dt_; // Incremental displacement this timestep
	// step(0, 3) = delta_x;

	// 6b) Apply incremental rotations (using angular velocities, not absolute angles)
	float delta_roll = rd_ * dt_;     // Incremental roll rotation
	float delta_yaw = yaw_d_ * dt_;      // Incremental yaw rotation (physics-based)  
	float delta_pitch = pitch_rate_ * dt_; // Incremental pitch rotation

	// Create rotation matrices for each axis
	// Eigen::Matrix3f R_roll = Eigen::AngleAxisf(delta_roll, Eigen::Vector3f::UnitX()).toRotationMatrix();
	Eigen::Matrix3f R_yaw = Eigen::AngleAxisf(delta_yaw, Eigen::Vector3f::UnitY()).toRotationMatrix();
	// Eigen::Matrix3f R_pitch = Eigen::AngleAxisf(delta_pitch, Eigen::Vector3f::UnitY()).toRotationMatrix();

	// Combine rotations (order: yaw * pitch * roll for proper composition)
	// Eigen::Matrix3f R_combined = R_yaw * R_pitch * R_roll;
	step.block<3,3>(0, 0) = R_yaw;

	// [7] Emit the instantaneous transformation to the delegate, will handle pose accumulation
	Q_EMIT applyTransform(step);
}

// --- Utility function implementations ---

float JoystickController::deadzone(float in, float tol) const
{
	if (std::abs(in) < tol)
		return 0.0;
	else
		return in;
}

float JoystickController::sign(float x, float threshold) const
{
	if (x > threshold)
		return 1.0;
	else if (x < -threshold)
		return -1.0;
	else
		return 0.0;
}

float JoystickController::clamp(float x, float min_val, float max_val) const
{
	return std::max(min_val, std::min(max_val, x));
}

} // namespace rf
