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
 * @date: [created: 2025-06-06, updated: 2025-07-30]
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
	, x_(0.0) // @todo: initialize x_ based on the robot's initial position
	, torque_(0.0) // Initialize roll torque
	, rdd_(0.0) // Initialize roll angular acceleration
	, rd_(0.0) // Initialize roll angular velocity
	, r_(0.0) // Initialize roll angle
	, pitch_ref_(0.0) // Initialize pitch reference
	, pitch_(0.0) // Initialize pitch angle
	, pitch_rate_(0.0) // Initialize pitch rate
	, yaw_(0.0)
	, yaw_rate_(0.0)
	, transform_()
	, robot_pose_({0, 0, 0, 0, 0, 0}) // Initialize robot pose to zero
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
	, yaw_Kp_(4.0)
	, yaw_Kd_(-0.3)
	, max_yaw_rate_(2.0)
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
			  << ", Yaw Kp: " << yaw_Kp_ << ", Yaw Kd: " << yaw_Kd_
			  << ", Pitch Kp: " << pitch_Kp_ << ", Pitch Kd: " << pitch_Kd_ << std::endl;

	// Check std::atan2(0.0, 0.0) behavior
	float test_yaw = std::atan2(0.0, 0.0);
	std::cout << "atan2(0.0, 0.0) = " << test_yaw << " (should be 0.0)" << std::endl;
	transform_.setToIdentity();
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

	float ux = deadzone(left_stick_hoz, 0.05);
	float uz = deadzone(left_stick_vet, 0.05);
	
	// @note: atan2(0.0, 0.0) return 0.0, which is the desired behavior here
	yaw_ref_ = std::atan2(ux, uz); // [rad]
	// std::cout << "ux=" << ux << ", uz=" << uz
	//           << ", yaw_ref=" << yaw_ref_ * 180.0 / M_PI << " degrees" << std::endl;

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

	// [4] Update yaw rate (attitude control)
	float yaw_diff = yaw_ref_ - yaw_;
	while (yaw_diff > M_PI) yaw_diff -= 2 * M_PI; // Normalize to [-π, π]
	while (yaw_diff < -M_PI) yaw_diff += 2 * M_PI;

	float yaw_cmd_rate = clamp(yaw_Kp_ * yaw_diff + yaw_Kd_ * yaw_rate_, -max_yaw_rate_, max_yaw_rate_);
	yaw_ += yaw_cmd_rate * dt_; // Update yaw, [rad]
	yaw_rate_ = yaw_cmd_rate;   // Update yaw rate, [rad/s]

	// [5] Update pitch rate (attitude control)
	float pitch_diff = pitch_ref_ - pitch_;
	while (pitch_diff > M_PI) pitch_diff -= 2 * M_PI; // Normalize to [-π, π]
	while (pitch_diff < -M_PI) pitch_diff += 2 * M_PI;

	float pitch_cmd_rate = clamp(pitch_Kp_ * pitch_diff + pitch_Kd_ * pitch_rate_, -max_pitch_rate_, max_pitch_rate_);
	pitch_ += pitch_cmd_rate * dt_; // Update pitch, [rad]
	pitch_rate_ = pitch_cmd_rate;   // Update pitch rate, [rad/s]

	// [6] Build incremental transform for this timestep
	QMatrix4x4 step;
	step.setToIdentity();

	// 6a) Translate along local +X by velocity increment (not absolute position!)
	float delta_x = xd_ * dt_; // Incremental displacement this timestep
	step.translate(delta_x, 0.0, 0.0);

	// 6b) Apply incremental rotations (using angular velocities, not absolute angles)
	float delta_roll = rd_ * dt_;     // Incremental roll rotation
	float delta_yaw = yaw_rate_ * dt_; // Incremental yaw rotation  
	float delta_pitch = pitch_rate_ * dt_; // Incremental pitch rotation

	//     - roll around X axis
	step.rotate(qRadiansToDegrees(delta_roll), QVector3D(1,0,0));
	//     - yaw around Y axis  
	step.rotate(qRadiansToDegrees(delta_yaw), QVector3D(0,1,0));
	//     - pitch around Z axis
	step.rotate(qRadiansToDegrees(delta_pitch), QVector3D(0,0,1));

	// 6c) Accumulate into world transform (this gives us proper 6-DOF composition)
	transform_ = transform_ * step;

	// [6] Decompose world transform → translation + Euler angles
	// Extract translation from the transform matrix
	QVector3D t = transform_.column(3).toVector3D();
	double world_x = t.x();
	double world_y = t.y();
	double world_z = t.z();

	// Extract orientation as quaternion → Euler (roll, pitch, yaw)
	QMatrix3x3 rotMat;
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 3; ++j)
			rotMat(i, j) = transform_(i, j);
	QQuaternion q = QQuaternion::fromRotationMatrix(rotMat);
	// Note: Qt's toEulerAngles() returns (x=roll, y=pitch, z=yaw) in degrees
	QVector3D euler = q.toEulerAngles();

	// [7] Populate robot_pose_ for Qt renderer (positions in cm, angles in degrees)
	robot_pose_[0] = world_x * 100.0;  // World X position, convert to cm
	robot_pose_[1] = world_y * 100.0;  // World Y position, convert to cm
	robot_pose_[2] = world_z * 100.0;  // World Z position, convert to cm
	robot_pose_[3] = euler.x();        // Roll angle in degrees
	robot_pose_[4] = euler.y();        // Pitch angle in degrees
	robot_pose_[5] = euler.z();        // Yaw angle in degrees

	Q_EMIT setPose(robot_pose_);
}

} // namespace rf
