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

namespace {

const float ROT_VEL_THRESH = 0.04f; // [rad/s]
const float LIN_VEL_THRESH = 0.02f; // [m/s]

}

namespace rf {

JoystickController::JoystickController(QObject* parent)
	: QObject(parent)
	, roll_max_torque_(3.0)
	, roll_torque_(0.0)
	, roll_inertia_(0.5)
	, roll_static_friction_(2.0)
	, roll_2nd_friction_k_(0.8)
	, roll_dd_(0.0)
	, roll_d_(0.0)	
	, pitch_max_torque_(5.0)
	, pitch_torque_(0.0)
	, pitch_inertia_(1.0)
	, pitch_static_friction_(2.0)
	, pitch_2nd_friction_k_(1.0)
	, pitch_dd_(0.0)
	, pitch_d_(0.0)
	, yaw_max_torque_(5.0)
	, yaw_torque_(0.0)
	, yaw_inertia_(1.0)
	, yaw_static_friction_(2.0)
	, yaw_2nd_friction_k_(1.0)
	, yaw_dd_(0.0)
	, yaw_d_(0.0)
	, x_max_force_(12.0)
	, x_force_(0.0)
	, x_mode_mass_(2.0)
	, x_static_friction_(4.0)
	, x_2nd_friction_k_(0.5)
	, x_dd_(0.0)
	, x_d_(0.0)
	, dt_(1.0 / 50.0)
	, nh_(ros::NodeHandle())
	, joy_sub_(nh_.subscribe("joy", 10, &JoystickController::joyCallback, this))
	, ctrl_timer_(nh_.createTimer(ros::Duration(dt_), &JoystickController::updateModel, this))	
{}

JoystickController::~JoystickController() = default;

void JoystickController::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
	// [0] Read joystick control
	float right_trigger_val = msg->axes[4]; // suppose from R2
	int left_bumper_val = msg->buttons[4];  // suppose from L1
	float left_trigger_val = msg->axes[3];  // suppose from L2
	int right_bumper_val = msg->buttons[5]; // suppose from R1
	float left_stick_hoz = msg->axes[0];    // suppose from Left stick horizontal axis
	float right_stick_vet = msg->axes[5];   // suppose from Right stick vertical axis

	// [1] Roll torque control: left bumper (roll direction) + right trigger (torque amount)
	float roll_input = (left_bumper_val < 1 ? 1.0 : -1.0) * (1.0 - right_trigger_val) * 0.5;
	roll_torque_ = roll_input * roll_max_torque_; // Roll torque, [N*m]

	// [2] Pitch torque control: right bumper (pitch direction) + left trigger (torque amount)
	float pitch_input = (right_bumper_val < 1 ? 1.0 : -1.0) * (1.0 - left_trigger_val) * 0.5;
	pitch_torque_ = pitch_input * pitch_max_torque_; // Pitch torque, [N*m]

	// [3] Yaw torque control: left stick horizontal
	float yaw_input = deadzone(left_stick_hoz, 0.05);
	yaw_torque_ = yaw_input * yaw_max_torque_; // Yaw torque [N*m]

	// [4] Translation force control: right stick vertical
	float x_input = deadzone(right_stick_vet, 0.05);
	x_force_ = x_input * x_max_force_; // Translation force, [N]
}

void JoystickController::updateModel(const ros::TimerEvent& event)
{
	// [0] Build incremental transform for this timestep
	Eigen::Matrix4f step = Eigen::Matrix4f::Identity();

	// [1] Update roll dynamics (physics-based model)
	
	// Compute current angular velocity by the *last* angular acceleration
	float roll_vel = roll_d_ + roll_dd_ * dt_; // [rad/s]
	// Update angular velocity
	roll_d_ = roll_vel;

	// Compute friction forces (corrected physics with numerical stability)
	float roll_friction = 0.0f;
	if (std::abs(roll_d_) < ROT_VEL_THRESH && std::abs(roll_torque_) < roll_static_friction_) {
		// Static friction region: opposes applied torque and damps velocity
		roll_friction = clamp(-roll_torque_, -roll_static_friction_, roll_static_friction_);
		// Add velocity damping to eliminate numerical oscillations
		roll_friction -= roll_d_ * (roll_static_friction_ / ROT_VEL_THRESH); // Strong damping near zero

		// If the total force would cause velocity to overshoot zero, clamp it
		float predicted_roll_vel = roll_d_ + (roll_torque_ + roll_friction) / roll_inertia_ * dt_;
		if (roll_d_ * predicted_roll_vel < 0) { // Velocity would change sign
			// Apply exact force needed to bring velocity to zero
			roll_friction = -roll_torque_ - roll_d_ * roll_inertia_ / dt_;
		}
	} else {
		// Kinetic friction: always opposes motion, magnitude depends on velocity
		float coulomb_friction = roll_static_friction_ * 0.8; // Kinetic < static typically
		roll_friction = -sign(roll_d_) * coulomb_friction - sign(roll_d_) * std::pow(std::abs(roll_d_), 2) * roll_2nd_friction_k_;
	}

	// Compute & Update instant angular acceleration
	roll_dd_ = (roll_torque_ + roll_friction) / roll_inertia_; // [rad/s^2] - using roll inertia

	// [5] Update pitch dynamics (physics-based model)
	
	// Compute current pitch angular velocity by the *last* angular acceleration
	float pitch_vel = pitch_d_ + pitch_dd_ * dt_; // [rad/s]
	// Update pitch angular velocity
	pitch_d_ = pitch_vel;

	// Compute friction forces (corrected physics with numerical stability)
	float friction_pitch = 0.0f;	
	if (std::abs(pitch_d_) < ROT_VEL_THRESH && std::abs(pitch_torque_) < pitch_static_friction_) {
		// Static friction region: opposes applied torque and damps velocity
		friction_pitch = clamp(-pitch_torque_, -pitch_static_friction_, pitch_static_friction_);
		// Add velocity damping to eliminate numerical oscillations
		friction_pitch -= pitch_d_ * (pitch_static_friction_ / ROT_VEL_THRESH); // Strong damping near zero
		
		// If the total force would cause velocity to overshoot zero, clamp it
		float predicted_pitch_vel = pitch_d_ + (pitch_torque_ + friction_pitch) / pitch_inertia_ * dt_;
		if (pitch_d_ * predicted_pitch_vel < 0) { // Velocity would change sign
			// Apply exact force needed to bring velocity to zero
			friction_pitch = -pitch_torque_ - pitch_d_ * pitch_inertia_ / dt_;
		}
	} else {
		// Kinetic friction: always opposes motion, magnitude depends on velocity
		float coulomb_friction = pitch_static_friction_ * 0.8; // Kinetic < static typically
		friction_pitch = -sign(pitch_d_) * coulomb_friction - sign(pitch_d_) * std::pow(std::abs(pitch_d_), 2) * pitch_2nd_friction_k_;
	}

	// Compute & Update instant pitch angular acceleration
	pitch_dd_ = (pitch_torque_ + friction_pitch) / pitch_inertia_; // [rad/s^2] - using pitch inertia	

	// [3] Update yaw dynamics (physics-based model)
	
	// Compute current yaw angular velocity by the *last* angular acceleration
	float yaw_vel = yaw_d_ + yaw_dd_ * dt_; // [rad/s]
	// Update yaw angular velocity
	yaw_d_ = yaw_vel;

	// Compute friction forces (corrected physics with numerical stability)
	float f_friction_yaw = 0.0f;
	if (std::abs(yaw_d_) < ROT_VEL_THRESH && std::abs(yaw_torque_) < yaw_static_friction_) {
		// Static friction region: opposes applied torque and damps velocity
		f_friction_yaw = clamp(-yaw_torque_, -yaw_static_friction_, yaw_static_friction_);
		// Add velocity damping to eliminate numerical oscillations
		f_friction_yaw -= yaw_d_ * (yaw_static_friction_ / ROT_VEL_THRESH); // Strong damping near zero
		
		// If the total force would cause velocity to overshoot zero, clamp it
		float predicted_vel = yaw_d_ + (yaw_torque_ + f_friction_yaw) / yaw_inertia_ * dt_;
		if (yaw_d_ * predicted_vel < 0) { // Velocity would change sign
			// Apply exact force needed to bring velocity to zero
			f_friction_yaw = -yaw_torque_ - yaw_d_ * yaw_inertia_ / dt_;
		}
	} else {
		// Kinetic friction: always opposes motion, magnitude depends on velocity
		float coulomb_friction = yaw_static_friction_ * 0.8; // Kinetic < static typically
		f_friction_yaw = -sign(yaw_d_) * coulomb_friction - sign(yaw_d_) * std::pow(std::abs(yaw_d_), 2) * yaw_2nd_friction_k_;
	}

	// Compute & Update instant yaw angular acceleration
	yaw_dd_ = (yaw_torque_ + f_friction_yaw) / yaw_inertia_;

	// [4] Update linear motion dynamics (physics-based model)

	// Compute current velocity by the *last* acceleration
	float x_vel = x_d_ + x_dd_ * dt_; // [m/s]
	// Update velocity
	x_d_ = x_vel;

	// Compute friction forces (same anti-drift strategy as rotational axes)
	float x_friction = 0.0f;
	if (std::abs(x_d_) < LIN_VEL_THRESH && std::abs(x_force_) < x_static_friction_) {
		// Static friction region: opposes applied force and damps velocity
		x_friction = clamp(-x_force_, -x_static_friction_, x_static_friction_);
		// Add velocity damping to eliminate numerical oscillations and drift
		x_friction -= x_d_ * (x_static_friction_ / LIN_VEL_THRESH); // Strong damping near zero
		
		// If the total force would cause velocity to overshoot zero, clamp it
		float predicted_x_vel = x_d_ + (x_force_ + x_friction) / x_mode_mass_ * dt_;
		if (x_d_ * predicted_x_vel < 0) { // Velocity would change sign
			// Apply exact force needed to bring velocity to zero
			x_friction = -x_force_ - x_d_ * x_mode_mass_ / dt_;
		}
	} else {
		// Kinetic friction: always opposes motion, magnitude depends on velocity
		float coulomb_friction = x_static_friction_ * 0.8; // Kinetic < static typically
		x_friction = -sign(x_d_) * coulomb_friction - sign(x_d_) * std::pow(std::abs(x_d_), 2) * x_2nd_friction_k_;
	}

	// Compute & Update instant acceleration
	x_dd_ = (x_force_ + x_friction) / x_mode_mass_; // [m/s^2]

	// [5] Pack the transformation 
	
	// 5a) Apply incremental rotations (using angular velocities, not absolute angles)
	float delta_roll = roll_d_ * dt_;     // Incremental roll rotation (physics-based)
	float delta_pitch = pitch_d_ * dt_;   // Incremental pitch rotation (physics-based)
	float delta_yaw = yaw_d_ * dt_;       // Incremental yaw rotation (physics-based)

	Eigen::Matrix3f R_roll = Eigen::AngleAxisf(delta_roll, Eigen::Vector3f::UnitX()).toRotationMatrix();
	Eigen::Matrix3f R_pitch = Eigen::AngleAxisf(delta_pitch, Eigen::Vector3f::UnitZ()).toRotationMatrix();
	Eigen::Matrix3f R_yaw = Eigen::AngleAxisf(delta_yaw, Eigen::Vector3f::UnitY()).toRotationMatrix();

	step.block<3,3>(0, 0) = R_yaw * R_pitch * R_roll;

	// 5b) Translate along local +X by velocity increment (heading direction)
	float delta_x = x_d_ * dt_;           // Incremental translation

	step(0, 3) = delta_x * 100;           // [m] to [cm] (for QML rendering)

	// [6] Emit the instantaneous transformation to the delegate, will handle pose accumulation
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
