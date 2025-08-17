/**
 * @file: robot_controller_delegate.cpp
 * 
 * @brief: Implementation of RobotControllerDelegate class
 * 
 * @author: t-tang-rfc
 * 
 * @date: [created: 2025-05-26, updated: 2025-08-17]
 **/

#include "robot_controller_delegate.hpp"

namespace rf {

RobotControllerDelegate::RobotControllerDelegate(QObject *parent)
	: QObject(parent)
	, t_(0.0f, 0.0f, 0.0f)       // fill dummy value
	, q_(1.0f, 0.0f, 0.0f, 0.0f) // fill dummy value
{
	// Initialize the transformation matrix with default pose (explicitly)
	pose_ << 1.0f, 0.0f, 0.0f, -100.0f,
			 0.0f, 1.0f, 0.0f, 100.0f,
			 0.0f, 0.0f, 1.0f, -100.0f,
			 0.0f, 0.0f, 0.0f, 1.0f;
	// Trigger initial pose update
	updateDisplayPose();
}

RobotControllerDelegate::~RobotControllerDelegate() = default;

const Eigen::Matrix4f& RobotControllerDelegate::getTransformMatrix() const
{
	return pose_;
}

QVector3D RobotControllerDelegate::getTranslation() const
{
	return t_;
}

QQuaternion RobotControllerDelegate::getRotation() const
{
	return q_;
}

void RobotControllerDelegate::applyTransform(const Eigen::Matrix4f& transform)
{
	// Apply the transform to the current pose (right multiplication for local frame operations)
	pose_ = pose_ * transform;
	updateDisplayPose();
}

void RobotControllerDelegate::updateDisplayPose()
{
	// Extract translation
	Eigen::Vector3f translation = pose_.block<3, 1>(0, 3);
	QVector3D newTranslation(translation.x(), translation.y(), translation.z());
	
	// Extract rotation matrix and convert to quaternion
	Eigen::Matrix3f rotationMatrix = pose_.block<3, 3>(0, 0);
	Eigen::Quaternionf eigenQuat(rotationMatrix);
	QQuaternion newRotation(eigenQuat.w(), eigenQuat.x(), eigenQuat.y(), eigenQuat.z());
	
	// Update Qt properties and emit signals if changed
	bool translationChanged = (t_ != newTranslation);
	bool rotationChanged = (q_ != newRotation);

	t_ = newTranslation;
	q_ = newRotation;

	if (translationChanged || rotationChanged) {
		Q_EMIT poseChanged();
	}
}

} // namespace rf
