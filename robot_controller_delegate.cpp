/**
 * @file: robot_controller_delegate.cpp
 * 
 * @brief: Implementation of RobotControllerDelegate class
 * 
 * @author: t-tang-rfc
 * 
 * @date: [created: 2025-05-26, updated: 2025-05-28]
 **/

#include "robot_controller_delegate.hpp"

#include <QList>

namespace rf {

const QList<float> DEFAULT_POSE = { -100, 100, -100, 0, 0, 0 };

}

namespace rf {

RobotControllerDelegate::RobotControllerDelegate(QObject *parent)
	: QObject(parent)
	, pose_(DEFAULT_POSE)
{
	// Initialize with the same default values as in QML
}

RobotControllerDelegate::~RobotControllerDelegate() = default;

QList<float> RobotControllerDelegate::getPose() const { return pose_; }

void RobotControllerDelegate::setPose(QList<float> pose)
{
	if (pose_ != pose) {
		pose_ = pose;
		Q_EMIT poseChanged();
	}
}

void RobotControllerDelegate::reset()
{
	setPose(DEFAULT_POSE);
}

} // namespace rf
