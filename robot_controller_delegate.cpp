/**
 * @file: robot_controller_delegate.cpp
 * 
 * @brief: Implementation of RobotControllerDelegate class
 * 
 * @author: t-tang-rfc
 * 
 * @date:
 * - created on 2025-05-26
 * - updated on 2025-05-28
 **/

#include "robot_controller_delegate.hpp"

#include <QList>

namespace rf {

const QList<qreal> DEFAULT_POSE = { -100, 100, -100, 0, 0, 0 };

}

namespace rf {

RobotControllerDelegate::RobotControllerDelegate(QObject *parent)
	: QObject(parent)
	, pose_(DEFAULT_POSE)
{
	// Initialize with the same default values as in QML
}

RobotControllerDelegate::~RobotControllerDelegate() = default;

QList<qreal> RobotControllerDelegate::getPose() const { return pose_; }

void RobotControllerDelegate::setPose(const QList<qreal>& pose)
{
	if (pose_ != pose) {
		pose_ = pose;
		emit poseChanged();
	}
}

void RobotControllerDelegate::reset()
{
	setPose(DEFAULT_POSE);
}

} // namespace rf
