/**
 * @file: robot_controller_delegate.hpp
 * 
 * @brief: A controller delegate class for the robot in the 3D scene
 * 
 * @author: t-tang-rfc
 * 
 * @note:
 * - Qt6 is changing drastically, and documentation is far from complete.
 * - see https://doc.qt.io/qt-6/qqmlintegration-h-qtqml-proxy.html for the macro definitions
 * - see https://doc.qt.io/qt-6/qtqml-cppintegration-definetypes.html which says if you are already linking to QtQml, you can use `qqmlregistration.h` which includes the necessary headers
 *
 * @date: [created: 2025-05-26, updated: 2025-08-17]
 **/

#ifndef ROBOT_CONTROLLER_DELEGATE_HPP
#define ROBOT_CONTROLLER_DELEGATE_HPP

#include <QObject>
#include <QVector3D>
#include <QQuaternion>
#include <QtQml/qqmlregistration.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

namespace rf {

class RobotControllerDelegate : public QObject
{
	Q_OBJECT
	QML_ELEMENT
	Q_PROPERTY(QVector3D translation MEMBER t_ READ getTranslation NOTIFY poseChanged)
	Q_PROPERTY(QQuaternion rotation MEMBER q_ READ getRotation NOTIFY poseChanged)

	public:
		explicit RobotControllerDelegate(QObject *parent = nullptr);
		~RobotControllerDelegate();

		const Eigen::Matrix4f& getTransformMatrix() const;

		QVector3D getTranslation() const;
		QQuaternion getRotation() const;

		void applyTransform(const Eigen::Matrix4f& transform);

		Q_SIGNAL void poseChanged();

	private:
		void updateDisplayPose();

		Eigen::Matrix4f pose_;       // 4x4 homogeneous transformation matrix
		QVector3D t_;                // translation
		QQuaternion q_;              // rotation
};

} // namespace rf

#endif // ROBOT_CONTROLLER_DELEGATE_HPP
