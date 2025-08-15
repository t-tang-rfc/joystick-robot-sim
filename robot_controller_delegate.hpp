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
 * @date:
 * - created on 2025-05-26
 * - updated on 2025-05-31
 **/

#ifndef ROBOT_CONTROLLER_DELEGATE_HPP
#define ROBOT_CONTROLLER_DELEGATE_HPP

#include <QObject>
#include <QtGlobal>
#include <QList>
#include <QtQml/qqmlregistration.h>

namespace rf {

class RobotControllerDelegate : public QObject
{
	Q_OBJECT
	QML_ELEMENT
	Q_PROPERTY(QList<qreal> pose READ getPose WRITE setPose NOTIFY poseChanged)

	public:
		explicit RobotControllerDelegate(QObject *parent = nullptr);
		~RobotControllerDelegate();

		QList<qreal> getPose() const;

		Q_SLOT void setPose(const QList<qreal>& pose);
		Q_SLOT void reset();

		Q_SIGNAL void poseChanged();

	private:
		QList<qreal> pose_;
};

} // namespace rf

#endif // ROBOT_CONTROLLER_DELEGATE_HPP
