/**
 * @file: main_window.cpp
 * 
 * @brief: implementation of the MainWindow class
 * 
 * @author: t-tang-rfc
 * 
 * @date:
 * - created on 2025-05-22
 * - updated on 2025-05-31
 **/

#include "main_window.hpp"

#include <QQuickItem>
#include <QtLogging>

#include "robot_controller_delegate.hpp"

namespace {

const qreal KBD_MOVE_STEP = 5.0; // Default movement step size via keyboard control
const int WINDOW_WIDTH = 1440; // Default width of the display window
const int WINDOW_HEIGHT = 810; // Default height of the display window

} // namespace

namespace rf {

MainWindow::MainWindow(QWindow* parent)
	: QQuickView(parent), robot_pose_({0, 0, 0, 0, 0, 0})
{
	setTitle("Display Main Window");
	setWidth(WINDOW_WIDTH);
	setHeight(WINDOW_HEIGHT);
	setResizeMode(QQuickView::SizeRootObjectToView);
	
	// Load the module defined by `qt_add_qml_module` in CMakeLists.txt
	loadFromModule("RobotSimulator", "Main");

	// Retrieve the controller delegate from the QML
	auto controller_delegate = rootObject()->findChild<RobotControllerDelegate*>("controller");
	if (controller_delegate) {
		// Set initial pose values
		robot_pose_ = controller_delegate->getPose();

		// Connect signals to controller slots
		connect(this, &MainWindow::setPose, controller_delegate, &RobotControllerDelegate::setPose);
	} else {
		qWarning() << "Controller delegate not retrieved from QML. You will NOT be able to control the robot.";
	}
}

MainWindow::~MainWindow() = default;

void MainWindow::keyPressEvent(QKeyEvent* event)
{
	switch (event->key()) {
		case Qt::Key_W:
			moveX(KBD_MOVE_STEP);
			break;
		case Qt::Key_S:
			moveX(-KBD_MOVE_STEP);
			break;
		case Qt::Key_A:
			moveZ(-KBD_MOVE_STEP);
			break;
		case Qt::Key_D:
			moveZ(KBD_MOVE_STEP);
			break;
		case Qt::Key_Up:
			moveY(KBD_MOVE_STEP);
			break;
		case Qt::Key_Down:
			moveY(-KBD_MOVE_STEP);
			break;
		default:
			QQuickView::keyPressEvent(event);
			break;
	}
}

void MainWindow::moveX(qreal delta_x) {
	robot_pose_[0] += delta_x;
	emit setPose(robot_pose_);
}

void MainWindow::moveY(qreal delta_y) {
	robot_pose_[1] += delta_y;
	emit setPose(robot_pose_);
}

void MainWindow::moveZ(qreal delta_z) {
	robot_pose_[2] += delta_z;
	emit setPose(robot_pose_);
}

} // namespace rf
