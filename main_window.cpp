/**
 * @file: main_window.cpp
 * 
 * @brief: implementation of the MainWindow class
 * 
 * @author: t-tang-rfc
 * 
 * @date: [created: 2025-05-22, updated: 2025-08-18]
 **/

#include "main_window.hpp"

#include <QQuickItem>
#include <QtLogging>

#include "robot_controller_delegate.hpp"

namespace {

const int WINDOW_WIDTH = 1440; // Default width of the display window
const int WINDOW_HEIGHT = 810; // Default height of the display window

} // namespace

namespace rf {

MainWindow::MainWindow(QWindow* parent) : QQuickView(parent)
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
		// Connect signals to controller slots
		connect(this, &MainWindow::applyTransform, controller_delegate, &RobotControllerDelegate::applyTransform);
	} else {
		qWarning() << "Controller delegate not retrieved from QML. You will NOT be able to control the robot.";
	}
}

MainWindow::~MainWindow() = default;

void MainWindow::keyPressEvent(QKeyEvent* event)
{
	switch (event->key()) {
		// @todo: May add reset function later, for now print a message
		default:
			qDebug() << "Key pressed:" << event->key();
			QQuickView::keyPressEvent(event);
			break;
	}
}

} // namespace rf
