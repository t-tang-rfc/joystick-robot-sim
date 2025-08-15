#include <QGuiApplication>
#include <QFutureWatcher>
#include <QtConcurrent>

#include "main_window.hpp"
#include "joystick_controller.hpp"

int main(int argc, char *argv[])
{
	// --- [1] Initialize ROS node ---
	ros::init(argc, argv, "simulation_node");

	// --- [2] Instantiate JoystickController (a ROS node handle) ---
	rf::JoystickController joystick_controller;

	// --- [3] Start ROS spinning thread using QtConcurrent (spin won't block Qt loop) ---
	QFutureWatcher<void> rosThread;
	rosThread.setFuture(QtConcurrent::run([](){ ros::spin(); }));

    // --- [4] Setup Qt application ---
	QGuiApplication app(argc, argv);

	rf::MainWindow w;

	// --- [5] Signal-slot connections ---
	QObject::connect(&rosThread, &QFutureWatcher<void>::finished,
		&app, &QCoreApplication::quit);
	QObject::connect(&app, &QCoreApplication::aboutToQuit, []() { ros::shutdown(); });
	QObject::connect(&joystick_controller, &rf::JoystickController::setPose,
		&w, &rf::MainWindow::setPose, Qt::QueuedConnection);

	// --- [6] Show MainWindow scene ---
	w.show();

	// --- [7] Enter main event loop (both ROS and Qt now running) ---
	return app.exec();
}
