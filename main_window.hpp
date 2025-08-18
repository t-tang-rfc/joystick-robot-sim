/**
 * @file: main_window.hpp
 * 
 * @brief: GUI main window
 * 
 * @author: t-tang-rfc
 *
 * @date: [created: 2025-05-22, updated: 2025-08-18]
 **/

#ifndef MAIN_WINDOW_HPP
#define MAIN_WINDOW_HPP

#include <QQuickView>
#include <QKeyEvent>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

namespace rf {

class MainWindow : public QQuickView
{
	Q_OBJECT

	public:
		explicit MainWindow(QWindow* parent = nullptr);
		~MainWindow();

		Q_SIGNAL void applyTransform(Eigen::Matrix4f transform); // Signal-relay, to RobotControllerDelegate
		Q_SIGNAL void resetPose(); // Signal to reset robot pose

	protected:
		void keyPressEvent(QKeyEvent* event) override;

};

} // namespace rf

#endif // MAIN_WINDOW_HPP
