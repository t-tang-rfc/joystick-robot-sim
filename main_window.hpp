/**
 * @file: main_window.hpp
 * 
 * @brief: GUI main window
 * 
 * @author: t-tang-rfc
 *
 * @date: [created: 2025-05-22, updated: 2025-06-06]
 **/

#ifndef MAIN_WINDOW_HPP
#define MAIN_WINDOW_HPP

#include <QQuickView>
#include <QtGlobal>
#include <QList>

namespace rf {

class MainWindow : public QQuickView
{
	Q_OBJECT

	public:
		explicit MainWindow(QWindow* parent = nullptr);
		~MainWindow();

		Q_SIGNAL void setPose(QList<float> pose); // Signal-relay, to RobotControllerDelegate

	protected:
		void keyPressEvent(QKeyEvent* event) override;

	private:
		QList<float> robot_pose_; // a 6-element list for robot pose tracking
};

} // namespace rf

#endif // MAIN_WINDOW_HPP
