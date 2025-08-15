/**
 * @file: main_window.hpp
 * 
 * @author: t-tang-rfc
 * 
 * @brief: GUI main window
 *
 * @date:
 * - created on 2025-05-22
 * - updated on 2025-06-06
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

	/// @note: the following parts implements a simple robot controller using keyboard input

		Q_SLOT void moveX(qreal delta_x);
		Q_SLOT void moveY(qreal delta_y);
		Q_SLOT void moveZ(qreal delta_z);

		Q_SIGNAL void setPose(const QList<qreal>& pose);

	protected:
		void keyPressEvent(QKeyEvent* event) override;

	private:
		QList<qreal> robot_pose_; // a 6-element list for robot pose tracking

	/// <<<
};

} // namespace rf

#endif // MAIN_WINDOW_HPP
