/**
 * @file: main_window.cpp
 * 
 * @author: t-tang-rfc
 * 
 * @brief: implementation of the GUI main window
 * 
 * @date:
 * - created on 2025-05-22
 * - updated on 2025-05-26
 **/

#include "main_window.hpp"

#include <QSize>

#include "display_widget.hpp"

namespace {
	const QSize MAIN_WINDOW_SIZE(1440, 810); // 16:9 aspect ratio
}

namespace rf {

MainWindow::MainWindow(QWidget* parent)
: QMainWindow(parent)
{
	// Set the main window size and title
	setFixedSize(MAIN_WINDOW_SIZE);
	setWindowTitle("Display Main Window");

	// Create and set the display widget as the central widget
	auto* display = new DisplayWidget(this);
	setCentralWidget(display);
}

MainWindow::~MainWindow() = default;

} // namespace rf
