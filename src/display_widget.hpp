/**
 * @file: display_widget.hpp
 * 
 * @author: t-tang-rfc
 * 
 * @brief: a widget for displaying a QML scene and allow user interaction
 * 
 * @date:
 * - created on 2025-05-22
 * - updated on 2025-05-22
 **/

#ifndef DISPLAY_WIDGET_HPP
#define DISPLAY_WIDGET_HPP

#include <QQuickWidget>
#include <QWidget>

namespace rf {

class DisplayWidget : public QQuickWidget
{
	Q_OBJECT
	
	public:
		explicit DisplayWidget(QWidget* parent = nullptr);
		~DisplayWidget();
};

} // namespace rf

#endif // DISPLAY_WIDGET_HPP
