/**
 * @file: display_widget.cpp
 * 
 * @brief: implementation of the DisplayWidget class
 * 
 * @date:
 * - created on 2025-05-22
 * - updated on 2025-05-26
 **/

#include "display_widget.hpp"

namespace rf {

DisplayWidget::DisplayWidget(QWidget* parent)
: QQuickWidget(parent)
{
	setResizeMode(QQuickWidget::SizeRootObjectToView);
	setSource(QUrl("qrc:/qml/Display.qml"));
}

DisplayWidget::~DisplayWidget() = default;

}
