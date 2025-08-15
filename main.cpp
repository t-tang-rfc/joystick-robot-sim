#include <QGuiApplication>
#include "main_window.hpp"

int main(int argc, char *argv[])
{
	QGuiApplication app(argc, argv);
	rf::MainWindow w;
	w.show();
	return app.exec();
}
