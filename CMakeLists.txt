cmake_minimum_required(VERSION 3.16)
project(joystick-robot-sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# @note: adjust the path to your Qt installation, or use symbolic link if necessary
list(APPEND CMAKE_PREFIX_PATH "/opt/qt/Qt-6.8.2")
find_package(Qt6 COMPONENTS Core Gui Quick Qml REQUIRED)
# @note: Qml module provides some type registrations

qt_standard_project_setup(REQUIRES 6.8)

set(QT_QML_GENERATE_QMLLS_INI ON)

add_executable(joystick-robot-sim
	main.cpp
	main_window.cpp
)

target_link_libraries(joystick-robot-sim PRIVATE Qt6::Core Qt6::Gui Qt6::Quick Qt6::Qml)

qt_add_qml_module(joystick-robot-sim
	URI RobotSimulator
	VERSION 1.0
	QML_FILES
		Main.qml
	SOURCES
		robot_controller_delegate.cpp
		robot_controller_delegate.hpp
	DEPENDENCIES QtQuick QtQuick3D QtQuick3D.Helpers
)
