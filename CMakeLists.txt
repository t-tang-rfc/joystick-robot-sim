cmake_minimum_required(VERSION 3.16)
project(joystick-robot-sim LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# @note: adjust the path to your Qt installation, or use symbolic link if necessary
list(APPEND CMAKE_PREFIX_PATH "/opt/qt/qt6/gcc_64/lib/cmake")
find_package(Qt6 COMPONENTS Core Gui Widgets Quick QuickWidgets REQUIRED)

qt_standard_project_setup(REQUIRES 6.8)

set(QT_QML_GENERATE_QMLLS_INI ON)

qt_add_resources(QML_RESOURCES qml/qml.qrc)

add_executable(joystick-robot-sim
	src/main.cpp
	src/main_window.cpp
	src/display_widget.cpp
	# src/robot_stick_controller.cpp
	${QML_RESOURCES}
)

target_link_libraries(joystick-robot-sim PRIVATE Qt6::Core Qt6::Gui Qt6::Widgets Qt6::Quick Qt6::QuickWidgets)
