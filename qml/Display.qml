import QtQuick
import QtQuick3D
import QtQuick3D.Helpers
// @note: Coordinates frame of the 3D world is:
// - the positive direction of the x axis is to the right
// - positive y points upwards
// - and positive z out of the screen.
// @see: https://www.qt.io/product/qt6/qml-book/ch12-qtquick3d-basics

View3D {
	id: viewport
	anchors.fill: parent
	camera: camera_node

	environment: SceneEnvironment {
		clearColor: "skyblue"
		backgroundMode: SceneEnvironment.Color
	}

	DirectionalLight {
		eulerRotation.x: -20
		eulerRotation.y: 110
		castsShadow: true
	}

	// @see: https://doc.qt.io/qt-6/qml-qtquick3d-helpers-orbitcameracontroller.html
	Node {
		id: origin_node
		PerspectiveCamera {
			id: camera_node
			position: Qt.vector3d(0, 100, 400)
			clipFar: 1000
			clipNear: 1
		}
	}

	OrbitCameraController {
		anchors.fill: parent
		origin: origin_node
		camera: camera_node
	}

	// [0] The world frame
	Node {
		id: world_frame
		position: Qt.vector3d(0, 0, 0)
		// @note: The default size of built-in geometries is 100 cm.
		// @see: https://doc.qt.io/qt-6/qtquick3dphysics-units.html
		property real mesh_sz_: 100
		property real axis_len_: 200

		Node {
			id: world_x_axis
			property string color_: "Red"

			Model {
				property real s_: 0.2
				source: "#Cone"
				position: Qt.vector3d(world_frame.axis_len_, 0, 0)
				scale: Qt.vector3d(s_, s_, s_)					
				eulerRotation: Qt.vector3d(0, 0, -90)
				materials: [ PrincipledMaterial { baseColor: world_x_axis.color_; } ]
				castsShadows: false
			}
			Model {
				property real sx_: 0.1
				property real sy_: world_frame.axis_len_/world_frame.mesh_sz_
				property real sz_: sx_
				source: "#Cylinder"
				position: Qt.vector3d(world_frame.axis_len_/2, 0, 0)
				scale: Qt.vector3d(sx_, sy_, sz_)					
				eulerRotation: Qt.vector3d(0, 0, -90)
				materials: [ PrincipledMaterial { baseColor: world_x_axis.color_; } ]
				castsShadows: false
			}
		}
		Node {
			id: world_y_axis
			property string color_: "Blue"

			Model {
				property real s_: 0.2
				source: "#Cone"
				position: Qt.vector3d(0, world_frame.axis_len_, 0)
				scale: Qt.vector3d(s_, s_, s_)
				eulerRotation: Qt.vector3d(0, 0, 0)
				materials: [ PrincipledMaterial { baseColor: world_y_axis.color_; } ]
				castsShadows: false
			}
			Model {
				property real sx_: 0.1
				property real sy_: world_frame.axis_len_/world_frame.mesh_sz_
				property real sz_: sx_
				source: "#Cylinder"
				position: Qt.vector3d(0, world_frame.axis_len_/2, 0)
				scale: Qt.vector3d(sx_, sy_, sz_)
				eulerRotation: Qt.vector3d(0, 0, 0)
				materials: [ PrincipledMaterial { baseColor: world_y_axis.color_; } ]
				castsShadows: false
			}
		}
		Node {
			id: world_z_axis
			property string color_: "Green"

			Model {
				property real s_: 0.2
				source: "#Cone"
				position: Qt.vector3d(0, 0, world_frame.axis_len_)
				scale: Qt.vector3d(s_, s_, s_)
				eulerRotation: Qt.vector3d(90, 0, 0)
				materials: [ PrincipledMaterial { baseColor: world_z_axis.color_; } ]
				castsShadows: false
			}
			Model {
				property real sx_: 0.1
				property real sy_: world_frame.axis_len_/world_frame.mesh_sz_
				property real sz_: sx_
				source: "#Cylinder"
				position: Qt.vector3d(0, 0, world_frame.axis_len_/2)
				scale: Qt.vector3d(sx_, sy_, sz_)
				eulerRotation: Qt.vector3d(90, 0, 0)
				materials: [ PrincipledMaterial { baseColor: world_z_axis.color_; } ]
				castsShadows: false
			}
		}			
	}

	/// [1] The stick
	Node {
		id: robot_stick

		readonly property real mesh_sz_: 100
		// These properties will be controlled by the C++ controller
		property real pos_x_: -100
		property real pos_y_: 100
		property real pos_z_: -100
		property real rot_x_: 0
		property real rot_y_: 0
		property real rot_z_: 0
		
		position: Qt.vector3d(robot_stick.pos_x_, robot_stick.pos_y_, robot_stick.pos_z_)
		eulerRotation: Qt.vector3d(robot_stick.rot_x_, robot_stick.rot_y_, robot_stick.rot_z_)

		Node {
			id: stick
			property real stick_len_: 160
			property real stick_phi_: 40
			property string color_: "Gray"

			Model {
				source: "#Cylinder"
				position: Qt.vector3d(-stick.stick_len_/2, 0, 0)
				scale: Qt.vector3d(stick.stick_phi_/robot_stick.mesh_sz_, stick.stick_len_/robot_stick.mesh_sz_, stick.stick_phi_/robot_stick.mesh_sz_)					
				eulerRotation: Qt.vector3d(0, 0, -90)
				materials: [ SpecularGlossyMaterial { specularColor: stick.color_; } ]
			}
		}

		Node {
			id: stick_frame
			position: Qt.vector3d(0, 0, 0)				
			// @note: The default size of built-in geometries is 100 cm.
			// @see: https://doc.qt.io/qt-6/qtquick3dphysics-units.html
			property real mesh_sz_: 100
			property real axis_len_: 50

			Node {
				id: stick_x_axis
				property string color_: "Red"

				Model {
					property real s_: 0.04
					source: "#Cone"
					position: Qt.vector3d(stick_frame.axis_len_, 0, 0)
					scale: Qt.vector3d(s_, s_, s_)						
					eulerRotation: Qt.vector3d(0, 0, -90)
					materials: [ PrincipledMaterial { baseColor: stick_x_axis.color_; } ]
					castsShadows: false
				}
				Model {
					property real sx_: 0.02
					property real sy_: stick_frame.axis_len_/stick_frame.mesh_sz_
					property real sz_: sx_
					source: "#Cylinder"
					position: Qt.vector3d(stick_frame.axis_len_/2, 0, 0)
					scale: Qt.vector3d(sx_, sy_, sz_)						
					eulerRotation: Qt.vector3d(0, 0, -90)
					materials: [ PrincipledMaterial { baseColor: stick_x_axis.color_; } ]
					castsShadows: false
				}
			}
			Node {
				id: stick_y_axis
				property string color_: "Blue"

				Model {
					property real s_: 0.04
					source: "#Cone"
					position: Qt.vector3d(0, stick_frame.axis_len_, 0)
					scale: Qt.vector3d(s_, s_, s_)						
					eulerRotation: Qt.vector3d(0, 0, 0)
					materials: [ PrincipledMaterial { baseColor: stick_y_axis.color_; } ]
					castsShadows: false
				}
				Model {
					property real sx_: 0.02
					property real sy_: stick_frame.axis_len_/stick_frame.mesh_sz_
					property real sz_: sx_
					source: "#Cylinder"
					position: Qt.vector3d(0, stick_frame.axis_len_/2, 0)
					scale: Qt.vector3d(sx_, sy_, sz_)						
					eulerRotation: Qt.vector3d(0, 0, 0)
					materials: [ PrincipledMaterial { baseColor: stick_y_axis.color_; } ]
					castsShadows: false
				}
			}
			Node {
				id: stick_z_axis
				property string color_: "Green"

				Model {
					property real s_: 0.04
					source: "#Cone"
					position: Qt.vector3d(0, 0, stick_frame.axis_len_)
					scale: Qt.vector3d(s_, s_, s_)						
					eulerRotation: Qt.vector3d(90, 0, 0)
					materials: [ PrincipledMaterial { baseColor: stick_z_axis.color_; } ]
					castsShadows: false
				}
				Model {
					property real sx_: 0.02
					property real sy_: stick_frame.axis_len_/stick_frame.mesh_sz_
					property real sz_: sx_
					source: "#Cylinder"
					position: Qt.vector3d(0, 0, stick_frame.axis_len_/2)
					scale: Qt.vector3d(sx_, sy_, sz_)						
					eulerRotation: Qt.vector3d(90, 0, 0)
					materials: [ PrincipledMaterial { baseColor: stick_z_axis.color_; } ]
					castsShadows: false
				}
			}
		}
	}

	/// [2] The ground
	Node {
		id: ground

		Model {
			source: "#Rectangle"
			position: Qt.vector3d(0, 0, 0)
			scale: Qt.vector3d(20, 20, 1)
			eulerRotation: Qt.vector3d(-90, 0, 0)
			materials: [ PrincipledMaterial {
				baseColor: "#dedede" // a light gray, or whatever you like
				roughness: 0.7
			}]
			// Optionally, make it not cast/receive shadows for performance:
			receivesShadows: true
			castsShadows: false
			opacity: 0.5
		}
	}
}
