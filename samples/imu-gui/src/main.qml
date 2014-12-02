import QtQuick 2.2
import QtQuick.Window 2.1
import Qt3D 2.0
import Qt3D.Shapes 2.0
import IMU 1.0

Window {
    visible: true

    IMU{
        id: imu
    }

    Viewport {
        anchors.fill: parent
        navigation: false //Disable turning the camera by clicking and dragging

        camera: Camera {
            eye:        Qt.vector3d(0, 0, 0) //Camera position
            center:     Qt.vector3d(0, 0,-1) //Camera looks towards this position
            upVector:   Qt.vector3d(0, 1, 0) //Camera's up (+Y axis) is towards this position
            fieldOfView: 38                 //In degrees, this needs to be calibrated/measured
        }

        light: Light{
            position:  Qt.vector3d(0,0,0)
            direction: Qt.vector3d(0,0,1)
        }

        Item3D{
            id: arrow
            mesh: Mesh{ source: "/assets/arrow.dae" }
            transform: [
                Rotation3D{ axis: imu.rotAxis; angle: -imu.rotAngle },
                Translation3D{ translate: Qt.vector3d(0,0,-10) }
            ]
        }

        Sphere{
            radius: 1
            effect: Effect{ color: "#00FFFF" }
            transform: [
                Translation3D{ translate: imu.linearAcceleration.times(0.5) },
                Rotation3D{ axis: imu.rotAxis; angle: -imu.rotAngle },
                Translation3D{ translate: Qt.vector3d(0,0,-10) }
            ]
        }
    }
}
