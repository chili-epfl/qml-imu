import QtQuick 2.2
import QtQuick.Window 2.1
import Qt3D 2.0
import Qt3D.Shapes 2.0
import IMU 1.0

Window {
    visible: true

    IMU{
        id: imu
        accBias: Qt.vector3d(0.397, -0.008, -0.005)
    }

    Viewport {
        anchors.fill: parent
        navigation: false

        camera: Camera {
            eye:        Qt.vector3d(0, 0, 0)
            center:     Qt.vector3d(0, 0,-1)
            upVector:   Qt.vector3d(0, 1, 0)
            fieldOfView: 38
        }

        light: Light{
            position:  Qt.vector3d(0,0,0)
            direction: Qt.vector3d(0,0,1)
        }

        //This is the global frame
        Item3D{
            id: globalFrame
            mesh: Mesh{ source: "/assets/arrow_x_y_z.dae" }
            transform: [

                //Global frame's rotation w.r.t device is the inverted device frame rotation, hence the negative angle
                Rotation3D{ axis: imu.rotAxis; angle: -imu.rotAngle },

                //Pull away from camera so that we can see
                Translation3D{ translate: Qt.vector3d(0,0,-10) }
            ]
        }

        //This is the device acceleration in global frame
        Item3D{
            id: accArrow
            mesh: Mesh{ source: "/assets/arrow_z_cyan.dae" }
            transform: [

                //Length of arrow describes acceleration amount, assume 1g is unit scale
                Scale3D { scale: Qt.vector3d(1, 1, imu.linearAcceleration.length()/9.81) },

                //Turn to direction of acceleration
                Rotation3D{
                    function angleBetween(a,b){ return 180*Math.acos(a.dotProduct(b)/a.length()/b.length())/Math.PI; }
                    axis: imu.linearAcceleration.crossProduct(Qt.vector3d(0,0,1));
                    angle: -angleBetween(imu.linearAcceleration,Qt.vector3d(0,0,1))
                },

                //Align to global frame
                Rotation3D{ axis: imu.rotAxis; angle: -imu.rotAngle },

                //Pull away from camera so that we can see
                Translation3D{ translate: Qt.vector3d(0,0,-10) }
            ]
        }
    }
}
