import QtQuick 2.2
import QtQuick.Controls 2.0
import QtQuick.Scene3D 2.0
import Qt3D.Core 2.0
import Qt3D.Render 2.0
import Qt3D.Input 2.0
import Qt3D.Extras 2.0
import IMU 1.0

ApplicationWindow{
    visible: true

    IMU{
        id: imu
        //accBias: Qt.vector3d(0.397, -0.008, -0.005)
    }

    Scene3D{
        anchors.fill: parent
        focus: true
        cameraAspectRatioMode: Scene3D.AutomaticAspectRatio

        Entity{
            components: [
                RenderSettings {
                    activeFrameGraph: ForwardRenderer {
                        camera: Camera{
                            position: Qt.vector3d(0, 0, 0)
                            viewCenter: Qt.vector3d(0, 0,-1)
                            upVector: Qt.vector3d(0, 1, 0)
                            nearPlane: 0.1
                            farPlane: 1000.0
                            fieldOfView: 38
                            projectionType: CameraLens.PerspectiveProjection
                        }
                        clearColor: "transparent"
                    }
                }
            ]

            Mesh{ id: arrowZMesh; source: "/assets/arrow_z.obj" }

            //Global frame
            Transform{
                id: globalFrameTransform
                rotation: fromAxisAndAngle(imu.rotAxis, -imu.rotAngle) //Global frame's rotation w.r.t device is the inverted device frame rotation, hence the negative angle
                translation: Qt.vector3d(0,0,-20) //Pull away from camera so that we can see
            }
            Entity{
                Mesh{ id: arrowXMesh; source: "/assets/arrow_x.obj" }
                PhongMaterial{ id: arrowXColor; ambient: "red" }
                components: [ globalFrameTransform, arrowXMesh, arrowXColor ]
            }
            Entity{
                Mesh{ id: arrowYMesh; source: "/assets/arrow_y.obj" }
                PhongMaterial{ id: arrowYColor; ambient: "green" }
                components: [ globalFrameTransform, arrowYMesh, arrowYColor ]
            }
            Entity{
                PhongMaterial{ id: arrowZColor; ambient: "blue" }
                components: [ globalFrameTransform, arrowZMesh, arrowZColor ]
            }

            //Device acceleration in global frame
            Entity{
                Transform{
                    id: accZScaleRot
                    function angleBetween(a,b){ return 180*Math.acos(a.dotProduct(b)/a.length()/b.length())/Math.PI; }
                    rotation: fromAxisAndAngle(imu.linearAcceleration.crossProduct(Qt.vector3d(0,0,1)), -angleBetween(imu.linearAcceleration,Qt.vector3d(0,0,1))) //Turn to direction of acceleration
                    scale3D: Qt.vector3d(1, 1, imu.linearAcceleration.length()/9.81) //Acceleration magnitude
                }
                Transform{ id: accFrameTransform; matrix: globalFrameTransform.matrix.times(accZScaleRot.matrix) }
                PhongMaterial{ id: accZColor; ambient: "cyan" }
                components: [ arrowZMesh, accZColor, accFrameTransform ]
            }
        }
    }
}
