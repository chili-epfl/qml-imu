import QtQuick 2.2
import QtQuick.Window 2.1
import QtQuick.Controls 1.2
import QtQuick.Controls.Styles 1.2
import Qt3D 2.0
import Qt3D.Shapes 2.0
import IMU 1.0

Window {
    visible: true

    IMU{
        id: imu

        //Adds the rotation described by q1 to the one described by q2
        function qmul(q1,q2){
            var result = Qt.quaternion(1,0,0,0);
            result.scalar = q1.scalar*q2.scalar - q1.x*q2.x         - q1.y*q2.y         - q1.z*q2.z;
            result.x =      q1.scalar*q2.x      + q1.x*q2.scalar    + q1.y*q2.z         - q1.z*q2.y;
            result.y =      q1.scalar*q2.y      - q1.x*q2.z         + q1.y*q2.scalar    + q1.z*q2.x;
            result.z =      q1.scalar*q2.z      + q1.x*q2.y         - q1.y*q2.x         + q1.z*q2.scalar;
            return result;
        }

        //Resets the position to zero and rotation to current rotation in global frame
        function resetPose(){
            resetDisplacement();
            deviceTrans = Qt.vector3d(0,0,0);
            deviceRot = Qt.quaternion(Math.cos(rotAngle/360*Math.PI),
                                rotAxis.x*Math.sin(rotAngle/360*Math.PI),
                                rotAxis.y*Math.sin(rotAngle/360*Math.PI),
                                rotAxis.z*Math.sin(rotAngle/360*Math.PI));
        }

        //Adds latest displacement to device pose and resets the displacement for next interval
        function addDisplacement(){
            var deltaT = imu.getLinearDisplacement(Qt.vector3d(0,0,0));
            imu.deviceTrans.x += deltaT.x;
            imu.deviceTrans.y += deltaT.y;
            imu.deviceTrans.z += deltaT.z;
            imu.deviceRot = imu.qmul(imu.getAngularDisplacement(), imu.deviceRot);
            var norm = imu.deviceRot.scalar*imu.deviceRot.scalar + imu.deviceRot.x*imu.deviceRot.x + imu.deviceRot.y*imu.deviceRot.y + imu.deviceRot.z*imu.deviceRot.z;
            imu.deviceRot.scalar /= norm;
            imu.deviceRot.x /= norm;
            imu.deviceRot.y /= norm;
            imu.deviceRot.z /= norm;
            imu.resetDisplacement();
        }

        //Describes the translation of the device in global frame
        property vector3d deviceTrans

        //Describes the rotation of the device in global frame
        property quaternion deviceRot
    }

    //In 30ms intervals, add the linear and angular displacement to the device's 3D model
    Timer {
        interval: 30; running: true; repeat: true
        property bool startupEnded: false
        onTriggered:{
            if(imu.startupComplete){

                //In the beginning, reset the device pose before doing anything
                if(!startupEnded){
                    startupEnded = true;
                    imu.resetPose();
                }
                else
                    imu.addDisplacement();
            }
        }
    }

    Viewport {
        anchors.fill: parent
        navigation: false
        fillColor: "#999999"

        //Camera looks top down from 1 meter high
        camera: Camera {
            eye:        Qt.vector3d(0, 0, 1)
            center:     Qt.vector3d(0, 0, 0)
            upVector:   Qt.vector3d(0, 1, 0)
            fieldOfView: 38
            nearPlane:  0.001
        }

        light: Light{
            position:  Qt.vector3d(0,0,0)
            direction: Qt.vector3d(0,0,1)
        }

        Item3D{
            id: device
            mesh: Mesh{ source: "/assets/tablet.dae" }
            transform: [
                Rotation3D{
                    //Have to convert quaternion to angle-axis...
                    property real vnorm: Math.sqrt(imu.deviceRot.x*imu.deviceRot.x + imu.deviceRot.y*imu.deviceRot.y + imu.deviceRot.z*imu.deviceRot.z)
                    property real theta: 2*Math.atan2(vnorm, imu.deviceRot.scalar)
                    property real sTheta2: Math.sin(theta/2)
                    angle: theta/Math.PI*180
                    axis: Qt.vector3d(imu.deviceRot.x*sTheta2, imu.deviceRot.y*sTheta2, imu.deviceRot.z*sTheta2)
                },
                Translation3D{ translate:imu.deviceTrans }
            ]
        }
    }

    Button {
        text: "Reset pose"
        onClicked: imu.resetPose();
        style: ButtonStyle {
            label: Text {
                renderType: Text.NativeRendering
                verticalAlignment: Text.AlignVCenter
                horizontalAlignment: Text.AlignHCenter
                font.pointSize: 50
                text: control.text
            }
        }
    }
}
