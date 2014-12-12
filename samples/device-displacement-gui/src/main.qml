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

        //Rotates the vector v by quaternion q
        function rotatedVector(q,v){
            var result = Qt.vector3d(0,0,0);
            result.x = (q.scalar*q.scalar + q.x*q.x - q.y*q.y - q.z*q.z)*v.x + 2*(q.x*q.y - q.scalar*q.z)*v.y + 2*(q.x*q.z + q.scalar*q.y)*v.z;
            result.y = 2*(q.x*q.y + q.scalar*q.z)*v.x + (q.scalar*q.scalar - q.x*q.x + q.y*q.y - q.z*q.z)*v.y + 2*(q.y*q.z - q.scalar*q.x)*v.z;
            result.z = 2*(q.x*q.z - q.scalar*q.y)*v.x + 2*(q.y*q.z + q.scalar*q.x)*v.y + (q.scalar*q.scalar - q.x*q.x - q.y*q.y + q.z*q.z)*v.z;
            return result;
        }

        //Resets the device/point positions to origin and rotation to current rotation in global frame
        function resetPose(){
            resetDisplacement();

            deviceTrans = Qt.vector3d(0,0,0);
            deviceRot = Qt.quaternion(Math.cos(rotAngle/360*Math.PI),
                                      rotAxis.x*Math.sin(rotAngle/360*Math.PI),
                                      rotAxis.y*Math.sin(rotAngle/360*Math.PI),
                                      rotAxis.z*Math.sin(rotAngle/360*Math.PI));
            r_C_G = deviceRot;

            pointTrans = rotatedVector(deviceRot, pointR);
            pointRot = qmul(deviceRot, pointOmega);
        }

        //Adds the latest displacement to poses and resets the displacement for next interval
        function addDisplacement(){

            //Device
            targetTranslation = Qt.vector3d(0,0,0);
            targetRotation = Qt.quaternion(1,0,0,0);

            var deltaTDevice = rotatedVector(r_C_G, getLinearDisplacement());
            deviceTrans.x += deltaTDevice.x;
            deviceTrans.y += deltaTDevice.y;
            deviceTrans.z += deltaTDevice.z;

            deviceRot = qmul(deviceRot, getAngularDisplacement());
            var norm = deviceRot.scalar*deviceRot.scalar + deviceRot.x*deviceRot.x + deviceRot.y*deviceRot.y + deviceRot.z*deviceRot.z;
            deviceRot.scalar /= norm;
            deviceRot.x /= norm;
            deviceRot.y /= norm;
            deviceRot.z /= norm;

            //Point on device
            targetTranslation = pointR;
            targetRotation = pointOmega;

            var deltaTPoint = rotatedVector(r_C_G, rotatedVector(pointOmega, getLinearDisplacement()));
            pointTrans.x += deltaTPoint.x;
            pointTrans.y += deltaTPoint.y;
            pointTrans.z += deltaTPoint.z;

            pointRot = qmul(pointRot, getAngularDisplacement());
            norm = pointRot.scalar*pointRot.scalar + pointRot.x*pointRot.x + pointRot.y*pointRot.y + pointRot.z*pointRot.z;
            pointRot.scalar /= norm;
            pointRot.x /= norm;
            pointRot.y /= norm;
            pointRot.z /= norm;

            resetDisplacement();

            r_C_G = Qt.quaternion(Math.cos(rotAngle/360*Math.PI),
                                  rotAxis.x*Math.sin(rotAngle/360*Math.PI),
                                  rotAxis.y*Math.sin(rotAngle/360*Math.PI),
                                  rotAxis.z*Math.sin(rotAngle/360*Math.PI));
        }

        //Describes the static translation of the local point in device frame
        property vector3d pointR: Qt.vector3d(0,0.076,0)

        //Describes the static rotation of the local point in device frame
        property quaternion pointOmega: Qt.quaternion(0,1,0,0)

        //Describes the translation of the device in global frame
        property vector3d deviceTrans

        //Describes the rotation of the device in global frame
        property quaternion deviceRot

        //Describes the translation of the point on the device
        property vector3d pointTrans

        //Describes the rotation of the point on the device
        property quaternion pointRot

        //Describes the previous rotation of the device frame in ground frame
        property quaternion r_C_G
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

        Item3D{
            id: point
            mesh: Mesh{ source: "/assets/arrow_x_y_z.dae" }
            transform: [
                Rotation3D{
                    //Have to convert quaternion to angle-axis...
                    property real vnorm: Math.sqrt(imu.pointRot.x*imu.pointRot.x + imu.pointRot.y*imu.pointRot.y + imu.pointRot.z*imu.pointRot.z)
                    property real theta: 2*Math.atan2(vnorm, imu.pointRot.scalar)
                    property real sTheta2: Math.sin(theta/2)
                    angle: theta/Math.PI*180
                    axis: Qt.vector3d(imu.pointRot.x*sTheta2, imu.pointRot.y*sTheta2, imu.pointRot.z*sTheta2)
                },
                Translation3D{ translate:imu.pointTrans }
            ]
        }
    }

    Button {
        text: "Reset translation"
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
