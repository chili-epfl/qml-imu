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

        //Returns the inverse (complement) of the given quaternion
        function qinv(q){
            return Qt.quaternion(q.scalar, -q.x, -q.y, -q.z);
        }

        //Rotates the vector v by quaternion q
        function rotatedVector(q,v){
            var result = Qt.vector3d(0,0,0);
            result.x = (q.scalar*q.scalar + q.x*q.x - q.y*q.y - q.z*q.z)*v.x + 2*(q.x*q.y - q.scalar*q.z)*v.y + 2*(q.x*q.z + q.scalar*q.y)*v.z;
            result.y = 2*(q.x*q.y + q.scalar*q.z)*v.x + (q.scalar*q.scalar - q.x*q.x + q.y*q.y - q.z*q.z)*v.y + 2*(q.y*q.z - q.scalar*q.x)*v.z;
            result.z = 2*(q.x*q.z - q.scalar*q.y)*v.x + 2*(q.y*q.z + q.scalar*q.x)*v.y + (q.scalar*q.scalar - q.x*q.x - q.y*q.y + q.z*q.z)*v.z;
            return result;
        }

        //Resets the external object pose to origin in camera frame
        function resetPose(){
            resetDisplacement();

            var objectRotC0 = Qt.quaternion(0,1,0,0);
            var objectTransC0 = Qt.vector3d(0,0,0.5);

            objectRot = objectRotC0;
            objectTrans = objectTransC0;
        }

        //Adds the latest displacement to poses and resets the displacement for next interval
        function addDisplacement(){
            var R_CNew_C = qinv(camOmega);
            R_CNew_C = qmul(R_CNew_C, getAngularDisplacement());
            R_CNew_C = qmul(R_CNew_C, camOmega);

            objectRot = qmul(qinv(R_CNew_C), objectRot);

            //var deltaT = getLinearDisplacement(Qt.vector3d(0,0,0));
            //console.log(deltaT);
            //objectTrans = objectTrans.minus(deltaT);
            objectTrans = rotatedVector(qinv(R_CNew_C), objectTrans);

            resetDisplacement();
        }

        //Describes the static rotation of the device camera in device frame
        property quaternion camOmega: Qt.quaternion(0,0,1,0) //??????????????????? should be (0,1,0,0)

        //Describes the translation of the external object in device frame
        property vector3d objectTrans

        //Describes the rotation of the external object in device frame
        property quaternion objectRot
    }

    //In 30ms intervals, add the linear and angular displacement to the external object
    Timer {
        interval: 30; running: true; repeat: true
        property bool startupEnded: false
        onTriggered:{
            if(imu.startupComplete){

                //In the beginning, reset external object pose before doing anything
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

        camera: Camera {
            eye:        Qt.vector3d(0,0,0)
            center:     Qt.vector3d(0,0,1)
            upVector:   Qt.vector3d(0,1,0)
            fieldOfView: 38
            nearPlane:  0.001
        }

        light: Light{
            position:  Qt.vector3d(0,0,0)
            direction: Qt.vector3d(0,0,-1)
        }

        Item3D{
            id: object
            mesh: Mesh{ source: "/assets/arrow_x_y_z.dae" }
            transform: [
                Rotation3D{
                    //Have to convert quaternion to angle-axis...
                    property real vnorm: Math.sqrt(imu.objectRot.x*imu.objectRot.x + imu.objectRot.y*imu.objectRot.y + imu.objectRot.z*imu.objectRot.z)
                    property real theta: 2*Math.atan2(vnorm, imu.objectRot.scalar)
                    property real sTheta2: Math.sin(theta/2)
                    angle: theta/Math.PI*180
                    axis: Qt.vector3d(imu.objectRot.x*sTheta2, imu.objectRot.y*sTheta2, imu.objectRot.z*sTheta2)
                },
                Translation3D{ translate:imu.objectTrans }
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
