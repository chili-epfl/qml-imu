import QtQuick 2.2
import QtQuick.Window 2.1
import QtQuick.Controls 1.2
import IMU 1.0

Window {
    visible: true

    AccelerometerBiasEstimator{
        id: abe
    }

    Label {
        anchors.verticalCenter: parent.verticalCenter
        anchors.horizontalCenter: parent.horizontalCenter
        text: "tr(cov): " + abe.covTrace + "\nx: " + abe.bias.x + "\ny: " + abe.bias.y + "\nz: " + abe.bias.z
        font.pixelSize: 100
    }
}

