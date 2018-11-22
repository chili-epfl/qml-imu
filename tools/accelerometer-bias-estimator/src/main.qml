import QtQuick 2.2
import QtQuick.Window 2.2
import IMU 1.0

Window {
    visible: true

    AccelerometerBiasEstimator{
        id: abe
    }

    Text {
        anchors.verticalCenter: parent.verticalCenter
        anchors.horizontalCenter: parent.horizontalCenter
        text: "tr(cov): " + abe.covTrace.toPrecision(6) + "\nx: " + abe.bias.x.toPrecision(6) + "\ny: " + abe.bias.y.toPrecision(6) + "\nz: " + abe.bias.z.toPrecision(6)
        font.pixelSize: 100
    }
}

