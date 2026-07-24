import QtQuick
import QtQuick.Effects

import "Diagnostics.js" as Logic

Item {

    implicitHeight: 30
    implicitWidth: 30

    Rectangle {
        id: statusCircle
        anchors.centerIn: parent
        width: Math.min(parent.height, parent.width, 10)
        height: width
        radius: height / 2
        color: Logic.levelToColor(model.display)
        visible: false  // rendered via multieffect
    }

    MultiEffect {
        source: statusCircle
        anchors.fill: statusCircle
        blurEnabled: true
        blurMax: 32
        blur: 0.2
        brightness: 0.5
        saturation: -0.5
    }


}
