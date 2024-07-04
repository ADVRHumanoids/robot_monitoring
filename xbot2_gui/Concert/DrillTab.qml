import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import Common

import "Drilling.js" as Logic

Control {

    property list<string> blobIds

    //
    id: root

    padding: 4
    topPadding: 8
    bottomPadding: 8

    contentItem: ColumnLayout {

        id: col

        spacing: 3

        Label {
            text: ' select blob'
            font.pointSize: 10
        }

        ComboBox {
            Layout.fillWidth: true
            id: blobIdCombo
            model: root.blobIds
            width: parent.width
        }

        Label {
            text: ' select depth (cm)'
            font.pointSize: 10
        }

        SpinBox {
            Layout.fillWidth: true
            id: depthSpin
            value: 5
            from: 0
            to: 10
        }

        Item {
            Layout.preferredHeight: 2
        }

        Label {
            text: ' select drill speed (mm/s)'
            font.pointSize: 10
        }

        SpinBox {
            Layout.fillWidth: true
            id: drillSpeedSpin
            value: 5
            from: 0
            to: 10
        }

        Item {
            Layout.preferredHeight: 2
        }

        DelayButton {
            Layout.fillWidth: true
            text: 'Execute drill'
            width: parent.width

            onActivated: {
                Logic.enableArmControl(false)
                .then( (res) => Logic.doDlillale(blobIdCombo.currentText, depthSpin.value*0.01, drillSpeedSpin.value*0.001) )
                progress = 0

            }
        }


        Button {
            Layout.fillWidth: true
            text: 'ABORT'
            width: parent.width

            onReleased: {
                Logic.abortDlillale()
            }
        }

        Item {
            Layout.fillHeight: true
        }

    }

}
