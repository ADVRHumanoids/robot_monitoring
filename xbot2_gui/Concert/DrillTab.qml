import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import Common

import "Drilling.js" as Logic

Control {

    property list<string> blobIds

    property alias selectedBlob: blobIdCombo.currentText

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
            model: []
            width: parent.width
        }

        Button {
            text: 'Update blobs'
            onClicked: blobIdCombo.model = root.blobIds
            Layout.fillWidth: true
        }

        Label {
            text: ' select depth (cm)'
            font.pointSize: 10
            topPadding: 4
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
            Layout.fillHeight: true
        }

        Label {
            id: blobIdLabel
            text: selectedBlob.length === 0 ?
                      'no blob selected' :
                      `will drill '${selectedBlob}''`
            Layout.fillWidth: true
            font.pixelSize: CommonProperties.font.h3
        }

        DelayButton {
            Layout.fillWidth: true
            text: 'Execute drill'
            width: parent.width

            onActivated: {
                Logic.enableArmControl(false)
                .then( (res) => Logic.doDlillale(selectedBlob,
                                                 depthSpin.value*0.01,
                                                 drillSpeedSpin.value*0.001) )
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

    }

}
