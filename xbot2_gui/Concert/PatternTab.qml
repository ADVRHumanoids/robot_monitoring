import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Common
import "Drilling.js" as Logic

Control {

    padding: 4
    topPadding: 8
    bottomPadding: 8

    contentItem: GridLayout {

        columns: 1

        Label {
            topPadding: 4
            text: ' pattern type'
            font.pointSize: 10
        }

        ComboBox {
            id: patternCombo
            model: ['horizontal', 'vertical']
            Layout.fillWidth: true
        }

        Label {
            text: ' pattern length [cm]'
            font.pointSize: 10
        }

        SpinBox {
            id: lengthSpin
            from: 0
            to: 100
            value: 60
            editable: true
            Layout.fillWidth: true
        }

        Label {
            text: ' num points'
            font.pointSize: 10
        }

        SpinBox {
            id: pointsSpin
            from: 1
            to: 100
            value: 3
            editable: true
            Layout.fillWidth: true
        }

        Label {
            topPadding: 4
            text: ' pattern x-offset [cm]'
            font.pointSize: 10

        }

        SpinBox {
            id: xOffsetSpin
            from: -100
            to: 100
            editable: true
            Layout.fillWidth: true
        }

        Label {
            topPadding: 4
            text: ' pattern y-offset [cm]'
            font.pointSize: 10
        }

        SpinBox {
            id: yOffsetSpin
            from: -100
            to: 100
            editable: true
            Layout.fillWidth: true
        }

        Label {
            topPadding: 4
            text: ' drill depth [cm]'
            font.pointSize: 10
        }

        SpinBox {
            id: drillDepthSpin
            from: 0
            to: 20
            value: 5
            editable: true
            Layout.fillWidth: true
        }

        Item {

            Layout.fillHeight: true
        }

        DelayButton {
            text: 'Drill pattern'
            Layout.fillWidth: true
            onActivated: {
                Logic.drillPattern()
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
