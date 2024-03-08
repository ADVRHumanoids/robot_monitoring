import QtQuick
import QtQuick.Layouts
import QtQuick.Controls

import Main
import Common

import "Ecat.js" as Logic

Control {

    property ClientEndpoint client

    signal pageSelected()

    property list<string> jointNames

    padding: CommonProperties.geom.margins

    contentItem: GridLayout {

        columnSpacing: CommonProperties.geom.margins

        columns: 5

        id: grid

        //
        Label {
            Layout.columnSpan: grid.columns
            text: 'Handle with care ! Expert users only !!'
            font.pixelSize: CommonProperties.font.h1
            bottomPadding: 20
        }

        //
        Label {
            text: 'Select Motor'
        }

        ComboBox {
            id: motorCombo
            model: jointNames
        }

        Item {
            Layout.columnSpan: 3
            Layout.fillWidth: true
        }

        //
        Label {
            text: 'Stop Motor'
        }

        Item {

        }

        Button {
            text: 'Stop'
            onClicked: {
                Logic.stop(motorCombo.currentText)
            }
        }

        Item {
            Layout.columnSpan: 2
            Layout.fillWidth: true
        }

        //
        Label {
            text: 'Start Motor'
        }

        ComboBox {
            id: ctrlCombo
            model: ['Position', 'Velocity', 'Impedance']
        }

        Button {
            text: 'Start'
            onClicked: {
                Logic.start(motorCombo.currentText, ctrlCombo.currentText)
            }
        }

        Item {
            Layout.columnSpan: 2
            Layout.fillWidth: true
        }

        //
        Item {
            Layout.fillHeight: true
        }

    }

    onPageSelected: {
        Logic.construct()
    }

}
