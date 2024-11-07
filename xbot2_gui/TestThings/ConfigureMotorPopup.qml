import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtWebView

import Common
import Main
import ExpandableBottomBar
import Font
import Menu
import Joy
import LivePlot

import "MotionTab.js" as Logic

Control {

    property ClientEndpoint client

    property var motorProperties

    property var selectedMotorProperties

    property string selectedMotorType

    property real loadMass: -1

    property real loadRadius: -1

    property bool configured: selectedMotorType.length > 0 && loadMass >= 0 && loadRadius >= 0

    signal done()

    function refresh() {
        Logic.updateMotorProperties()
    }

    //
    id: root

    contentItem: ColumnLayout {

        GridLayout {

            columns: 2

            Label {
                text: 'Motor type'
                verticalAlignment: Text.AlignVCenter
            }

            ComboBox {
                id: motorCombo
                editable: true
            }

            Label {
                text: 'Load radius [cm]'
            }

            TextField {
                id: loadRadiusTxt
                text: '0.0'
                validator: DoubleValidator {
                    bottom: 0
                    top: 50
                    notation: DoubleValidator.StandardNotation
                }
            }

            Label {
                text: 'Load mass [kg]'
            }

            TextField {
                id: loadMassTxt
                text: '0.0'
                validator: DoubleValidator {
                    bottom: 0
                    top: 30.0
                    notation: DoubleValidator.StandardNotation
                }
            }

            Item {
                Layout.columnSpan: 2
                Layout.preferredHeight: 24
            }

            Label {
                text: 'Properties'
                Layout.columnSpan: 2
            }

            Label {
                text: 'Max Velocity'
                verticalAlignment: Text.AlignVCenter
            }

            TextField {
                readOnly: true
                text: selectedMotorProperties['max_velocity']
                enabled: false
            }


            Label {
                text: 'Max Torque'
                verticalAlignment: Text.AlignVCenter
            }

            TextField {
                readOnly: true
                text: selectedMotorProperties['max_torque']
                enabled: false
            }

        }

        Row {

            spacing: 16

            Button {

                text: 'Ok'
                Keys.onReturnPressed: clicked()
                onClicked: {
                    selectedMotorType = motorCombo.currentText
                    loadMass = parseFloat(loadMassTxt.text)
                    loadRadius = parseFloat(loadRadiusTxt.text)*0.01
                    console.log(`${selectedMotorType} ${loadMass} ${loadRadius}`)
                    selectedMotorProperties = motorProperties[selectedMotorType]
                }
            }


            Button {

                text: 'Close'
                onClicked: {
                    done()
                }
            }

            Layout.alignment: Qt.AlignRight
        }

    }

}
