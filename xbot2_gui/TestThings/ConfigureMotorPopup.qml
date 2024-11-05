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

    property var selectedMotorProperties: motorProperties[motorCombo.currentText]

    property string selectedMotorType

    property real loadMass: -1

    property real loadRadius: -1

    function refresh() {
        Logic.updateMotorProperties()
    }

    //
    id: root

    contentItem: ColumnLayout {

        RowLayout {

            spacing: 16

            Label {
                text: 'Motor type'
                verticalAlignment: Text.AlignVCenter
                anchors.verticalCenter: parent.verticalCenter
            }

            ComboBox {
                id: motorCombo
            }

            Item {
                Layout.preferredWidth: 24
            }

            CheckBox {
                id: lockedOutputCheck
                text: 'Locked Output'
            }

        }

        RowLayout {

            spacing: 16

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

            Item {
                Layout.preferredWidth: 24
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
        }

        GridLayout {

            Layout.fillWidth: true
            Layout.fillHeight: true

            columns: 2

            Label {
                text: 'Max Velocity'
                verticalAlignment: Text.AlignVCenter
            }

            TextField {
                readOnly: true
                text: selectedMotorProperties['max_velocity']
            }


            Label {
                text: 'Max Torque'
                verticalAlignment: Text.AlignVCenter
            }

            TextField {
                readOnly: true
                text: selectedMotorProperties['max_torque']
            }

        }

    }

}
