import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Common
import Font
import "/qt/qml/Main/sharedData.js" as SharedData

Item {

    property alias currentIndex: stack.currentIndex

    property list<string> jointNames: SharedData.jointNames

    function setJointStateMessage(js) {
        for(let aux of js.aux_types) {
            if(auxTypes.indexOf(aux) < 0) {
                fieldsModel.append(
                            {
                                'shortName': aux,
                                'longName': aux,
                                'numeric': true,
                                'precision': 1,
                                'enablePlot': true
                            })
                auxTypes.push(aux)
            }
        }
    }

    function selectJoint(jName) {
        currentIndex = jointNames.indexOf(jName)
    }

    signal plotAdded(string jName, string fieldName)

    signal setFaultCode(string jName, string faultCode)

    //
    id: root
    implicitHeight: stack.implicitHeight
    implicitWidth: stack.implicitWidth

    property list<string> auxTypes

    MaterialSymbols {
        id: syms
    }

    ListModel {

        id: fieldsModel

        ListElement {
            shortName: 'posRef'
            longName: 'Pos. Ref.'
            precision: 3
            enablePlot: true
            numeric: true
        }

        ListElement {
            shortName: 'linkPos'
            longName: 'Link Pos.'
            precision: 3
            enablePlot: true
            numeric: true
        }

        ListElement {
            shortName: 'motPos'
            longName: 'Mot. Pos.'
            precision: 3
            enablePlot: true
            numeric: true
        }


        ListElement {
            shortName: 'velRef'
            longName: 'Vel. Ref.'
            precision: 1
            enablePlot: true
            numeric: true
        }

        ListElement {
            shortName: 'linkVel'
            longName: 'Link Vel.'
            precision: 1
            enablePlot: true
            numeric: true
        }

        ListElement {
            shortName: 'motVel'
            longName: 'Mot. Vel.'
            precision: 1
            enablePlot: true
            numeric: true
        }


        ListElement {
            shortName: 'torFfwd'
            longName: 'Torque Ffwd'
            precision: 1
            enablePlot: true
            numeric: true
        }
        ListElement {
            shortName: 'torRef'
            longName: 'Torque Ref.'
            precision: 1
            enablePlot: true
            numeric: true
        }
        ListElement {
            shortName: 'tor'
            longName: 'Torque'
            precision: 1
            enablePlot: true
            numeric: true
        }


        ListElement {
            shortName: 'k'
            longName: 'Stiffness'
            precision: 0
            enablePlot: true
            numeric: true
        }

        ListElement {
            shortName: 'd'
            longName: 'Damping'
            precision: 0
            enablePlot: true
            numeric: true
        }

        ListElement {
            shortName: ''
            longName: ''
            precision: 0
        }


        ListElement {
            shortName: 'motorTemp'
            longName: 'Motor Temp.'
            precision: 0
            enablePlot: true
            numeric: true
        }

        ListElement {
            shortName: 'driverTemp'
            longName: 'Driver. Temp.'
            precision: 0
            enablePlot: true
            numeric: true
        }


        ListElement {
            shortName: ''
            longName: ''
            precision: 0
        }

        ListElement {
            shortName: ''
            longName: 'Fault Code'
            precision: 0
            numeric: false
        }


        ListElement {
            shortName: ''
            longName: ''
            precision: 0
        }

        ListElement {
            shortName: ''
            longName: ''
            precision: 0
        }

    }

    Timer {
        id: updTimer
        repeat: true
        interval: 333
        running: true
    }

    StackLayout {

        id: stack

        anchors.fill: parent

        Repeater {

            id: repeater

            model: root.jointNames.length

            GridLayout {

                id: grid

                Layout.fillHeight: true
                Layout.fillWidth: true
                rowSpacing: -6
                columnSpacing: 10

                required property int index

                property string jName: root.jointNames[index]

                columns: 9

                // normal fields
                Repeater {

                    model: fieldsModel

                    delegate: Item {

                        // required property int index

                        visible: false

                        Label {
                            text: longName
                            Layout.preferredWidth: 50
                            Layout.maximumWidth: 50
                            Layout.minimumWidth: 50
                            wrapMode: Text.WordWrap
                        }

                        Label {

                            id: valueLabel

                            property bool isFault: longName === 'Fault Code'

                            Layout.fillWidth: true
                            Layout.preferredWidth: 1
                            Layout.columnSpan: 1

                            text: '--'
                            background: Rectangle {
                                color: Qt.rgba(1, 1, 1, 0.1)
                                radius: 4
                            }
                            topPadding: 4
                            bottomPadding: 4
                            leftPadding: 4
                            rightPadding: 4
                            opacity: shortName !== '' || isFault ? 1 : 0
                            horizontalAlignment: isFault ? Text.AlignLeft : Text.AlignRight

                            Connections {
                                target: updTimer
                                onTriggered: function() {

                                    if(shortName === '') {
                                        return
                                    }

                                    try {

                                        valueLabel.text =
                                                SharedData.latestJointState[shortName][grid.index].toFixed(precision)

                                    }
                                    catch(error) {

                                    }
                                }
                            }

                            Connections {
                                enabled: valueLabel.isFault
                                target: root
                                onSetFaultCode: function(jName, faultCode) {
                                    if(jName !== grid.jName) {
                                        return
                                    }

                                    valueLabel.text = faultCode
                                    valueLabel.color = 'red'
                                    valueLabel.font.bold = true
                                    valueLabel.font.italic = false

                                    faultExpiredTimer.restart()
                                }
                            }

                            Timer {
                                id: faultExpiredTimer
                                interval: 1000
                                repeat: false
                                onTriggered: {
                                    valueLabel.color = valueLabel.palette.text
                                    valueLabel.font.bold = false
                                    valueLabel.font.italic = true
                                }
                            }
                        }

                        Control {

                            contentItem: SmallToolButton {
                                text: MaterialSymbolNames.plotSmall
                                font.family: syms.font.family
                                font.pixelSize: 18
                                opacity: enablePlot ? 1 : 0

                                onClicked: root.plotAdded(jName, shortName)
                            }

                            leftPadding: -12
                            rightPadding: 4

                        }

                        Component.onCompleted: {
                            while (children.length > 0) children[0].parent = grid;
                        }

                    }

                }

            }

        }

    }

}



