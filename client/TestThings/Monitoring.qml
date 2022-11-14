import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import xbot2_gui.common

import "../SingleJointState"
import "../BarPlot"
import "../Plotter"
import ".."


Item {

    id: root
    property ClientEndpoint client: undefined
    property bool mobileLayout: width < mainGrid.brMedium

    TabBar {
        id: bar
        width: mainGrid.contentWidth
        anchors.horizontalCenter: parent.horizontalCenter
        TabButton {
            text: 'Joint Monitoring'
        }
        TabButton {
            text: 'Plot'
        }
    }

    Component.onCompleted: {

        let jsCallback = function(js) {
            barPlot.setJointStateMessage(js)
            jointState.setJointStateMessage(js)
            plotter.setJointStateMessage(js)
        }

        client.jointStateReceived.connect(jsCallback)
    }

    SwipeView {
        id: mainSwipe
        width: parent.width
        anchors {
            top: bar.bottom
            bottom: parent.bottom
        }
        currentIndex: bar.currentIndex

        ScrollView {

            id: scroll
            contentWidth: availableWidth
            contentHeight: mainGrid.implicitHeight

            MaterialResponsiveGrid {
                id: mainGrid
                brMedium: 900
                width: scroll.contentWidth
                height: scroll.contentHeight

                Rectangle {

                    color: CommonProperties.colors.cardBackground
                    height: barPlot.implicitHeight + barPlotTitle.height + 16*5
                    radius: 4
                    property var columnSpan: [4, 8, 8, 8]

                    Label {
                        id: barPlotTitle
                        text: "Joint overview"
                        font.pixelSize: CommonProperties.font.h1
                        x: 16
                        y: 16
                    }

                    BarPlotStack {
                        id: barPlot

                        anchors {
                            top: barPlotTitle.bottom
                            bottom: parent.bottom
                            left: parent.left
                            right: parent.right
                        }

                        anchors.margins: 16

                        onJointClicked: jointName => {
                            jointState.selectJoint(jointName)
                        }
                    }

                }

                Rectangle {

                    color: Qt.lighter(Material.background)
                    height: jointState.implicitHeight + singleJointTitle.height + 16*4
                    radius: 4
                    property var columnSpan: [4, 8, 4, 4]

                    Label {
                        id: singleJointTitle
                        text: "Joint data"
                        font.pixelSize: CommonProperties.font.h1
                        x: 16
                        y: 16
                    }

                    SingleJointStateStack {
                        id: jointState
                        anchors {
                            top: singleJointTitle.bottom
                            bottom: parent.bottom
                            left: parent.left
                            right: parent.right
                        }

                        anchors.margins: 16

                        onPlotAdded: (jName, fieldName) => {
                            plotter.addSeries(jName, fieldName)
                        }
                    }
                }
            }
        }

        MaterialResponsiveGrid {
            id: plotterGrid

            Plotter {
                id: plotter
                property alias columnSpan: plotterGrid.columns
                height: parent.contentHeight
            }
        }

    }

}