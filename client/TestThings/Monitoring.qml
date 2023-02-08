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

        onCurrentIndexChanged: {
            console.log('tab bar index changed to ' + currentIndex)
            mainSwipe.setCurrentIndex(currentIndex)
        }
    }

    function _jsCallback(js) {
        barPlot.setJointStateMessage(js)
        jointState.setJointStateMessage(js)
        plotter.setJointStateMessage(js)
    }

    function _objCallback(obj) {
        if(obj.type === 'joint_device_info') {
            jointDevice.filterActive = obj.filter_active
            jointDevice.filterCutoff = obj.filter_cutoff_hz
            jointDevice.jointActive = obj.joint_active
        }
    }

    Component.onCompleted: {

        client.jointStateReceived.connect(_jsCallback)
        client.objectReceived.connect(_objCallback)
    }

    Component.onDestruction: {
        client.jointStateReceived.disconnect(_jsCallback)
        client.objectReceived.disconnect(_objCallback)
    }

    SwipeView {
        id: mainSwipe
        width: parent.width
        anchors {
            top: bar.bottom
            bottom: parent.bottom
        }

        onCurrentIndexChanged: {
            console.log('swipe index changed to ' + currentIndex)
            bar.setCurrentIndex(currentIndex)
        }

        ScrollView {

            id: scroll
            contentWidth: availableWidth
            contentHeight: mainGrid.implicitHeight

            MaterialResponsiveGrid {
                id: mainGrid
                brMedium: 900
                width: scroll.contentWidth

                JointDevice {
                    id: jointDevice
                    property int column: 0
                    property var columnSpan: [4, 8, 4, 4]
                }

                Rectangle {

                    color: CommonProperties.colors.cardBackground
                    height: childrenRect.height + 16*2

                    radius: 4
                    property int column: 0
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

                        onPlotRemoved: (jName, fieldName) => {
                                           plotter.removeSeries(jName, fieldName)
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
