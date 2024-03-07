import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import "/qt/qml/Main/sharedData.js" as SharedData

Item {

    property alias stack: stack
    property alias repeater: repeater

    implicitWidth: stack.implicitWidth
    implicitHeight: stack.implicitHeight


    StackLayout {

        id: stack
        anchors.fill: parent
        currentIndex: root.currentIndex

        Repeater {

            id: repeater
            model: jointNames

            Loader {
                id: loader
                active: true
                sourceComponent: root.jointStateComponent

                Connections {

                    target: loader.item

                    function onPlotAdded(jName, fieldName) {
                        root.plotAdded(jName, fieldName)
                    }

                    function onPlotRemoved(jName, fieldName) {
                        root.plotRemoved(jName, fieldName)
                    }
                }

                onLoaded: {
                    item.jName = jointNames[index]
                    item.setJointStateMessage(SharedData.latestJointState)
                }
            }

        }

    }
}
