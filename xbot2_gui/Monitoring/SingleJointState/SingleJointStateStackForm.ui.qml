import QtQuick 2.4
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import "../../sharedData.js" as SharedData
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
                active: index === stack.currentIndex
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
