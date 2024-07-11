import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Item {

    property alias blobIds: drillTab.blobIds
    property alias motionTarget: motionTab.motionTarget
    property alias maxSpeed: motionTab.maxSpeed
    property alias joyXEnabled: motionTab.joyXEnabled
    property alias joyYEnabled: motionTab.joyYEnabled
    property alias joyZEnabled: motionTab.joyZEnabled
    property bool motionMode: bar.currentIndex === 0
    property bool drillMode: bar.currentItem.text === 'Blob'
    property alias armEE: motionTab.armEE
    property alias armEEOptions: motionTab.armEEOptions
    signal modeChanged()

    //
    id: root
    implicitHeight: lay.implicitHeight
    implicitWidth: lay.implicitWidth


    ColumnLayout {

        id: lay

        anchors.fill: parent


        TabBar {

            id: bar

            anchors {
                top: parent.top
                left: parent.left
                right: parent.right
            }

            onCurrentIndexChanged: root.modeChanged()

            TabButton {
                text: 'Jogging'
            }

            TabButton {
                text: 'Blob'
            }

            TabButton {
                text: 'Pattern'
            }

        }

        StackLayout {

            id: stack

            currentIndex: bar.currentIndex

            anchors {
                top: bar.bottom
                bottom: parent.bottom
                left: parent.left
                right: parent.right
            }

            MotionTab {
                id: motionTab
            }

            DrillTab {
                id: drillTab
            }

            PatternTab {
                id: patternTab
            }
        }

    }

}
