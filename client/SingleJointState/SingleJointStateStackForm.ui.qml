import QtQuick 2.4
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12

Item {

    property alias stack: stack
    property alias loader: loader

    implicitWidth: stack.implicitWidth

    ScrollView
    {
        id: scroll
        anchors.fill: parent
        contentWidth: availableWidth

        StackLayout {

            id: stack
            anchors.fill: parent
            currentIndex: root.currentIndex

            Repeater {
                id: loader
                model: jointNames
                Loader {

                    active: index === stack.currentIndex
                    sourceComponent: root.jointStateComponent

                    onLoaded: {
                        item.jName = jointNames[index]
                    }
                }
            }
        }

    }
}
