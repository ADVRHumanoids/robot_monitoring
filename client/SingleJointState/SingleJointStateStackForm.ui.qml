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

            Repeater {
                id: loader
                model: []
                Loader {
                    sourceComponent: root.jointStateComponent

                    onLoaded: {
                        item.jName = modelData.jName
                    }
                }
            }
        }

    }
}
