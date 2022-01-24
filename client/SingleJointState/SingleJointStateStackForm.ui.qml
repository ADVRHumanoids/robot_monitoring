import QtQuick 2.4
import QtQuick.Controls
import QtQuick.Layouts

Item {

    property alias stack: stack

    StackLayout {
        id: stack
        currentIndex: jointStateStack.currentIndex
        anchors.fill: parent
    }
}
