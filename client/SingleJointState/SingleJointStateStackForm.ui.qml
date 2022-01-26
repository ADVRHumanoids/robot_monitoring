import QtQuick 2.4
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12

Item {

    property alias stack: stack

    StackLayout {
        id: stack
        currentIndex: jointStateStack.currentIndex
        anchors.fill: parent
    }
}
