import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Common
import Main
import ExpandableBottomBar

Item {

    id: root
    property ClientEndpoint client

    MultiColumnLayout {

        id: mcl

        anchors.fill: parent
        anchors.margins: 30

        columns: spin.value

//        Card1 {
//            id: card
//            name: 'Prova Prova'
//            frontItem: Column {
//                id: col
//                spacing: 10
//                Repeater {
//                    Rectangle {
//                        required property int index
//                        height: 100
//                        width: card.availableContentWidth
//                        color: 'red'
//                        Text {
//                            text: parent.index
//                            anchors.centerIn: parent
//                        }
//                    }
//                    model: 10
//                }
//            }
//        }

        Repeater {

            model: spin1.value

            Rectangle {

                required property int index
                color: Qt.rgba(0, 0, 0, 0)
                border.color: 'red'
                border.width: 2
                height: 100
                implicitWidth: 1

                property int columnSpan: index == 0 ? 2 : 1

                Behavior on height {
                    NumberAnimation {

                    }
                }

                Button {
                    anchors.centerIn: parent
                    onClicked: {
                        parent.height += 50
                    }
                }

                Text {
                    text: parent.index
                    anchors {
                        top: parent.top
                        left: parent.left
                        margins: 10
                    }
                    color: 'red'
                }
            }
        }
    }

    Row {

        spacing: 12

        Button {
            text: 'Compute Layout'
            onClicked: mcl.computeLayout()
        }

        SpinBox {
            from: 1
            to: 10
            value: 3
            id: spin
        }

        SpinBox {
            from: 1
            to: 10
            value: 3
            id: spin1
        }

    }

    //    Card1 {
    //        id: card
    //        anchors.centerIn: parent
    //        width: 400
    //        name: 'Prova Prova'
    //        frontItem: Column {
    //            id: col
    //            spacing: 10
    //            Repeater {
    //                Rectangle {
    //                    required property int index
    //                    height: 100
    //                    width: card.availableContentWidth
    //                    color: 'red'
    //                    Text {
    //                        text: parent.index
    //                        anchors.centerIn: parent
    //                    }
    //                }
    //                model: 10
    //            }
    //        }
    //    }


    //    DebugRectangle {
    //        name: 'acci'
    //        target: card
    //    }

    NavigationBar {



        anchors {
            left: parent.left
            leftMargin: 50
            right: parent.right
            rightMargin: 50
            bottom: parent.bottom
            bottomMargin: 50
        }

        NavigationBarButton {
            text: "Home"
            icon.source: "/ExpandableBottomBar/assets/home.png"
            palette {
                buttonText: "#969696"
                highlight: "#FBE5EE"
                highlightedText: "#EE7CA4"
            }
            onCheckedChanged: {
                if (checked) {
                    root.color = palette.highlight
                }
            }
        }

        NavigationBarButton {
            text: "Today"
            icon.source: "/ExpandableBottomBar/assets/sun.png"
            palette {
                buttonText: "#969696"
                highlight: "#dfdfdf"
                highlightedText: "#888888"
            }
            onCheckedChanged: {
                if (checked) {
                    root.color = palette.highlight
                }
            }
        }

        NavigationBarButton {
            text: "Done"
            icon.source: "/ExpandableBottomBar/assets/check.png"
            palette {
                buttonText: "#969696"
                highlight: "#fbe8e7"
                highlightedText: "#f08e8b"
            }
            onCheckedChanged: {
                if (checked) {
                    root.color = palette.highlight
                }
            }
        }

        NavigationBarButton {
            text: "Settings"
            icon.source: "/ExpandableBottomBar/assets/settings.png"
            palette {
                buttonText: "#969696"
                highlight: "#dcebfb"
                highlightedText: "#56a2ec"
            }
            onCheckedChanged: {
                if (checked) {
                    root.color = palette.highlight
                }
            }
        }
    }

}

