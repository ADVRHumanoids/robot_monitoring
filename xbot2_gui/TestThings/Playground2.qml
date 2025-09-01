import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Main



Item {

    id: root
    property ClientEndpoint client

    // function appendText(i, txt) {
    //     txt = `<font color="red">` + txt + '</font>'
    //     rep.itemAt(i).model.append({'txt': txt})
    //     rep.itemAt(i).scrollToEnd()
    // }


    // ColumnLayout {

    //     anchors.fill: parent

    //     TabBar {
    //         id: bar
    //         Layout.fillWidth: true

    //         TabButton {
    //             text: 'A'
    //         }

    //         TabButton {
    //             text: 'B'
    //         }
    //     }

    //     StackLayout {

    //         currentIndex: bar.currentIndex

    //         Layout.fillHeight: true
    //         Layout.fillWidth: true

    //         StackLayout {

    //             Layout.fillHeight: true
    //             Layout.fillWidth: true


    //             Flipable {
    //                 id: flip
    //                 property bool flipped: false
    //                 Layout.fillHeight: true
    //                 Layout.fillWidth: true

    //                 front: StackLayout {

    //                     height: flip.height
    //                     width: flip.width
    //                     currentIndex: pageSpin.value

    //                     Repeater {
    //                         id: rep
    //                         model: 5

    //                         Item {
    //                             property alias listview: view
    //                             property alias model: listModel
    //                             required property int index
    //                             Layout.fillHeight: true
    //                             Layout.fillWidth: true
    //                             implicitHeight: view.implicitHeight
    //                             implicitWidth: view.implicitWidth

    //                             function scrollToEnd() {
    //                                 Qt.callLater(view.positionViewAtEnd)
    //                             }

    //                             ListModel {
    //                                 id: listModel
    //                             }

    //                             ListView {

    //                                 anchors.fill: parent

    //                                 id: view
    //                                 model: listModel
    //                                 delegate: TextEdit {
    //                                     text: txt
    //                                     width: view.width
    //                                     wrapMode: Text.WrapAnywhere
    //                                     readOnly: true
    //                                     textFormat: TextEdit.RichText
    //                                     // color: palette.active.text
    //                                 }
    //                                 implicitHeight: contentHeight
    //                                 spacing: 1
    //                                 clip: true
    //                                 ScrollBar.vertical: ScrollBar {
    //                                     active: true
    //                                 }

    //                             }

    //                         }
    //                     }
    //                 }

    //                 transform: Rotation {
    //                     id: rotation
    //                     origin.x: flip.width/2
    //                     origin.y: flip.height/2
    //                     axis.x: 0; axis.y: 1; axis.z: 0     // set axis.y to 1 to rotate around y-axis
    //                     angle: 0    // the default angle
    //                 }

    //                 states: State {
    //                     name: "back"
    //                     PropertyChanges { target: rotation; angle: 180 }
    //                     when: flip.flipped
    //                 }

    //                 transitions: Transition {
    //                     NumberAnimation { target: rotation; property: "angle"; duration: 400 }
    //                 }

    //             }

    //         }

    //         Rectangle {
    //             color: 'red'
    //         }

    //     }

    //     RowLayout {

    //         Layout.fillWidth: true

    //         Button {
    //             text: 'Generate a lot of text'
    //             Layout.fillWidth: true
    //             onClicked: {
    //                 for(let i = 0; i < 10000; i++) {
    //                     appendText(pageSpin.value, 'Example text Example text Example text Example text Example text Example text Example text Example text Example text Example text Example text Example text Example text ')
    //                 }
    //             }
    //         }

    //         Button {
    //             text: 'Flip'
    //             onClicked: flip.flipped = !flip.flipped
    //         }

    //         SpinBox {
    //             from: 0
    //             to: 5
    //             id: pageSpin
    //         }

    //     }
    // }


    LauncherConsoleItem2 {
        anchors.fill: parent
    }

}

