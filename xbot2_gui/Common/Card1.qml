import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Common


Item {



    // public

    property string name: 'CardName'

    property int margins: CommonProperties.geom.spacing

    property color backgroundColor: defaultBackground

    readonly property color defaultBackground: CommonProperties.colors.cardBackground

    property alias nameFont: titleLabel.font

    property int maxContentHeight: 100000

    property alias availableContentWidth: frontScroll.availableWidth

    property alias availableContentHeight: frontScroll.availableHeight

    property bool flipped: false

    property bool collapsable: true

    property bool collapsed: false

    property bool configurable: true

    property Item frontItem: Item {

    }

    property Item backItem: Item {

    }

    property list<Item> toolButtons

    signal applyConfiguration()


    // private
    id: root

    Component.onCompleted: {
        frontItem.parent = frontScrollContent
        backItem.parent = flip.backSide.contentItemWrapper
    }

    implicitWidth: flip.implicitWidth
    implicitHeight: flip.implicitHeight

    height: flip.height
    clip: true

    Behavior on height {
        NumberAnimation {
            duration: 333
            easing.type: Easing.OutQuad
        }
    }

    Flipable {

        id: flip

        implicitWidth: root.flipped ? backSide.implicitWidth : frontSide.implicitWidth
        implicitHeight: root.flipped ? backSide.implicitHeight : frontSide.implicitHeight

        width: parent.width
        height: root.flipped ? backSide.height : frontSide.height

        // front side is rendered as a rectangle whose content is layed out in a column
        // with header (card title and tool buttons) and content (item)
        property Item frontSide: Rectangle {

            property alias contentItemWrapper: frontScroll

            id: frontSideRoot
            width: flip.width
            height: frontColumn.height
            color: root.backgroundColor
            radius: CommonProperties.geom.cardRadius

            implicitHeight: frontColumn.implicitHeight
            implicitWidth: frontColumn.implicitWidth

            Component.onCompleted: {
                for(let i = 0; i < root.toolButtons.length; i++) {
                    let tb = root.toolButtons[i]
                    tb.parent = toolBtnRowInner
                    try {
                        tb.font.pixelSize = CommonProperties.font.h3
                    }
                    catch(error) { }
                    //                    tb.anchors.verticalCenter = toolBtnRow.verticalCenter
                }
            }

            // column holding banner (title + toolbuttons) and content
            Column {

                id: frontColumn
                width: parent.width
                spacing: root.margins

                // row with tool buttons
                RowLayout {

                    id: toolBtnRow

                    anchors {
                        left: parent.left
                        right: parent.right
                        margins: root.margins
                    }

                    spacing: 0

                    // banner
                    Label {
                        id: titleLabel
                        text: root.name
                        font.pixelSize: CommonProperties.font.h2
                        verticalAlignment: Text.AlignVCenter
                        Layout.fillWidth: true
                        wrapMode: Text.Wrap
                    }

                    RowLayout {
                        id: toolBtnRowInner
                        Layout.alignment: Qt.AlignVCenter
                    }

                    // configuration button
                    SmallToolButton {
                        id: configureBtn

                        visible: root.configurable
                        Layout.alignment: Qt.AlignVCenter
                        text: '\uf013'
                        font.family: CommonProperties.fontAwesome.solid.family
                        font.pixelSize: CommonProperties.font.h3

                        onClicked: {
                            root.flipped = true
                            root.collapsed = false
                        }
                    }

                    // expand/collpse button
                    SmallToolButton {
                        id: showHideBtn
                        visible: root.collapsable
                        Layout.alignment: Qt.AlignVCenter
                        text: root.collapsed ? '\uf078' : '\uf077'
                        font.family: CommonProperties.fontAwesome.solid.family
                        font.pixelSize: CommonProperties.font.h3

                        onClicked: {
                            root.collapsed = !root.collapsed
                        }
                    }

                }

                Item {

                    id: frontItemWrapper

                    anchors {
                        left: parent.left
                        right: parent.right
                        margins: root.margins
                    }

                    height: root.collapsed ? 0 : Math.min(frontScroll.implicitHeight, root.maxContentHeight)
                    implicitWidth: frontScroll.implicitWidth



                    ScrollView {
                        id: frontScroll
                        anchors.fill: parent
                        contentWidth: availableWidth
                        contentHeight: frontScrollContent.height
                        clip: true
                        bottomPadding: 10

                        Item {
                            id: frontScrollContent
                            width: frontScroll.availableWidth
                            height: children.length > 0 ? children[0].height : 0
                        }
                    }
                }
            }
        }

        property Item backSide: Control {

            property alias contentItemWrapper: backItemWrapper

            width: flip.width
            height: titleLabelBack.height + backItemWrapper.height + cfgOkBtn.height + 2*root.margins


            implicitWidth: backColumn.implicitWidth

            implicitHeight: backColumn.implicitHeight

            background: Rectangle {
                color: root.backgroundColor
                radius: CommonProperties.geom.cardRadius
            }

            padding: root.margins

            contentItem: Column {

                id: backColumn

                width: parent.width


                Label {
                    id: titleLabelBack
                    text: root.name
                    font.pixelSize: CommonProperties.font.h2
                }

                Item {
                    width: parent.width
                    height: root.margins
                }

                Item {
                    id: backItemWrapper
                    implicitWidth: children[0].implicitWidth
                    implicitHeight: children[0].implicitHeight
                    width: parent.width
                    height:  children.length > 0 ? children[0].height : 0
                    clip: true
                }

                Row {

                    spacing: root.margins

                    Button {
                        id: cfgOkBtn
                        text: 'Ok'
                        onReleased: {
                            root.flipped = false
                            root.applyConfiguration()
                        }
                    }

                    Button {
                        id: cfgCancelBtn
                        text: 'Cancel'
                        onReleased: {
                            root.flipped = false
                        }
                    }

                }

            }

        }

        front: frontSide

        back: backSide

        transform: Rotation {
            id: rotation
            origin.x: root.width/2
            origin.y: root.height/2
            axis.x: 0; axis.y: 1; axis.z: 0     // set axis.y to 1 to rotate around y-axis
            angle: 0    // the default angle
        }

        states: State {
            name: "back"
            PropertyChanges { target: rotation; angle: 180 }
            when: root.flipped
        }

        transitions: Transition {
            NumberAnimation {
                target: rotation
                property: "angle"
                duration: 500
                easing.type: Easing.OutQuad
            }
        }
    }

}
