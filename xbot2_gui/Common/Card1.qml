import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Common


Item {

    id: root

    // public

    property string name: 'CardName'

    property int margins: CommonProperties.geom.spacing

    property color backgroundColor: defaultBackground

    readonly property color defaultBackground: CommonProperties.colors.cardBackground

    property alias nameFont: titleLabel.font

    property int contentHeight: 300

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

    Timer {
        running: true
        interval: 1000
        repeat: true
        onTriggered: {
            console.log(toolBtnRow.implicitWidth)
        }
    }


    // private

    Component.onCompleted: {
        frontItem.parent = flip.frontSide.contentItemWrapper
        backItem.parent = flip.backSide.contentItemWrapper
    }

    implicitWidth: flip.implicitWidth
    implicitHeight: flip.implicitHeight
    height: flip.height

    Flipable {

        id: flip

        implicitWidth: root.flipped ? backSide.implicitWidth : frontSide.implicitWidth
        implicitHeight: root.flipped ? backSide.implicitHeight : frontSide.implicitHeight

        width: parent.width
        height: frontSide.height

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
                    tb.parent = toolBtnRow
                    try {
                        tb.font.pixelSize = CommonProperties.font.h3
                    }
                    catch(error) { }
                    tb.anchors.verticalCenter = toolBtnRow.verticalCenter
                }
            }

            // column holding banner (title + toolbuttons) and content
            Column {

                id: frontColumn
                width: parent.width

                // row with tool buttons
                RowLayout {

                    id: toolBtnRow

                    width: parent.width

                    spacing: root.margins

                    //                    layoutDirection: Qt.RightToLeft

                    // banner
                    Label {
                        id: titleLabel
                        //width: parent.width
                        text: root.name
                        font.pixelSize: CommonProperties.font.h2
                        height: implicitHeight + 2*root.margins
                        verticalAlignment: Text.AlignVCenter
                    }

                    Item {
                        implicitHeight: 1
                        implicitWidth: 1
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        DebugRectangle {
                            target: parent
                            border.color: 'green'
                        }
                    }

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

                    SmallToolButton {
                        id: showHideBtn
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

                    default property alias content: frontScroll.contentData

                    anchors {
                        left: parent.left
                        right: parent.right
                        margins: root.margins
                    }

                    height: root.collapsed ? 0 : root.contentHeight

                    Behavior on height {
                        NumberAnimation {
                            duration: 1000
                        }
                    }

                    ScrollView {
                        id: frontScroll
                        anchors.fill: parent
                        contentWidth: availableWidth
                        clip: true
                    }

                }

            }

        }

        property Item backSide: Rectangle {

            property alias contentItemWrapper: backItemWrapper

            width: flip.width
            height: flip.height
            color: root.backgroundColor
            radius: CommonProperties.geom.cardRadius

            implicitWidth: 2*root.margins +
                           Math.max(contentItemWrapper.implicitWidth,
                                    titleLabelBack.implicitWidth,
                                    cfgCancelBtn.implicitWidth +
                                    cfgOkBtn.implicitWidth +
                                    root.margins
                                    )

            implicitHeight: 4*root.margins +
                            titleLabelBack.implicitHeight +
                            contentItemWrapper.implicitHeight +
                            cfgCancelBtn.implicitHeight

            Label {
                id: titleLabelBack
                text: root.name
                font.pixelSize: CommonProperties.font.h3
                anchors {
                    top: parent.top
                    left: parent.left
                    margins: root.margins
                }
            }

            Item {
                id: backItemWrapper
                implicitWidth: children[0].implicitWidth
                implicitHeight: children[0].implicitHeight
                width: parent.width - 2*root.margins
                height: parent.height - 4*root.margins - titleLabelBack.height - cfgOkBtn.height
                clip: true
                anchors {
                    top: titleLabelBack.bottom
                    left: parent.left
                    margins: root.margins
                }
            }

            Button {
                id: cfgOkBtn
                anchors {
                    top: backItemWrapper.bottom
                    left: parent.left
                    margins: root.margins
                }

                text: 'Ok'
                onReleased: {
                    root.flipped = false
                    root.applyConfiguration()
                }
            }

            Button {
                id: cfgCancelBtn
                anchors {
                    top: backItemWrapper.bottom
                    left: cfgOkBtn.right
                    margins: root.margins
                }
                text: 'Cancel'
                onReleased: {
                    root.flipped = false
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
