import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Common


Item {


    // public

    property string name: 'CardName'

    property int verticalMargins: 4

    property int margins: CommonProperties.geom.spacing

    property color backgroundColor: defaultBackground

    property color borderColor

    property int borderWidth: 0

    readonly property color defaultBackground: CommonProperties.colors.cardBackground

    property alias nameFont: titleLabel.font

    property int maxContentHeight: 100000

    property int bannerHeight: -1

    // property alias availableContentWidth: frontScroll.availableWidth

    // property alias availableContentHeight: frontScroll.availableHeight

    property bool flipped: false

    property bool collapsable: true

    property bool collapsed: false

    property bool configurable: true

    property Item frontItem: Item {

    }

    property Item backItem: Item {

    }

    property Item badgeItem: Item {

    }

    property list<Item> toolButtons

    property alias statusIcon: statusIcon

    signal applyConfiguration()

    signal titleClicked()


    // private
    id: root

    Component.onCompleted: {
        frontItem.parent = flip.front.contentItemWrapper
        backItem.parent = flip.back.contentItemWrapper
        badgeControl.contentItem = badgeItem
    }

    implicitWidth: flip.implicitWidth
    implicitHeight: flip.implicitHeight

    // height: flip.height
    clip: true

    property bool _collapsed_before_flip: true

    Behavior on implicitHeight {
        NumberAnimation {
            duration: 333
            easing.type: Easing.OutQuad
        }
    }

    Flipable {

        id: flip

        implicitWidth: root.flipped ? back.implicitWidth : front.implicitWidth
        implicitHeight: root.flipped ? back.implicitHeight : front.implicitHeight

        width: parent.width
        height: parent.height

        // front side is rendered as a rectangle whose content is layed out in a column
        // with header (card title and tool buttons) and content (item)

        front: Control {

            property alias contentItemWrapper: frontContentWrapper

            id: frontSideRoot
            width: flip.width
            height: flip.height
            // padding: root.margins
            leftPadding: root.margins
            rightPadding: root.margins
            bottomPadding: root.margins * !root.collapsed

            background: Rectangle {
                color: root.backgroundColor
                radius: CommonProperties.geom.cardRadius
                border.color: root.borderColor
                border.width: root.borderWidth
            }

            // implicitHeight: frontColumn.implicitHeight
            // implicitWidth: frontColumn.implicitWidth

            Component.onCompleted: {
                for(let i = 0; i < root.toolButtons.length; i++) {
                    let tb = root.toolButtons[i]
                    tb.parent = toolBtnRowInner
                }
            }

            // column holding banner (title + toolbuttons) and content
            contentItem: ColumnLayout {

                id: frontColumn
                // width: parent.width
                // spacing: root.margins

                // row with tool buttons
                RowLayout {

                    id: toolBtnRow

                    // Layout.preferredHeight: root.bannerHeight > 0 ? root.bannerHeight : implicitHeight

                    Layout.fillWidth: true

                    spacing: 0

                    // badge
                    Control {
                        id: badgeControl
                        rightPadding: contentItem.implicitWidth > 0 ? 6 : 0
                    }

                    // banner
                    Label {
                        Layout.fillWidth: true

                        id: titleLabel
                        text: root.name
                        font.pixelSize: CommonProperties.font.h2
                        verticalAlignment: Text.AlignVCenter
                        wrapMode: Text.Wrap
                        MouseArea {
                            id: mouse
                            enabled: root.collapsable
                            anchors.fill: parent
                            onDoubleClicked: root.collapsed = !root.collapsed
                            onClicked: root.bannerClicked()
                        }
                    }

                    Label {
                        id: statusIcon
                        visible: text !== ''
                        verticalAlignment: Text.AlignVCenter
                        leftPadding: 4
                    }

                    // Item {
                    //     Layout.preferredWidth: 1
                    //     Layout.minimumWidth: 6
                    //     Layout.fillWidth: true
                    // }

                    Control {
                        padding: root.verticalMargins
                        Layout.alignment: Qt.AlignVCenter
                        contentItem: RowLayout {
                            id: toolBtnRowInner
                        }
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
                            _collapsed_before_flip = root.collapsed
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

                    Layout.fillHeight: true
                    Layout.fillWidth: true

                    // anchors {
                    //     left: parent.left
                    //     right: parent.right
                    //     margins: root.margins
                    // }

                    Layout.preferredHeight: root.collapsed ? 0 : implicitHeight

                    implicitHeight: frontContentWrapper.implicitHeight

                    clip: true

                    Item {
                        id: frontContentWrapper
                        anchors.fill: parent
                        implicitHeight: children.length > 0 ? children[0].implicitHeight : 0
                    }

                }
            }
        }

        back: Control {

            property alias contentItemWrapper: backItemWrapper

            width: flip.width
            height: flip.height

            background: Rectangle {
                color: root.backgroundColor
                radius: CommonProperties.geom.cardRadius
            }

            padding: root.margins

            contentItem: ColumnLayout {

                id: backColumn

                // width: parent.width

                RowLayout {

                    id: backHeaderRow
                    Layout.fillWidth: true

                    Label {
                        id: titleLabelBack
                        text: root.name
                        font: titleLabel.font
                        Layout.fillWidth: true
                    }
                }

                Item {
                    width: parent.width
                    height: root.margins
                }

                Item {
                    Layout.fillHeight: true
                    Layout.fillWidth: true
                    id: backItemWrapper
                    implicitWidth: children[0].implicitWidth
                    implicitHeight: children[0].implicitHeight
                    // width: parent.width
                    // height:  children.length > 0 ? children[0].height : 0
                    clip: true
                }

                RowLayout {

                    spacing: root.margins

                    Button {
                        id: cfgOkBtn
                        text: 'Ok'
                        Layout.fillWidth: true
                        onReleased: {
                            root.flipped = false
                            root.collapsed = root._collapsed_before_flip
                            root.applyConfiguration()
                        }
                    }

                    Button {
                        id: cfgCancelBtn
                        text: 'Cancel'
                        Layout.fillWidth: true
                        onReleased: {
                            root.flipped = false
                            root.collapsed = root._collapsed_before_flip
                        }
                    }

                }

            }

        }

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
