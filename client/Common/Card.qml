import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import xbot2_gui.Common


Item {

    id: root

    // public

    property string name: 'CardName'

    property int margins: CommonProperties.geom.spacing

    property color backgroundColor: defaultBackground

    readonly property color defaultBackground: CommonProperties.colors.cardBackground

    property alias nameFont: titleLabel.font

    property int defaultHeight: -1

    property bool flipped: false

    property bool hidden: false

    property bool configurable: true

    property Item frontItem: Item {

    }

    property Item backItem: Item {

    }

    property list<Item> toolButtons

    signal applyConfiguration()


    // private

    property bool completed: false

    property int _hiddenHeight: 2*margins + frontSideRoot.implicitBannerHeight

    Component.onCompleted: {
        frontItem.parent = flip.frontSide.contentItemWrapper
        backItem.parent = flip.backSide.contentItemWrapper
        console.log(`card ${name} impl size is ${frontItem.implicitWidth} x ${frontItem.implicitHeight}`)
    }

    implicitWidth: flip.implicitWidth
    implicitHeight: flip.implicitHeight

    Flipable {

        id: flip

        implicitWidth: root.flipped ? backSide.implicitWidth : frontSide.implicitWidth
        implicitHeight: root.flipped ? backSide.implicitHeight : frontSide.implicitHeight

        width: parent.width
        height: parent.height

        property Item frontSide: Rectangle {

            property alias contentItemWrapper: frontItemWrapper

            id: frontSideRoot
            width: flip.width
            height: flip.height
            color: root.backgroundColor
            radius: CommonProperties.geom.cardRadius

            property int hiddenImplicitHeight: 2*root.margins +
                                             implicitBannerHeight

            property int defaultImplicitHeight: hiddenImplicitHeight +
                                               root.margins +
                                               contentItemWrapper.implicitHeight

            implicitWidth: 2*root.margins +
                           Math.max(contentItemWrapper.implicitWidth,
                                    titleLabel.implicitWidth +
                                    toolBtnRow.implicitWidth +
                                    root.margins)

            implicitHeight: root.hidden ? hiddenImplicitHeight : defaultImplicitHeight


            property int implicitBannerHeight: Math.max(titleLabel.implicitHeight,
                                                        toolBtnRow.implicitHeight - 16)

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

            Label {
                id: titleLabel
                text: root.name
                font.pixelSize: CommonProperties.font.h2
                height: frontSideRoot.implicitBannerHeight
                verticalAlignment: Text.AlignVCenter
                anchors {
                    top: parent.top
                    left: parent.left
                    margins: root.margins
                }
            }

            Row {

                id: toolBtnRow

                anchors {
                    right: parent.right
                    margins: root.margins
                    verticalCenter: titleLabel.verticalCenter
                }

                spacing: root.margins

                layoutDirection: Qt.RightToLeft

                SmallToolButton {
                    id: configureBtn

                    visible: root.configurable

                    text: '\uf013'
                    font.family: CommonProperties.fontAwesome.solid.family
                    font.pixelSize: CommonProperties.font.h3

                    onClicked: {
                        root.flipped = true
                        root.hidden = false
                    }
                }

                SmallToolButton {
                    id: showHideBtn

                    text: root.hidden ? '\uf077' : '\uf078'
                    font.family: CommonProperties.fontAwesome.solid.family
                    font.pixelSize: CommonProperties.font.h3

                    onClicked: {
                        root.hidden = !root.hidden
                    }
                }

            }

            Item {
                id: frontItemWrapper
                implicitWidth: children[0].implicitWidth > 0 ? children[0].implicitWidth : childrenRect.width
                implicitHeight: children[0].implicitHeight > 0 ? children[0].implicitHeight : childrenRect.height
                width: parent.width - 2*root.margins
                height: parent.height - 3*root.margins - titleLabel.height

                clip: true

                anchors {
                    top: titleLabel.bottom
                    left: parent.left
                    margins: root.margins
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
