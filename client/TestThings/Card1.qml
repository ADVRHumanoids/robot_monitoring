import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import xbot2_gui.common


Item {

    id: root

    // public

    property string name: 'CardName'

    property int margins: CommonProperties.geom.spacing

    width: implicitWidth

    property int defaultHeight: -1

    property bool flipped: false

    property bool hidden: false

    property Item frontItem: Item {

    }

    property Item backItem: Item {

    }

    signal configurationChanged()


    // private

    property int _hiddenHeight: 2*margins + titleLabel.height

    height: hidden ? _hiddenHeight : (defaultHeight > 0 ? defaultHeight : implicitHeight)

    clip: true

    Behavior on height {
        id: heightTransition
        enabled: false
        NumberAnimation {
            duration: 500
            easing.type: Easing.OutQuad
        }
    }

    Component.onCompleted: {
        frontItem.parent = flip.frontSide.contentItemWrapper
        backItem.parent = flip.backSide.contentItemWrapper
        heightTransition.enabled = true
    }

    implicitWidth: flipped ? flip.backSide.implicitWidth : flip.frontSide.implicitWidth
    implicitHeight: flipped ? flip.backSide.implicitHeight : flip.frontSide.implicitHeight

    Flipable {

        id: flip

        anchors.fill: parent

        property Item frontSide: Rectangle {

            property alias contentItemWrapper: frontItemWrapper

            id: frontSideRoot
            width: flip.width
            height: flip.height
            color: CommonProperties.colors.cardBackground
            radius: CommonProperties.geom.cardRadius

            implicitWidth: 2*root.margins +
                           Math.max(contentItemWrapper.implicitWidth,
                                    titleLabel.implicitWidth +
                                        configureBtn.implicitWidth +
                                        showHideBtn.implicitWidth +
                                        2*root.margins)

            implicitHeight: 3*root.margins +
                            titleLabel.implicitHeight +
                            contentItemWrapper.implicitHeight

            property int implicitBannerHeight: Math.max(titleLabel.implicitHeight,
                                                configureBtn.implicitHeight,
                                                showHideBtn.implicitHeight) - 16

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

            ToolButton {
                id: configureBtn
                anchors {
                    verticalCenter: titleLabel.verticalCenter
                    right: parent.right
                    margins: root.margins
                }

                width: implicitWidth - 20

                text: '\uf013'
                font.family: CommonProperties.fontAwesome.solid.family
                font.pixelSize: titleLabel.font.pixelSize

                onClicked: root.flipped = true
            }

            ToolButton {
                id: showHideBtn
                anchors {
                    verticalCenter: titleLabel.verticalCenter
                    right: configureBtn.left
                    margins: root.margins
                }

                width: implicitWidth - 20

                text: root.hidden ? '\uf077' : '\uf078'
                font.family: CommonProperties.fontAwesome.solid.family
                font.pixelSize: titleLabel.font.pixelSize

                onClicked: {
                    root.hidden = !root.hidden
                }
            }

            Item {
                id: frontItemWrapper
                implicitWidth: childrenRect.width
                implicitHeight: childrenRect.height
                width: parent.width - 2*root.margins
                height: parent.height - 3*root.margins - titleLabel.height

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
            color: CommonProperties.colors.cardBackground
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
                font.pixelSize: CommonProperties.font.h2
                anchors {
                    top: parent.top
                    left: parent.left
                    margins: root.margins
                }
            }

            Item {
                id: backItemWrapper
                implicitWidth: childrenRect.width
                implicitHeight: childrenRect.height
                width: parent.width - 2*root.margins
                height: parent.height - 4*root.margins - titleLabelBack.height - cfgOkBtn.height
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
                    root.configurationChanged()
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
