import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import Common
import Main

import "Sanding.js" as Logic

Item {

    id: root

    property ClientEndpoint client

    signal pageSelected()

    property list<real> heightBreakpoints: [0.0, 1.0, 2.0, 2.4]

    property list<real> widthBreakpoints: [-1, -0.5, 0.5, 1.0]

    property int selectedIndex: -1

    property var robotToBreakpoints



    ColumnLayout {

        // enabled: root.selectedIndex >= 0

        id: configPanel

        anchors.verticalCenter: root.verticalCenter
        anchors.left: parent.left

        spacing: 8



        Label {
            text: 'Parameters'
        }

        Item {

            Layout.fillWidth: true
            implicitHeight: paramFrame.implicitHeight
            implicitWidth: paramFrame.implicitWidth

            Label {

                z: 10

                id: msgLabel

                visible: false

                anchors.centerIn: paramFrame
                width: paramFrame.width - 16

                anchors.bottomMargin: 20

                property bool selectPatch: !configPanel.enabled
                property bool adjustPatch: configPanel.enabled && !enableSanding && !sandingInProgress
                property bool enableSanding: statusText.text === 'Waiting'
                property bool sandingInProgress: statusText.text === 'Sanding'

                text: 'Start the sanding tool and confirm when done!'

                color: 'red'

                wrapMode: Label.WordWrap

                font.pixelSize: CommonProperties.font.h3

                Timer {
                    running: parent.enableSanding
                    interval: 666
                    repeat: true

                    onTriggered: {
                        parent.visible = !parent.visible
                    }

                    onRunningChanged: parent.visible = running
                }

                background: Rectangle {
                    color: Qt.alpha('red', 0.3)
                    radius: 4
                }

                topPadding: 4
                bottomPadding: 4
                leftPadding: 4
                rightPadding: 4

            }

            Frame {

                id: paramFrame

                anchors.fill: parent

                opacity: msgLabel.enableSanding ? 0.1 : 1.0
                enabled: !msgLabel.enableSanding

                contentItem: GridLayout {


                    columns: 2
                    columnSpacing: 12
                    rowSpacing: 8

                    Label {
                        text: 'robot type'
                    }

                    ComboBox {
                        id: robotTypeCombo
                        Layout.fillWidth: true
                        onCurrentTextChanged: {
                            // change breakpoints
                            root.heightBreakpoints = robotToBreakpoints[currentText]
                            fillRectTimer.start()
                        }

                        Timer {
                            id: fillRectTimer
                            repeat: false
                            interval: 1
                            onTriggered: {
                                for(let i = 0; i < colRepeater.count; i++) {

                                    colRepeater.itemAt(i).resizableRect.fill()
                                }
                            }
                        }
                    }

                    Item {
                        Layout.columnSpan: 2
                        Layout.preferredHeight: 4
                    }

                    Label {
                        text: 'max height'
                    }

                    DoubleSpinBox {
                        Layout.fillWidth: true
                        value: colRepeater.itemAt(root.selectedIndex).patchMaxY || 0
                        from: heightBreakpoints[0]
                        to: heightBreakpoints[heightBreakpoints.length-1]
                        decimals: 2

                        onValueModified: function(value) {
                            let patch = colRepeater.itemAt(root.selectedIndex)
                            patch.setPatchDimensions(
                                        patch.patchMinX, patch.patchMaxX, patch.patchMinY, value
                                        )
                        }
                    }

                    Label {
                        text: 'min height'
                    }

                    DoubleSpinBox {
                        Layout.fillWidth: true
                        value: colRepeater.itemAt(root.selectedIndex).patchMinY || 0
                        from: heightBreakpoints[0]
                        to: heightBreakpoints[heightBreakpoints.length-1]
                        decimals: 2

                        onValueModified: function(value) {
                            let patch = colRepeater.itemAt(root.selectedIndex)
                            patch.setPatchDimensions(
                                        patch.patchMinX, patch.patchMaxX, value, patch.patchMaxY
                                        )
                        }
                    }
                    Label {
                        text: 'min width'
                    }

                    DoubleSpinBox {
                        Layout.fillWidth: true
                        value: colRepeater.itemAt(root.selectedIndex).patchMinX || 0
                        from: widthBreakpoints[0]
                        to: widthBreakpoints[widthBreakpoints.length-1]
                        decimals: 2

                        onValueModified: function(value) {
                            let patch = colRepeater.itemAt(root.selectedIndex)
                            patch.setPatchDimensions(
                                        value, patch.patchMaxX, patch.patchMinY, patch.patchMaxY
                                        )
                        }
                    }

                    Label {
                        text: 'max width'
                    }

                    DoubleSpinBox {
                        Layout.fillWidth: true
                        value: colRepeater.itemAt(root.selectedIndex).patchMaxX || 0
                        from: widthBreakpoints[0]
                        to: widthBreakpoints[widthBreakpoints.length-1]
                        decimals: 2

                        onValueModified: function(value) {
                            let patch = colRepeater.itemAt(root.selectedIndex)
                            patch.setPatchDimensions(
                                        patch.patchMinX, value, patch.patchMinY, patch.patchMaxY
                                        )
                        }
                    }

                    Item {
                        Layout.columnSpan: 2
                        Layout.preferredHeight: 4
                    }

                    Label {
                        text: 'force (N)'
                    }

                    SpinBox {
                        Layout.fillWidth: true
                        id: maxForceSpin
                        from: 0
                        to: 100
                        value: 30
                        stepSize: 10
                    }

                    Label {
                        text: 'speed (cm/s)'
                    }

                    DoubleSpinBox {
                        Layout.fillWidth: true
                        id: speedSpin
                        from: 0
                        to: 5
                        value: 2.0
                        stepSize: 0.1
                    }
                }
            }
        }


        Item {
            height: 8
            width: parent.width
        }

        Label {
            text: 'Execution status'
        }

        Frame {

            Layout.fillWidth: true


            contentItem: GridLayout {

                columns: 2
                columnSpacing: 12
                rowSpacing: 8

                RowLayout {

                    Layout.columnSpan: 2
                    Layout.fillWidth: true
                    spacing: 8

                    Button {

                        enabled: root.selectedIndex >= 0

                        Layout.fillWidth: true
                        text: 'Start'

                        onClicked: {
                            Logic.start()
                        }
                    }

                    Button {

                        Layout.fillWidth: true
                        text: 'Kill'

                        onClicked: {
                            Logic.kill()
                        }
                    }

                }

                Button {
                    enabled: statusText.text === 'Waiting'
                    Layout.columnSpan: 2
                    Layout.fillWidth: true
                    text: 'Sanding Tool Started'

                    onClicked: {
                        Logic.sandingToolStartedAck()
                    }
                }


                Label {
                    text: 'Status'
                }

                TextArea {
                    id: statusText
                    Layout.fillWidth: true
                    Layout.maximumWidth: progressBar.width
                    readOnly: true
                    text: '--'
                }


                Label {
                    text: 'Progress'
                }

                ProgressBar {
                    id: progressBar
                    Layout.fillWidth: true
                    enabled: statusText.text !== '--'
                    from: 0
                    to: 100
                }
            }
        }

    }

    GridLayout {

        id: patchLayout

        anchors.left: configPanel.right
        anchors.right: root.right
        anchors.margins: CommonProperties.geom.margins
        anchors.top: root.top
        anchors.bottom: root.bottom

        columnSpacing: 16
        rowSpacing: 16
        rows: heightBreakpoints.length - 1
        columns: widthBreakpoints.length - 1

        Repeater {

            id: colRepeater
            model: (heightBreakpoints.length - 1)*(widthBreakpoints.length - 1)

            Rectangle {

                required property int index
                property alias checked: resizableRect.checked

                property alias resizableRect: resizableRect

                property int xPatchIdx: index % patchLayout.columns //Math.floor(index / patchLayout.rows)
                property int yPatchIdx: patchLayout.rows - Math.floor(index / patchLayout.columns) - 1//patchLayout.rows - index % patchLayout.rows - 1

                property string patchType: xPatchIdx === 0 ? 'left' : (xPatchIdx === patchLayout.columns - 1 ? 'right' : 'center')

                property real yBrkLo: root.heightBreakpoints[yPatchIdx]
                property real yBrkHi: root.heightBreakpoints[yPatchIdx+1]
                property real xBrkLo: root.widthBreakpoints[xPatchIdx]
                property real xBrkHi: root.widthBreakpoints[xPatchIdx+1]
                property real xScaling: (xBrkHi - xBrkLo)/resizableRect.width
                property real yScaling: (yBrkHi - yBrkLo)/resizableRect.height

                // pixel -> length conversions
                property real patchMinX: xBrkLo + resizableRect.rectX*xScaling
                property real patchMaxX: patchMinX + patchWidth
                property real patchMinY: patchMaxY - patchHeight
                property real patchMaxY: yBrkHi - resizableRect.rectY*yScaling
                property real patchHeight: resizableRect.rectHeight*yScaling
                property real patchWidth: resizableRect.rectWidth*xScaling

                Layout.fillHeight: true
                Layout.fillWidth: true
                Layout.preferredHeight: yBrkHi - yBrkLo
                Layout.preferredWidth: xBrkHi - xBrkLo

                function setPatchDimensions(xmin, xmax, ymin, ymax) {
                    let x = xmin
                    let y = ymax
                    let width = xmax - xmin
                    let height = ymax - ymin
                    resizableRect.rectX = (x - xBrkLo)/xScaling
                    resizableRect.rectY = (yBrkHi - y)/yScaling
                    resizableRect.rectWidth = width/xScaling
                    resizableRect.rectHeight = height/yScaling
                }

                color: Qt.rgba(1, 1, 1, checked ? 0.9 : 0.8)
                border.width: 1

                ResizableRectangle {

                    id: resizableRect
                    property bool checked: false
                    showHandle: checked
                    color: checked ? 'green' : CommonProperties.colors.primary
                    handleColor: checked ? Qt.alpha(CommonProperties.colors.accent, 0.9) : Qt.alpha('grey', 0.9)
                    anchors.fill: parent

                    Label {
                        text: `${patchWidth.toFixed(2)} x ${patchHeight.toFixed(2)} m`
                        anchors.centerIn: parent.innerRect
                        horizontalAlignment: Text.AlignHCenter
                    }

                    onClicked: {

                        checked = !checked

                        if(!checked) {
                            root.selectedIndex = -1
                            return
                        }

                        root.selectedIndex = parent.index

                        for(let i = 0; i < colRepeater.count; i++) {

                            if(i === parent.index) {
                                continue
                            }

                            colRepeater.itemAt(i).checked = false
                        }
                    }
                }

            }
        }

        Component.onCompleted: {
            for(let i = 0; i < colRepeater.count; i++) {

                colRepeater.itemAt(i).resizableRect.fill()
            }
        }
    }


    Connections {

        target: client

        onObjectReceived: function(msg) {

            if(msg.type === 'concert_sanding_progress') {
                statusText.text = msg.status
                progressBar.value = msg.progress
                progressBar.indeterminate = msg.progress < 0

                if(msg.progress === 100) {
                    statusText.text = 'Completed'
                }
            }
        }

    }

    Component.onCompleted: Logic.construct()

    onPageSelected: Logic.construct()
}
