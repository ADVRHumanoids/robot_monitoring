import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Common
import Main

import "Parameters.js" as Logic

Item {

    id: root

    property list<ParameterView> paramViews: [rootView]

    property ClientEndpoint client

    property Component paramViewDelegate: ParameterView {}

    property Component boolDelegate: BoolDelegate {}

    property Component intDelegate: IntDelegate {}

    property Component vectorDelegate: VectorDelegate {}

    property Component doubleDelegate: RealDelegate {}

    property Component discreteValuesDelegate: DiscreteValuesDelegate {}

    property Component wrapperItemDelegate: Control {

        property alias name: header.text
        property alias loader: loader
        property bool busy: false

        function set() {
            control.busy = !autoSwitch.checked
            Logic.setParams(control.name, loader.item.value)
            .then((res) => control.busy = false)
        }

        id: control
        leftPadding: 10
        rightPadding: 10
        topPadding: 8
        bottomPadding: 8

        contentItem: Column {

            spacing: 8

            add: Transition {
                NumberAnimation{
                    property: "opacity"
                    from: 0
                    to: 1
                }
            }

            SectionHeader {
                id: header
                pixelSize: CommonProperties.font.h3
                onClicked: loaderWrapper.visible = !loaderWrapper.visible
                width: parent.width
                // Layout.fillWidth: true

                Button {
                    text: 'Set'
                    enabled: !control.busy
                    onClicked: control.set()
                }

                Switch {
                    text: 'Auto'
                    enabled: !control.busy
                    checked: false
                    id: autoSwitch
                }
            }

            Item {

                id: loaderWrapper
                width: parent.width
                height: loader.height

                Loader {
                    id: loader
                    // Layout.fillWidth: true
                    width: parent.width
                    opacity: control.busy ? 0.1 : 1
                }

                Connections {
                    target: loader.item
                    ignoreUnknownSignals: false
                    function onValueChangedByUser(value) {
                        if(autoSwitch.checked) {
                            control.set()
                        }
                    }
                }

                BusyIndicator {
                    running: control.busy
                    anchors.centerIn: loader
                    z: 1
                }

            }

        }

        background: Rectangle {
            radius: 4
            color: Qt.rgba(1, 1, 1, 0.1)
        }
    }

    ScrollView {

        id: scroll
        anchors.fill: parent
        contentWidth: availableWidth

        Column {

            spacing: 16
            width: scroll.availableWidth

            RowLayout{
                width: parent.width
                Label {
                    text: 'Parameter Tuning'
                    font.pixelSize: CommonProperties.font.h1
                    Layout.fillWidth: true
                }

                Button {
                    Layout.fillWidth: false
                    text: 'Refresh'
                    onClicked: Logic.refresh1()
                }
            }

            ParameterView {

                id: rootView
                width: parent.width

                viewName: '/'

            }

        }

    }

    Component.onCompleted: Logic.refresh1()

}
