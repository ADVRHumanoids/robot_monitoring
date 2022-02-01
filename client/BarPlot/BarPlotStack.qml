import QtQuick 2.4
import QtQuick.Layouts
import QtQuick.Controls
import "logic.js" as Logic
import "../sharedData.js" as SharedData

Item {

    id: root
    width: 400
    height: 400

    function construct() {
        container.model = Logic.barPlotFields.length
    }

    function setJointStateMessage(js_msg) {
        container.itemAt(stack.currentIndex).setJointStateMessage(js_msg)
    }

    signal jointClicked(string jointName)

    ColumnLayout {

        anchors.fill: parent

        ComboBox {
            id: combo
            Layout.fillWidth: true
            model: Logic.barPlotFields.map(f => Logic.shortToLongName[f])
            wheelEnabled: true
        }

        StackLayout {

            id: stack
            Layout.fillWidth: true
            Layout.fillHeight: true
            currentIndex: combo.currentIndex

            Repeater {

                id: container
                model: 0

                BarPlot {

                    Layout.fillHeight: true
                    Layout.fillWidth: true

                    Component.onCompleted: {
                        jointNames = SharedData.jointNames
                        min = Logic.barPlotMin()[index]
                        max = Logic.barPlotMax()[index]
                        fieldName = Logic.barPlotFields[index]
                        fieldNameRef = Logic.refName[index]
                    }

                    onJointClicked: function(jn) {
                        root.jointClicked(jn)
                    }

                }

            }

        }
    }
}
