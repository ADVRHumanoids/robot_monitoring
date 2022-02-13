import QtQuick 2.4
import QtQuick.Layouts
import QtQuick.Controls
import "logic.js" as Logic
import "../sharedData.js" as SharedData

Item {

    id: root
    width: 400
    height: 400

    function setJointStateMessage(js_msg) {
        container.itemAt(stack.currentIndex).item.setJointStateMessage(js_msg)
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

            onCurrentIndexChanged: {
                root.setJointStateMessage(SharedData.latestJointState)
            }

            Repeater {

                id: container
                model: Logic.barPlotFields.length

                Loader {

                    active: index === stack.currentIndex
                    sourceComponent: barPlotComponent

                    onLoaded: {
                        active = true
                        item.jointNames = SharedData.jointNames
                        item.min = Logic.barPlotMin()[index]
                        item.max = Logic.barPlotMax()[index]
                        item.fieldName = Logic.barPlotFields[index]
                        item.fieldNameRef = Logic.refName[index]
                        item.setJointStateMessage(SharedData.latestJointState)
                    }
                }

            }

        }

    }


    property Component barPlotComponent: Component {
        BarPlot {

            Layout.fillHeight: true
            Layout.fillWidth: true

            onJointClicked: function(jn) {
                root.jointClicked(jn)
            }

        }
    }
}
