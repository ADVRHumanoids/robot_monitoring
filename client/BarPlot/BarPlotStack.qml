import QtQuick 2.4
import QtQuick.Layouts
import QtQuick.Controls
import "logic.js" as Logic
import "../sharedData.js" as SharedData

Item {

    property var fieldNames: Logic.barPlotFields.map(f => Logic.shortToLongName[f])

    property alias currentIndex: stack.currentIndex

    function setJointStateMessage(js_msg) {
        container.itemAt(stack.currentIndex).item.setJointStateMessage(js_msg)
    }

    signal jointClicked(string jointName)

    function setStatus(jName, ok) {
        let idx = SharedData.jointNames.indexOf(jName)
        statusOk[idx] = ok

        for(let i = 0; i < container.count; i++) {
            let loader = container.itemAt(i)
            if(loader.active) {
                loader.item.setStatus(statusOk)
            }
        }
    }


    // private

    id: root
    implicitWidth: stack.implicitWidth
    implicitHeight: stack.implicitHeight

    property list<bool> statusOk

    StackLayout {

        id: stack
        anchors.fill: parent

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
                    item.setStatus(root.statusOk)
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

    Component.onCompleted: {
        statusOk = Array(SharedData.jointNames.length)
        statusOk.fill(true)
    }

}
