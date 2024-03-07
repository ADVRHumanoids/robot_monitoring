import QtQuick
import QtQuick.Layouts
import QtQuick.Controls

import "BarPlot.js" as Logic
import "/qt/qml/Main/sharedData.js" as SharedData

Item {

    property list<string> fieldNames

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
                loader.item.setStatus(idx, ok)
            }
        }
    }

    function addAuxType(aux) {
        if(fieldNames.indexOf(aux) < 0) {

            console.log(`new aux type ${aux}`)

            fieldNames.push(aux)
            fieldNamesChanged()

            let model = container.model

            model.push(
                        {
                            'fieldName': aux,
                            'refName': '',
                            'min': Array(SharedData.jointNames.length).fill(-50),
                            'max': Array(SharedData.jointNames.length).fill(50),
                        })

            container.model = model
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
            model: fieldNames.length

            Loader {

                active: index === stack.currentIndex
                sourceComponent: barPlotComponent

                onLoaded: {
                    active = true
                    item.jointNames = SharedData.jointNames
                    item.min = modelData.min
                    item.max = modelData.max
                    item.fieldName = modelData.fieldName
                    item.fieldNameRef = modelData.refName
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
        container.model = Logic.barPlotDefaultModel
        for(let item of container.model) {
            root.fieldNames.push(Logic.getLongName(item.fieldName))
        }
    }

}
