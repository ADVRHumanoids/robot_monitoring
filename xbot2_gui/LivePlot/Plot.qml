import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Main
import Common
import LivePlot
import Font

import "../Main/sharedData.js" as SharedData

Item {

    id: root
    property ClientEndpoint client
    property var jointStateNumericFields: Array()
    property var jointStateCallbacks: Array()
    property var genericCallbacks: Array()
    property real t0: -1.0
    property Plotter activePlot

    function addSingleSeries(item) {

        let seriesName = `${item.src}/${item.name}` + (item.idx >= 0 ? `[${item.idx}]` : '')

        console.log(`will add series ${seriesName}...`)

        if(item.src === 'jointState' && item.idx >= 0) {
            let jName = SharedData.jointNames[item.idx]
            seriesName = `${jName}/${item.name}`
        }

        let series = activePlot.addSeries(seriesName, {}, false)

        let recursiveObjectFields = item.name.split('.')

        let _thisPlot = activePlot

        let cb = function(msg) {


            if(msg?.type !== undefined && msg.type !== item.src) {
                // console.log(`${msg.type} != ${item.src}`)
                // msg not intended for this callback
                return true
            }

            let time = appData.getTimeNs()

            let value = msg
            for(let field of recursiveObjectFields) {
                value = value[field]
            }

            if(item.idx >= 0) {
                value = value[item.idx]
            }

            if(_thisPlot === null) {
                console.log('parent plot died')
                console.log(`removing callback for series ${seriesName}`)
                return false
            }

            try {
                _thisPlot.addPoint(series, (time - t0)*1e-9, value)
            }
            catch(err) {
                console.error(`error adding point to series ${seriesName}: ${err}`)
                console.log(`removing callback for series ${seriesName}`)
                return false
            }

            return true
        }

        if(item.src === 'jointState') {
            jointStateCallbacks.push(cb)
            console.log('..done')
        }
        else {
            genericCallbacks.push(cb)
        }
    }

    function addSeries() {
        for(let i = 0; i < listModel.count; i++) {

            let item = listModel.get(i)

            if(!item.checked) {
                continue
            }

            addSingleSeries(item)
        }
    }





    function discoverJointStateNumericFields(msg) {
        let fields = Object.keys(msg)

        for (let field of fields) {

            let ftype = typeof msg[field];

            // console.log(`jointState.${field} (${ftype}) (${Array.isArray(msg[field])})`)

            if(ftype === 'number') {
                console.log(`jointState.${field}`)
                jointStateNumericFields.push(
                            {
                                src: 'jointState',
                                name: field,
                                type: 'number',
                                length: 1
                            })
            }

            if(ftype === 'object' &&
                    typeof msg[field][0] === 'number') {
                // console.log(`jointState.${field}[${msg[field].length}]`)
                jointStateNumericFields.push(
                            {
                                src: 'jointState',
                                name: field,
                                type: 'array',
                                length: msg[field].length
                            })
            }

        }

        jointStateNumericFieldsChanged()
    }



    ListModel {

        id: listModel

        function build() {

            for(let [k, v] of Object.entries(client.objNumericFields)) {
                for(let v1 of v) {
                    console.log(JSON.stringify(v1))
                    if(v1.type === 'array') {
                        for(let i = 0; i < v1.length; i++) {
                            append({
                                       src: v1.src,
                                       name: v1.name,
                                       idx: i,
                                       checked: false
                                   })
                        }
                    }
                    else {
                        append({
                                   src: v1.src,
                                   name: v1.name,
                                   idx: -1,
                                   checked: false
                               })
                    }
                }

            }

            for(let v1 of jointStateNumericFields) {
                console.log(JSON.stringify(v1))
                if(v1.type === 'array') {
                    for(let j = 0; j < v1.length; j++) {
                        append({
                                   src: v1.src,
                                   name: v1.name,
                                   idx: j,
                                   checked: false
                               })
                    }
                }
                else {
                    append({
                               src: v1.src,
                               name: v1.name,
                               idx: -1,
                               checked: false
                           })
                }
            }
        }
    }

    Drawer {

        id: drawer

        height: parent.height

        padding: 16
        leftPadding: 16
        rightPadding: 16

        ColumnLayout {

            anchors.fill: parent

            spacing: 16

            TextField {
                id: filterField
                Layout.fillWidth: true
                placeholderText: 'Filter'
            }

            ListView {

                Layout.fillHeight: true
                Layout.fillWidth: true

                model: listModel

                delegate:  Label {
                    required property bool checked
                    required property int index
                    required property int idx
                    required property string src
                    required property string name
                    property string idxText: src === 'jointState' ? SharedData.jointNames[idx] : `${idx}`
                    id: label
                    text: `${src}/${name}` + (idx >= 0 ? `[${idxText}]` : '')
                    color: checked ? palette.accent : palette.text
                    font.bold: checked || mouse.containsMouse
                    visible: filterField.text === '' ||
                             text.toLowerCase().includes(filterField.text.toLowerCase())
                    height: visible ? implicitHeight : 0
                    topPadding: 1
                    bottomPadding: 1

                    MouseArea {
                        id: mouse
                        anchors.fill: parent
                        onClicked: listModel.get(index).checked = !listModel.get(index).checked
                        hoverEnabled: true
                    }
                }

            }

            Button {
                Layout.fillWidth: true
                text: 'Add series'
                onClicked: root.addSeries()
            }


            SpinBox {
                Layout.fillWidth: true
                from: 1
                to: 1000
                value: activePlot.timeSpan
                onValueModified: activePlot.timeSpan = value
                editable: true
            }

        }

        onAboutToShow: {
            for(let i = 0; i < listModel.count; i++) {
                listModel.get(i).checked = false
            }
        }
    }



    RecursiveSplitView {

        anchors.fill: parent

        delegate: Plotter {
            id: plot
            signal splitHorizontal()
            signal splitVertical()
            signal closeSplit()
            plotterLegend: legend
            PlotterLegend {
                id: legend
                chart: plot.chartView
            }
            GridLayout {
                anchors.right: plot.right
                rows: 1
                ToolButton {
                    text: MaterialSymbolNames.splitHorz
                    font.family: 'Material Symbols Outlined'
                    // font.variableAxes: {'opsz': 48}
                    font.pixelSize: 24
                    onClicked: plot.splitHorizontal()
                }
                ToolButton {
                    text: MaterialSymbolNames.splitVert
                    font.family: 'Material Symbols Outlined'
                    // font.variableAxes: {'opsz': 48}
                    font.pixelSize: 24
                    onClicked: plot.splitVertical()
                }
                ToolButton {
                    text: MaterialSymbolNames.home
                    font.family: 'Material Symbols Outlined'
                    // font.variableAxes: {'opsz': 48}
                    font.pixelSize: 24
                    onClicked: plot.resetView()
                }
                ToolButton {
                    text: MaterialSymbolNames.clean
                    font.family: 'Material Symbols Outlined'
                    // font.variableAxes: {'opsz': 48}
                    font.pixelSize: 24
                    onClicked: plot.clearPoints()
                }
                ToolButton {
                    text: MaterialSymbolNames.close
                    font.family: 'Material Symbols Outlined'
                    // font.variableAxes: {'opsz': 48}
                    font.pixelSize: 24
                    onClicked: plot.closeSplit()
                }
                ToolButton {
                    text: MaterialSymbolNames.more
                    font.family: 'Material Symbols Outlined'
                    // font.variableAxes: {'opsz': 48}
                    font.pixelSize: 24
                    onClicked: {
                        listModel.build()
                        activePlot = plot
                        drawer.open()
                    }
                }
            }

            Component.onCompleted: root.activePlot = plot
        }
    }

    Connections {

        id: jsConn

        target: client

        function onJointStateReceived(msg) {

            if(SharedData.jointNames.length === 0) {
                return
            }

            if(jointStateNumericFields.length === 0) {
                discoverJointStateNumericFields(msg)
            }

            root.jointStateCallbacks = root.jointStateCallbacks.filter(cb => cb(msg))

        }

        function onObjectReceived(msg) {

            root.genericCallbacks = root.genericCallbacks.filter(cb => cb(msg))

        }
    }

    Connections {

        target: CommonProperties.plot

        function onAddJointStateSeriesRequested(jName, jField) {
            let seriesDescription = {
                'src': 'jointState',
                'name': jField,
                'idx': SharedData.jointNames.indexOf(jName),
                'checked': true
            }
            addSingleSeries(seriesDescription)
        }

    }

    Component.onCompleted: {
        root.t0 = appData.getTimeNs()
    }


}

