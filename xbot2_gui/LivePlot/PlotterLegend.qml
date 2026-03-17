import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtGraphs

Control {

    property GraphsView chart

    id: root

    property var seriesLastValues: Object()

    Timer {
        running: true
        repeat: true
        interval: 666
        onTriggered: {
            seriesLastValuesChanged()
        }
    }

    signal hideSeries(string name, bool hide)
    signal highlightSeries(string name, bool highlight)
    signal removeSeriesRequested(string name)

    function addSeries(series, seriesColor) {
        seriesLastValues[series.name] = Number.NaN
        legendModel.append({
                               seriesName: series.name,
                               seriesColor: String(seriesColor)
                           })
    }

    function removeSeries(series) {
        let idxToRemove = -1
        for(let i = 0; i < legendModel.count; i++) {
            if(legendModel.get(i).seriesName === series.name) {
                idxToRemove = i
                break
            }
        }
        legendModel.remove(idxToRemove, 1)
    }

    function updateLastValue(seriesName, value) {
        seriesLastValues[seriesName] = value
    }

    onHideSeries: function(seriesName, hidden) {
        chart.series(seriesName).visible = !hidden
    }

    onHighlightSeries: function(seriesName, highlighted) {
        chart.series(seriesName).width = 2 * (highlighted ? 2 : 1)
    }

    onRemoveSeriesRequested: function(seriesName) {
        let s = chart.series(seriesName)
        removeSeries(s)
        chart.removeSeries(s)
    }

    property int maxItemWidth: 0

    contentItem: GridLayout {
        id: grid
        rowSpacing: 5
        columnSpacing: 10
        columns: Math.max(1, root.width/maxItemWidth)
        Repeater {
            model: legendModel
            delegate: legendDelegate
        }
    }

    ListModel {
        id: legendModel
    }

    Component {

        id: legendDelegate

        Item {

            width: row.width
            height: row.height

            property bool highlighed: mouseArea.containsMouse
            property bool hidden: false

            onHiddenChanged: {
                hideSeries(seriesName, hidden)
            }

            onHighlighedChanged: {
                highlightSeries(seriesName, highlighed)
            }

            MouseArea {
                id: mouseArea
                anchors.fill: parent
                hoverEnabled: true

                onClicked: {
                    hidden = !hidden
                }

                onDoubleClicked: {
                    removeSeriesRequested(seriesName)
                }
            }

            Row {

                id: row
                spacing: 5

                Rectangle {
                    id: markerRect
                    height: nameText.height/2
                    radius: height/2
                    width: height
                    color: Qt.color(seriesColor)
                    anchors.verticalCenter: parent.verticalCenter
                }

                Label {
                    id: nameText
                    text: `${seriesName} (${root.seriesLastValues[seriesName].toFixed(2)})`
                    font.weight: Qt.application.font.weight * (highlighed && !hidden ? 2 : 1)
                    font.strikeout: hidden
                }

                Component.onCompleted: {
                    root.maxItemWidth = Math.max(root.maxItemWidth, row.implicitWidth)
                }
            }


        }


    }
}
