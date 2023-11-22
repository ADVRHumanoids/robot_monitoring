import QtQuick
import QtQuick.Controls
import QtCharts

Item {

    property ChartView chart

    id: root
    implicitHeight: grid.implicitHeight
    implicitWidth: grid.implicitWidth

    signal hideSeries(string name, bool hide)
    signal highlightSeries(string name, bool highlight)
    signal removeSeriesRequested(string name)

    function addSeries(series) {
        legendModel.append({
                               seriesName: series.name,
                               seriesColor: String(series.color)
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

    onHideSeries: function(seriesName, hidden) {
        chart.series(seriesName).visible = !hidden
    }

    onHighlightSeries: function(seriesName, highlighted) {
        chart.series(seriesName).width = 2 * (highlighted ? 2 : 1)
    }

    onRemoveSeriesRequested: function(seriesName) {
        chart.removeSeries(chart.series(seriesName))
    }

    property int maxItemWidth: 0

    Grid {
        id: grid
        anchors.centerIn: parent
        rowSpacing: 5
        columnSpacing: 10
        columns: Math.max(1, parent.width/maxItemWidth)
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
                    width: height
                    color: Qt.color(seriesColor)
                    anchors.verticalCenter: parent.verticalCenter
                }

                Label {


                    id: nameText
                    text: seriesName
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
