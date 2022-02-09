import QtQuick 2.0

Item {

    id: root
    height: grid.implicitHeight

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

                Text {


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
