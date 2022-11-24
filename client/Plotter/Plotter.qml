import QtQuick
import QtCharts
import QtQuick.Controls
import xbot2_gui.common

Item {

    id: root
    implicitWidth: 300

    property real currTime: 0
    property real timeSpan: 10
    property real initialTime: -1

    property var currSeries: Object()

    function addSeries(jName, fieldName) {

        let seriesName = jName + "/" + fieldName

        let seriesEntry = currSeries[seriesName]

        if(seriesEntry !== undefined) {
            return
        }

        let series = chart.createSeries(ChartView.SeriesTypeLine, seriesName,
                                        axisTime, axisValue);
        series.useOpenGL = true
        series.antialiasing = false

        currSeries[seriesName] = {
            series: series,
            jName: jName,
            fieldName: fieldName,
            jIndex: 0
        }
    }

    function removeSeries(jName, fieldName) {
        let seriesName = jName + "/" + fieldName
        chart.removeSeries(chart.series(seriesName))
        delete currSeries[seriesName]
    }

    function setJointStateMessage(msg) {
        for(let [key, value] of Object.entries(currSeries)) {
            let series = value.series
            let jName = value.jName

            if(msg.name[value.jIndex] !== jName) {
                value.jIndex = msg.name.indexOf(jName)
            }

            let t = msg.stamp

            if(initialTime < 0) {
                initialTime = t
            }

            t = t - initialTime
            let val = msg[value.fieldName][value.jIndex]

            series.append(t, val)

            if(axisValue.max < val && chart.autoscale) {
                axisValue.max = val + (axisValue.max - axisValue.min)*0.05
            }
            else if(axisValue.min > val && chart.autoscale) {
                axisValue.min = val - (axisValue.max - axisValue.min)*0.05
            }

            currTime = t

        }
    }

    Column {
        id: column
        anchors.fill: parent

        PlotterLegend {
            id: plotterLegend
            width: root.width

            onHideSeries: function(seriesName, hidden) {
                chart.series(seriesName).visible = !hidden
            }

            onHighlightSeries: function(seriesName, highlighted) {
                chart.series(seriesName).width = 2 * (highlighted ? 2 : 1)
            }

            onRemoveSeriesRequested: function(seriesName) {
                chart.removeSeries(chart.series(seriesName))
            }
        }

        ChartView {

            id: chart
            width: root.width
            height: parent.height - plotterLegend.height - buttonRow.height
            legend.visible: false
            antialiasing: true
            margins {
                bottom: 3
                top: 3
                left: 3
                right: 3
            }
            backgroundColor: Qt.rgba(1, 1, 1, 0.2)

            property bool autoscale: true
            property bool autoscroll: true

            function centredZoom(scale, center) {

                chart.autoscale = false
                chart.autoscroll = false
                axisTime.min = axisTime.min
                axisTime.max = axisTime.max

                let zoomRect = Qt.rect(chart.plotArea.x + (scale.x - 1)*center.x,
                                       chart.plotArea.y + (scale.y - 1)*center.y,
                                       chart.plotArea.width/scale.x,
                                       chart.plotArea.height/scale.y)

                chart.zoomIn(zoomRect)
            }

            onAutoscrollChanged: {
                if(autoscroll) {
                    axisTime.min = Qt.binding(function(){ return Math.max(currTime - timeSpan, 0) })
                    axisTime.max = Qt.binding(function(){ return currTime })
                }
                else {
                    axisTime.min = axisTime.min
                    axisTime.max = axisTime.max
                }
            }

            Rectangle {

                function setSignedWidth(new_width) {

                    if(new_width > 0) {
                        width = new_width
                        transform.xScale = 1
                    }
                    else {
                        width = -new_width
                        transform.xScale = -1
                    }

                }

                function setSignedHeight(new_height) {

                    if(new_height > 0) {
                        height = new_height
                        transform.yScale = 1
                    }
                    else {
                        height = -new_height
                        transform.yScale = -1
                    }
                }

                id: rubberBand
                color: Qt.rgba(0.8, 0.8, 0.9, 0.2)
                border.color: Qt.rgba(0.8, 0.8, 0.9, 1.0)
                border.width: 1
                visible: false
                transform: Scale {
                    xScale: 1
                    yScale: 1
                }
            }

            MouseArea {
                id: mouseArea
                anchors.fill: parent

                onWheel: function (wheel) {
                    print(wheel.angleDelta)
                    print(wheel.x)
                    print(wheel.y)
                    print(chart.plotArea)

                    let scale = wheel.angleDelta.y > 0 ? 6/5 : 5/6
                    let scaleXy = Qt.point(scale, scale)

                    let center = Qt.point(wheel.x - chart.plotArea.x,
                                          wheel.y - chart.plotArea.y)
                    chart.centredZoom(scaleXy, center)
                }

                onPressed: {
                    rubberBand.x = mouseX
                    rubberBand.y = mouseY
                    rubberBand.visible = true
                }

                onMouseXChanged: {
                    rubberBand.setSignedWidth(mouseX - rubberBand.x)
                }

                onMouseYChanged: {
                    rubberBand.setSignedHeight(mouseY - rubberBand.y)
                }

                onReleased: {

                    if(rubberBand.visible) {
                        chart.autoscale = false
                        chart.autoscroll = false
                    }

                    rubberBand.visible = false

                    chart.zoomIn(Qt.rect(rubberBand.x,
                                         rubberBand.y,
                                         rubberBand.width,
                                         rubberBand.height));
                }
            }

            ValuesAxis {
                id: axisTime
                max: currTime
                min: Math.max(currTime - timeSpan, 0)
                titleText: "time [s]"
                labelsColor: CommonProperties.colors.primaryText
            }

            ValuesAxis {
                id: axisValue
                min: -1
                max: 1
                labelsColor: CommonProperties.colors.primaryText
            }

            LineSeries {
                id: emptySeries
                axisX: axisTime
                axisY: axisValue
            }

            onSeriesAdded: function(series) {
                plotterLegend.addSeries(series)
            }

            onSeriesRemoved: function(series) {
                delete currSeries[series.name]
                plotterLegend.removeSeries(series)
            }

            Component.onCompleted: {
                removeAllSeries()
            }

        }

        Row {
            id: buttonRow
            width: root.width

            Button {
                text: "Reset view"
                onReleased: {
                    chart.autoscale = true
                    chart.autoscroll = true
                }
            }
        }

    }

}
