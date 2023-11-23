import QtQuick
import QtCharts
import QtQuick.Controls
import QtQuick.Layouts
import Common

Item {

    // public
    property Item plotterLegend

    property alias chartView: chart

    property string axisLeftTitle: ''

    property string axisRightTitle: ''

    function addSeries(seriesName, seriesProps, useSecondaryValueAxis) {
        return  _addSeries(seriesName, seriesProps, useSecondaryValueAxis)
    }

    function addPoint(seriesData, t, val) {

        // add point
        seriesData.series.append(t, val)

        // handle autoscale
        let axisValue = seriesData.axisValue

        if(axisValue.max < val && chart.autoscale) {
            axisValue.max = val + (axisValue.max - axisValue.min)*0.1
        }
        else if(axisValue.min > val && chart.autoscale) {
            axisValue.min = val - (axisValue.max - axisValue.min)*0.1
        }

        // save current time for autoscroll
        currTime = t
    }

    function resetView() {
        axisValueLeft.min = -1
        axisValueLeft.max = 1
        axisValueRight.min = -1
        axisValueRight.max = 1
        chart.autoscale = true
        chart.autoscroll = true

        chart.enabled = !chart.enabled
    }

    property real timeSpan: 10

    property var currSeries: Object()


    // private
    id: root

    implicitWidth: 400
    implicitHeight: 300

    property real currTime: 0


    function _addSeries(seriesName, seriesProps, useSecondaryValueAxis) {

        // check for existance
        let seriesEntry = currSeries[seriesName]

        // already exists, do nothing
        if(seriesEntry !== undefined) {
            console.log(`series "${seriesName}" already exists`)
            return
        }

        // create series, attach to axes
        let series = chart.createSeries(ChartView.SeriesTypeLine,
                                        seriesName);
        series.useOpenGL = true
        series.antialiasing = false
        series.axisX = axisTime

        let axisValue = undefined

        if(useSecondaryValueAxis) {
            series.axisYRight = axisValueRight
            axisValue = axisValueRight
        }
        else {
            series.axisY = axisValueLeft
            axisValue = axisValueLeft
        }


        // save to internal dict
        currSeries[seriesName] = {
            series: series,
            axisValue: axisValue,
            properties: seriesProps
        }

        return currSeries[seriesName]
    }


    function removeSeries(seriesName) {
        chart.removeSeries(chart.series(seriesName))
        delete currSeries[seriesName]
    }


    ChartView {

        id: chart
        anchors.fill: parent
        legend.visible: false
        antialiasing: true
        backgroundColor: Qt.rgba(1, 1, 1, 0.1)

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
                    xScale = 1
                }
                else {
                    width = -new_width
                    xScale = -1
                }
            }

            function setSignedHeight(new_height) {

                if(new_height > 0) {
                    height = new_height
                    yScale = 1
                }
                else {
                    height = -new_height
                    yScale = -1
                }
            }

            id: rubberBand
            color: Qt.rgba(0.8, 0.8, 0.9, 0.2)
            border.color: Qt.rgba(0.8, 0.8, 0.9, 1.0)
            border.width: 1
            visible: false
            transform: Scale {
                xScale: rubberBand.xScale
                yScale: rubberBand.yScale
            }
            property real xScale: 1.0
            property real yScale: 1.0
        }

        MouseArea {

            id: mouseArea
            anchors.fill: parent
            preventStealing: true
            acceptedButtons: Qt.LeftButton | Qt.RightButton

            onWheel: function (wheel) {

                let scale = wheel.angleDelta.y > 0 ? 6/5 : 5/6
                let scaleXy = Qt.point(scale, scale)

                let center = Qt.point(wheel.x - chart.plotArea.x,
                                      wheel.y - chart.plotArea.y)
                chart.centredZoom(scaleXy, center)
            }

            property point lastPos

            onPressed: function(mouse){
                if(mouse.button === Qt.LeftButton)
                {
                    lastPos.x = mouse.x
                    lastPos.y = mouse.y
                    chart.autoscale = false
                    chart.autoscroll = false
                }
                else if(mouse.button === Qt.RightButton)
                {
                    rubberBand.x = mouseX
                    rubberBand.y = mouseY
                    rubberBand.visible = true
                }
            }

            onMouseXChanged: {
                if(rubberBand.visible)
                {
                    rubberBand.setSignedWidth(mouseX - rubberBand.x)
                }
                else
                {
                    if(mouseX > lastPos.x)
                        chart.scrollLeft(mouseX - lastPos.x)
                    else
                        chart.scrollRight(-mouseX + lastPos.x)
                    lastPos.x = mouseX
                }
            }

            onMouseYChanged: {
                if(rubberBand.visible)
                {
                    rubberBand.setSignedHeight(mouseY - rubberBand.y)
                }
                else
                {
                    if(mouseY > lastPos.Y)
                        chart.scrollUp(mouseY - lastPos.y)
                    else
                        chart.scrollDown(-mouseY + lastPos.y)
                    lastPos.y = mouseY
                }
            }

            onReleased: {

                if(rubberBand.visible) {
                    chart.autoscale = false
                    chart.autoscroll = false
                    chart.zoomIn(Qt.rect(rubberBand.x,
                                         rubberBand.y,
                                         rubberBand.width,
                                         rubberBand.height));
                    rubberBand.visible = false
                }
            }
        }

        ValuesAxis {
            id: axisTime
            max: currTime
            min: Math.max(currTime - timeSpan, 0)
            titleText: "<font color='white'>time [s]</font>"
            labelsColor: CommonProperties.colors.primaryText
        }

        ValuesAxis {
            id: axisValueLeft
            min: -1
            max: 1
            titleText: `<font color='white'>${root.axisLeftTitle}</font>`
            labelsColor: CommonProperties.colors.primaryText
        }

        ValuesAxis {
            id: axisValueRight
            min: -1
            max: 1
            titleText: `<font color='white'>${root.axisRightTitle}</font>`
            labelsColor: CommonProperties.colors.primaryText
        }

        onSeriesAdded: function(series) {
            console.log(`++chart has ${count} elems`)
            plotterLegend.addSeries(series)
        }

        onSeriesRemoved: function(series) {
            console.log(`--chart has ${count} elems`)
            delete currSeries[series.name]
            plotterLegend.removeSeries(series)
        }

        Component.onCompleted: {
            removeAllSeries()
        }

    }



}
