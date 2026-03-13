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

    property string axisXTitle: 'time [s]'

    property alias interactive: mouseArea.enabled

    signal doubleClicked()

    function addSeries(seriesName, seriesProps, useSecondaryValueAxis) {
        return  _addSeries(seriesName, seriesProps, useSecondaryValueAxis)
    }

    function addPoint(seriesData, t, val) {

        // add point
        seriesData.series.append(t, val)

        // handle autoscale
        let axisValue = seriesData.axisValue
        let paddedValUp = val + (axisValue.max - axisValue.min)*0.1
        let paddedValDown = val - (axisValue.max - axisValue.min)*0.1

        if(axisValue.max < paddedValUp && chart.autoscale) {
            axisValue.max = paddedValUp
        }
        else if(axisValue.min > paddedValDown && chart.autoscale) {
            axisValue.min = paddedValDown
        }

        valMax = Math.max(val, valMax)
        valMin = Math.min(val, valMin)

        // remove old samples to avoid out of memory
        if(seriesData.series.count > 110000) {
            seriesData.series.removePoints(0, 10000)
        }

        // save current time for autoscroll
        currTime = t

        // update last value inside legend
        plotterLegend.updateLastValue(seriesData.series.name, val)
    }

    function setPoints(seriesData, t_list, val_list) {

        // set points
        rebuilder.setPoints(seriesData.series, t_list, val_list)
        let valMax = Math.max(...val_list)
        let valMin = Math.min(...val_list)

        // handle autoscale
        let axisValue = seriesData.axisValue

        console.log(`min = ${valMin} max = ${valMax} -- ${axisValue.max} ${axisValue.min}`)

        if(axisValue.max < valMax && chart.autoscale) {
            axisValue.max = valMax + (axisValue.max - axisValue.min)*0.1
        }

        if(axisValue.min > valMin && chart.autoscale) {
            axisValue.min = valMin - (axisValue.max - axisValue.min)*0.1
        }
    }

    function clearPoints() {
        for(let i = 0; i < chart.count; i++) {
            let s = chart.series(i)
            s.removePoints(0, s.count)
        }
    }

    function setXRange(xmin, xmax) {
        chart.autoscroll = false
        axisTime.min = xmin
        axisTime.max = xmax
    }

    function rebuild() {

        if(_rebuilding) {
            return
        }

        _rebuilding = true



        for(let i = 0; i < chart.count; i++) {

            // save points, type, name
            let series = chart.series(i)
            console.log(series, series.name)

            let points = rebuilder.getPoints(series)
            let type = series.type
            let name = series.name

            // remove
            chart.removeSeries(series)

            // create
            series = chart.createSeries(type,
                                        name);

            series.useOpenGL = true
            series.antialiasing = false
            series.axisX = axisTime
            series.axisY = axisValueLeft

            // fill with saved points
            rebuilder.setPoints(series, points)

            // update seriesdata
            currSeries[name].series = series
        }
        _rebuilding = false
    }

    function resetView() {
        axisValueLeft.min = -1e-16
        axisValueLeft.max = 1e-16
        axisValueRight.min = -1e-16
        axisValueRight.max = 1e-16
        chart.autoscale = true
        chart.autoscroll = true
    }

    property real timeSpan: 30

    property var currSeries: Object()


    // private
    id: root

    implicitWidth: 400
    implicitHeight: 300

    property bool _rebuilding: false
    property real currTime: 0
    property real valMin: 1e9
    property real valMax: -1e9

    PlotRebuilder {
        id: rebuilder
    }

    function _addSeries(seriesName, seriesProps, useSecondaryValueAxis) {

        // check for existance
        let seriesEntry = currSeries[seriesName]

        // already exists, do nothing
        if(seriesEntry !== undefined) {
            console.log(`series "${seriesName}" already exists`)
            return seriesEntry
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
        currSeries[seriesName].destroy()
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

            onDoubleClicked: root.doubleClicked()
        }

        ValuesAxis {
            id: axisTime
            max: currTime
            min: Math.max(currTime - timeSpan, 0)
            titleText: `<font color='white'>${root.axisXTitle}</font>`
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
            if(!root._rebuilding) {
                delete currSeries[series.name]
            }
            plotterLegend.removeSeries(series)
        }

        Component.onCompleted: {
            // removeAllSeries()
        }

    }



}
