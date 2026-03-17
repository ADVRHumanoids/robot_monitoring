import QtQuick
import QtGraphs
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

    property bool interactive: true

    signal doubleClicked()

    function addSeries(seriesName, seriesProps, useSecondaryValueAxis) {
        return  _addSeries(seriesName, seriesProps, useSecondaryValueAxis)
    }

    function addPoint(seriesData, t, val) {

        // add point
        seriesData.series.append(t, val)

        // remove old samples to avoid out of memory
        if(seriesData.series.count > 110000) {
            seriesData.series.removeMultiple(0, 10000)
        }

        // update last value inside legend
        plotterLegend.updateLastValue(seriesData.series.name, val)

        // if visible, autoscale and scroll
        let seriesVisible = seriesData.series.visible

        if(!seriesVisible) {
            return
        }

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

        // save current time for autoscroll
        currTime = t


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
        for(let i = 0; i < chart.seriesList.length; i++) {
            let s = chart.seriesList[i]
            s.removeMultiple(0, s.count)
        }
    }

    function setXRange(xmin, xmax) {
        chart.autoscroll = false
        axisTime.min = xmin
        axisTime.max = xmax
    }

    function resetView() {
        axisValueLeft.min = -1e-16
        axisValueLeft.max = 1e-16
        axisValueRight.min = -1e-16
        axisValueRight.max = 1e-16
        chart.autoscale = true
        chart.autoscroll = true
        axisTime.pan = 0
        axisValueLeft.pan = 0
        axisValueRight.pan = 0
        axisTime.zoom = 1
        axisValueLeft.zoom = 1
        axisValueRight.zoom = 1
    }

    property real timeSpan: 30

    property var currSeries: Object()


    // private
    id: root

    implicitWidth: 400
    implicitHeight: 300

    property real currTime: 0
    property real valMin: 1e9
    property real valMax: -1e9

    function _addSeries(seriesName, seriesProps, useSecondaryValueAxis) {

        // check for existance
        let seriesEntry = currSeries[seriesName]

        // already exists, do nothing
        if(seriesEntry !== undefined) {
            console.log(`series "${seriesName}" already exists`)
            return seriesEntry
        }

        // create series, attach to axes
        let series = chart.createSeries('LineSeries',
                                        seriesName);
        // series.useOpenGL = true
        // series.antialiasing = false
        // series.axisX = axisTime

        let axisValue = undefined

        if(useSecondaryValueAxis) {
            // series.axisY = axisValueRight
            axisValue = axisValueRight
        }
        else {
            // series.axisY = axisValueLeft
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


    GraphsView {

        id: chart
        anchors.fill: parent
        antialiasing: true
        marginLeft: 10
        axisX: axisTime
        axisY: axisValueLeft
        panStyle: interactive ? GraphsView.PanStyle.Drag : GraphsView.PanStyle.None
        zoomStyle: interactive ? GraphsView.ZoomStyle.Center : GraphsView.ZoomStyle.None

        theme: GraphsTheme {
            theme: GraphsTheme.Theme.MixSeries
            colorScheme: GraphsTheme.ColorScheme.Dark
            axisX.mainWidth: 1
            axisY.mainWidth: 1
            grid.mainWidth: 1
            seriesColors: [
                '#4FC3F7', // sky blue
                '#FFB74D', // warm orange
                '#81C784', // soft green
                '#BA68C8', // violet
                '#E57373', // coral red
                '#FFF176', // muted yellow
                '#4DB6AC', // teal
                '#F06292', // pink
                '#90A4AE', // cool gray-blue
                '#AED581'  // lime green
            ]
        }

        property bool autoscale: true
        property bool autoscroll: true
        property int seriesColorId: 0

        property Component lineSeriesComponent: LineSeries {}

        function createSeries(seriesType, seriesName) {
            let numAvailableColors = theme.seriesColors.length
            let seriesColor = theme.seriesColors[seriesColorId % numAvailableColors]
            seriesColorId += 1
            let s = lineSeriesComponent.createObject(chart, {'name': seriesName})
            chart.addSeries(s)
            plotterLegend.addSeries(s, seriesColor)
            return s
        }

        function series(seriesName) {
            for(let i = 0; i < seriesList.length; i++) {
                if(seriesList[i].name === seriesName) {
                    return seriesList[i]
                }
            }
        }

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

        // Rectangle {

        //     function setSignedWidth(new_width) {

        //         if(new_width > 0) {
        //             width = new_width
        //             xScale = 1
        //         }
        //         else {
        //             width = -new_width
        //             xScale = -1
        //         }
        //     }

        //     function setSignedHeight(new_height) {

        //         if(new_height > 0) {
        //             height = new_height
        //             yScale = 1
        //         }
        //         else {
        //             height = -new_height
        //             yScale = -1
        //         }
        //     }

        //     id: rubberBand
        //     color: Qt.rgba(0.8, 0.8, 0.9, 0.2)
        //     border.color: Qt.rgba(0.8, 0.8, 0.9, 1.0)
        //     border.width: 1
        //     visible: false
        //     transform: Scale {
        //         xScale: rubberBand.xScale
        //         yScale: rubberBand.yScale
        //     }
        //     property real xScale: 1.0
        //     property real yScale: 1.0
        // }

        // MouseArea {

        //     id: mouseArea
        //     anchors.fill: parent
        //     preventStealing: true
        //     acceptedButtons: Qt.LeftButton | Qt.RightButton

        //     onWheel: function (wheel) {

        //         let scale = wheel.angleDelta.y > 0 ? 6/5 : 5/6
        //         let scaleXy = Qt.point(scale, scale)

        //         let center = Qt.point(wheel.x - chart.plotArea.x,
        //                               wheel.y - chart.plotArea.y)
        //         chart.centredZoom(scaleXy, center)
        //     }

        //     property point lastPos

        //     onPressed: function(mouse){
        //         if(mouse.button === Qt.LeftButton)
        //         {
        //             lastPos.x = mouse.x
        //             lastPos.y = mouse.y
        //             chart.autoscale = false
        //             chart.autoscroll = false
        //         }
        //         else if(mouse.button === Qt.RightButton)
        //         {
        //             rubberBand.x = mouseX
        //             rubberBand.y = mouseY
        //             rubberBand.visible = true
        //         }
        //     }

        //     onMouseXChanged: {
        //         if(rubberBand.visible)
        //         {
        //             rubberBand.setSignedWidth(mouseX - rubberBand.x)
        //         }
        //         else
        //         {
        //             if(mouseX > lastPos.x)
        //                 chart.scrollLeft(mouseX - lastPos.x)
        //             else
        //                 chart.scrollRight(-mouseX + lastPos.x)
        //             lastPos.x = mouseX
        //         }
        //     }

        //     onMouseYChanged: {
        //         if(rubberBand.visible)
        //         {
        //             rubberBand.setSignedHeight(mouseY - rubberBand.y)
        //         }
        //         else
        //         {
        //             if(mouseY > lastPos.Y)
        //                 chart.scrollUp(mouseY - lastPos.y)
        //             else
        //                 chart.scrollDown(-mouseY + lastPos.y)
        //             lastPos.y = mouseY
        //         }
        //     }

        //     onReleased: {

        //         if(rubberBand.visible) {
        //             chart.autoscale = false
        //             chart.autoscroll = false
        //             chart.zoomIn(Qt.rect(rubberBand.x,
        //                                  rubberBand.y,
        //                                  rubberBand.width,
        //                                  rubberBand.height));
        //             rubberBand.visible = false
        //         }
        //     }

        //     onDoubleClicked: root.doubleClicked()
        // }

        ValueAxis {
            id: axisTime
            max: currTime
            min: Math.max(currTime - timeSpan, 0)
            titleText: `<font color='white'>${root.axisXTitle}</font>`
            gridVisible: true
            // labelsColor: CommonProperties.colors.primaryText
        }

        ValueAxis {
            id: axisValueLeft
            min: -1
            max: 1
            titleText: `<font color='white'>${root.axisLeftTitle}</font>`
            gridVisible: true
            subTickCount: 2
            // labelsColor: CommonProperties.colors.primaryText
        }

        ValueAxis {
            id: axisValueRight
            min: -1
            max: 1
            titleText: `<font color='white'>${root.axisRightTitle}</font>`
            // labelsColor: CommonProperties.colors.primaryText
        }

        // onSeriesRemoved: function(series) {
        //     console.log(`--chart has ${count} elems`)
        //     if(!root._rebuilding) {
        //         delete currSeries[series.name]
        //     }
        //     plotterLegend.removeSeries(series)
        // }

        Component.onCompleted: {
            // removeAllSeries()
        }

    }


    Connections {
        target: plotterLegend
        function onRemoveSeriesRequested(name) {
            delete currSeries[name]
        }
    }


}
