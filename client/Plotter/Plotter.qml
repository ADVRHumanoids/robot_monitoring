import QtQuick 2.0
import QtCharts

Item {

    id: root
    implicitWidth: 300

    property real currTime: 0
    property real timeSpan: 10
    property real initialTime: -1

    property var currSeries: []

    function addSeries(jName, fieldName) {

        let idx = currSeries.findIndex( item => item.jName === jName && item.fieldName === fieldName)

        if(idx > -1)
        {
            print('series already exists')
            return
        }

        let series = chart.createSeries(ChartView.SeriesTypeLine, jName + "/" + fieldName,
                                        axisTime, axisValue);
        series.useOpenGL = true
        series.antialiasing = false

        currSeries.push({
                            series: series,
                            jName: jName,
                            fieldName: fieldName,
                            jIndex: 0
                        })
    }

    function setJointStateMessage(msg) {
        for(let i = 0; i < currSeries.length; i++) {

            let series = currSeries[i].series
            let jName = currSeries[i].jName

            if(msg.name[currSeries[i].jIndex] !== jName) {
                currSeries[i].jIndex = msg.name.indexOf(jName)
            }

            let t = msg.stamp

            if(initialTime < 0) {
                initialTime = t
            }

            t = t - initialTime
            let val = msg[currSeries[i].fieldName][currSeries[i].jIndex]

            series.append(t, val)

            if(axisValue.max < val) {
                axisValue.max = val + (axisValue.max - axisValue.min)*0.05
            }
            else if(axisValue.min > val) {
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
            height: parent.height - plotterLegend.height
            legend.visible: false
            antialiasing: true

            ValuesAxis {
                id: axisTime
                max: currTime
                min: currTime - timeSpan
            }

            ValuesAxis {
                id: axisValue
                min: -1
                max: 1
            }

            onSeriesAdded: function(series) {
                plotterLegend.addSeries(series)
            }

            onSeriesRemoved: function(series) {
                currSeries = currSeries.filter( item => item.series.name !== series.name )
                plotterLegend.removeSeries(series)
            }

        }

    }

}
