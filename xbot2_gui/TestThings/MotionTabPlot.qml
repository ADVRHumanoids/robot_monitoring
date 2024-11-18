import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import LivePlot

Control {

    property alias title: plot.chartView.title

    property alias xLabel: plot.axisXTitle

    property alias timeSpan: plot.timeSpan

    function setXRange(xmin, xmax) {
        plot.setXRange(xmin, xmax)
    }

    function resetView() {
        plot.resetView()
    }

    function addSeries(name, props) {
        series[props.fieldName] = plot.addSeries(name, props, false)
    }

    function addPointsFromMsg(t, msg) {
        for(const [field, s] of Object.entries(series)) {
            plot.addPoint(s, t, msg[field][0])
        }
    }

    function addPoint(field, t, x) {
        plot.addPoint(series[field], t, x)
    }

    signal activated()

    //
    id: root

    property var series: Object()

    contentItem: Plotter {

        id: plot

        plotterLegend: legend

        interactive: true

        onDoubleClicked: root.activated()

        chartView.title: 'Title'
        chartView.titleColor: palette.text
        chartView.margins {
            bottom: 0
            left: 0
            right: 3
            top: 0
        }

        Control {

            padding: 3

            anchors {
                top: parent.top
                right: parent.right
                margins: 3
            }

            contentItem: PlotterLegend {
                id: legend
                chart: plot.chartView
                visible: true
            }

            background: Rectangle {
                radius: 3
                color: Qt.rgba(0, 0, 0, 0.6)
                border.width: 1
                border.color: Qt.rgba(1, 1, 1, 0.4)
            }

        }

    }

}
