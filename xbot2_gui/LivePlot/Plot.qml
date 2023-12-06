import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Common

MultiPaneResponsiveLayout {

    signal pageSelected()

    onPageSelected: livePlot.rebuild()

    onAfterLayoutChange: livePlot.rebuild()

    LayoutClassHelper {
        id: lay
        targetWidth: parent.width
    }

    GridLayout {

        anchors.fill: parent

        rows: lay.expanded ? 1 : -1
        columns: lay.expanded ? -1 : 1

        PlotterLegend {
            id: plotterLegend
            chart: livePlot.chartView
            Layout.fillWidth: !lay.expanded
            Layout.fillHeight: lay.expanded
        }

        Plotter {

            id: livePlot
            plotterLegend: plotterLegend
            Layout.fillHeight: true
            Layout.fillWidth: true
            property real initialTime: -1.0
            timeSpan: rangeSpin.value

            Control {

                anchors {
                    top: livePlot.top
                    right: livePlot.right
                    rightMargin: CommonProperties.geom.margins
                    topMargin: 24
                }

                opacity: 0.9

                padding: CommonProperties.geom.margins/2

                background: Rectangle {
                    radius: 4
                    color: palette.window
                }

                contentItem: Column {
                    id: rangeCol
                    spacing: 3

                    Button {
                        text: 'Reset'
                        onClicked: livePlot.resetView()
                    }

                    Button {
                        text: 'Rebuild'
                        onClicked: livePlot.rebuild()
                        visible: false
                    }

                    Label {
                        text: 'Range'
                        font.pointSize: 10
                    }

                    DoubleSpinBox {
                        id: rangeSpin
                        from: 0
                        to: 1000
                        stepSize: 1
                        value: 10
                    }
                }
            }
        }
    }

    Component.onCompleted: {
        CommonProperties.globalLivePlot = livePlot
    }

}
