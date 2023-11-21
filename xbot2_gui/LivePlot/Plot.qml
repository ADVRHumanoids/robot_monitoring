import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Item {

    ColumnLayout {

        anchors.fill: parent

        PlotterLegend {

            Layout.fillWidth: true

        }

        Plotter {

            Layout.fillHeight: true
            Layout.fillWidth: true

        }

    }

}
