import QtQuick
import QtQuick.Layouts
import "../Plotter"
import "../BarPlot"

MultiColumnPage {

    columnItems: [leftItem, rightItem]

    property Item leftItem: ColumnLayout {
        Layout.preferredWidth: 1
        Layout.fillWidth: true
        Card {
            Layout.fillHeight: true
            Layout.fillWidth: true
            frontItem: Plotter {
                anchors.fill: parent
            }
        }
        BarPlotStack {
            Layout.fillWidth: true
        }
    }

    property Item rightItem: Rectangle {
        Layout.preferredWidth: 1
        Layout.fillWidth: true
        Layout.fillHeight: true
        color: 'cyan'
        radius: 8
    }

    //        JointDevice {
    //        Layout.fillWidth: true
    //        hidden: true
    //    }



}
