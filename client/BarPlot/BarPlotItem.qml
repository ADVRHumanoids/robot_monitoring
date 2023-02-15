import QtQuick

BarPlotItemForm {

    id: root

    property alias statusOk: root.statusOk

    signal jointClicked(string jName)

    labelMouseArea.onHoveredChanged: {
        if(labelMouseArea.containsMouse)
        {
            labelColorAlpha = 0.6
        }
        else
        {
            labelColorAlpha = 0.3
        }
    }

    labelMouseArea.onClicked:  jointClicked(jointName)

}
