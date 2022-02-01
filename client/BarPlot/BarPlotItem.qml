import QtQuick 2.4

BarPlotItemForm {

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
