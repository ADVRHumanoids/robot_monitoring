import QtQuick 2.4

BarPlotItemForm {

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

}
