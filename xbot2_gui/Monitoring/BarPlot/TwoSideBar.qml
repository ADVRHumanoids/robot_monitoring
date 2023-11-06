import QtQuick 2.4
import "BarPlot.js" as Logic

TwoSideBarForm {

    enum Type {
        Bar = 0,
        Segment
        }

    property int type: TwoSideBar.Type.Segment

    bar.x: type === TwoSideBar.Type.Bar ? Logic.xOffset() : Logic.normalize(value) * width - bar.width/2
    bar.width: type === TwoSideBar.Type.Bar ? Logic.width() : refMarker.width/2
    refMarker.x: Logic.normalize(valueRef) * width - refMarker.width/2
}
