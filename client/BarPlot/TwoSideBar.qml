import QtQuick 2.4
import "logic.js" as Logic

TwoSideBarForm {

    bar.x: Logic.xOffset()
    bar.width: Logic.width()
    centerLine.x: Logic.xZero()
}
