import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import "SingleJointState"
import "BarPlot"
import "sharedData.js" as SharedData
import "monitoring.js" as Logic
import "Plotter"

Page {

    id: root

    function setJointStateMessage(msg) {
        for(let i = 0; i < items.length; i++) {
            items[i].setJointStateMessage(msg)
        }
    }

    property string layoutMode: ""

    property list<Item> items

    BarPlotStack {
        id: barPlot

        Layout.fillHeight: true
        Layout.fillWidth: true

        onJointClicked: function(jn)
        {
            jointState.selectJoint(jn)
        }

        GridLayout.rowSpan: 2
    }

    SingleJointStateStack {
        id: jointState
        Layout.fillHeight: true
        Layout.fillWidth: true
        Layout.minimumWidth: implicitWidth
        onPlotAdded: function(jName, fieldName) {
            plotter.addSeries(jName, fieldName)
        }
        onPlotRemoved: function(jName, fieldName) {
            plotter.removeSeries(jName, fieldName)
        }
    }

    Plotter {
        id: plotter
        Layout.fillHeight: true
        Layout.fillWidth: true
    }

    SwipeView {
        id: swipeView
        anchors.fill: parent
        anchors.margins: 8
        clip: true
    }

    GridLayout
    {
        id: gridLayout
        anchors.fill: parent
        anchors.margins: 8
        columns: 2
    }

    footer: ToolBar {
        PageIndicator {
        anchors.centerIn: parent
        count: swipeView.count
        currentIndex: swipeView.currentIndex
    }
    }

    onWidthChanged: Logic.handleResponsiveLayout()

    onLayoutModeChanged: Logic.setLayoutMode(layoutMode)

    Component.onCompleted: {
        items = [barPlot, jointState, plotter]
        Logic.handleResponsiveLayout()
        Logic.setLayoutMode(layoutMode)
    }


}
