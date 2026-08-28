import QtQuick
import QtQuick.Layouts
import QtQuick.Controls

import Common

Control {

    function loadData(msg) {
        treeView.loadData(msg)
    }

    function focusItem(path) {
        treeView.focusItem(path)
    }

    property alias fullModel: treeView.fullModel

    contentItem: GridLayout {

        columns: layout.expanded ? 2 : 1
        columnSpacing: 16

        DiagnosticsTreeView {
            id: treeView
            Layout.fillHeight: true
            Layout.fillWidth: true
            Layout.preferredWidth: 400
            Layout.preferredHeight: 400
        }

        Item {

            Layout.fillHeight: true
            Layout.fillWidth: true
            Layout.preferredWidth: 200
            Layout.preferredHeight: 400

            // Label {
            //     width: parent.width
            //     horizontalAlignment: Text.AlignHCenter
            //     anchors.centerIn: parent
            //     anchors.margins: 16
            //     visible: !treeView.selectionModel.currentIndex.valid
            //     text: 'No item selected'
            //     color: palette.disabled.text
            //     font.pixelSize: CommonProperties.font.h2
            //     wrapMode: Text.Wrap
            // }

            DetailedView {

                anchors.fill: parent
                // visible: treeView.selectionModel.currentIndex.valid
                enabled: treeView.selectionModel.currentIndex.valid
                message: treeView.model.data(treeView.selectionModel.currentIndex, TreeModel.MessageRole) ?? '--'
                hwId: treeView.model.data(treeView.selectionModel.currentIndex, TreeModel.HardwareIdRole) ?? '--'
                path: treeView.model.data(treeView.selectionModel.currentIndex, TreeModel.PathRole) ?? '--'
                name: treeView.model.data(treeView.selectionModel.currentIndex, TreeModel.NameRole) ?? 'no diagnostic selected'
                level: treeView.model.data(treeView.selectionModel.currentIndex, TreeModel.LevelRole) ?? 0
                metrics: treeView.model.data(treeView.selectionModel.currentIndex, TreeModel.MetricsRole)

            }

        }
    }

}
