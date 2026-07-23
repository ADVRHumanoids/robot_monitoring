import QtQuick
import QtQuick.Layouts
import QtQuick.Controls

Item {

    TreeView {
        anchors.fill: parent
        anchors.margins: 16
        model: TreeModel { }
        selectionModel: ItemSelectionModel {}
    }
}
