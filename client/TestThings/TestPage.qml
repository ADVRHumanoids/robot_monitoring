import QtQuick
import QtQuick.Layouts
import QtQuick.Controls.Material


Rectangle {

    id: root
    color: Material.color(Material.LightGreen)

    RowLayout {

        id: mainRow

        anchors.fill: parent
        anchors.leftMargin: root.margin(root.width)
        anchors.rightMargin: root.margin(root.width)
        anchors.topMargin: 16
        anchors.bottomMargin: 16
        spacing: 16

        Repeater {

            id: mainColRepeater
            model: root.ncol(root.width)

            Rectangle {

                height: root.height
                Layout.preferredWidth: 1
                Layout.fillWidth: true
                Layout.fillHeight: true
                color: Material.color(Material.Red)

            }

            Component.onCompleted: {
                root.computeLayout()
            }

            onCountChanged: {
                root.computeLayout()
            }

        }

        Component.onCompleted: {
            root.computeLayout()
        }
    }

    function computeLayout() {

        console.log(root.width)

        let ncol = root.ncol(root.width)

        let colHeight = new Array(ncol).fill(0);

        for(let i = 0; i < container.children.length; i++) {

            let item = container.children[i]
            let colSpan = item.columnSpan[root.id(root.width)]
            let colId = item.column[root.id(root.width)]

            let cols = colHeight.slice(colId, colId + colSpan)
            let maxHeight = Math.max(...cols)

            item.y = maxHeight + mainRow.y + mainRow.childrenRect.y
            item.x = mainColRepeater.itemAt(colId).x + mainRow.x
            item.width = 1 + mainColRepeater.itemAt(colSpan-1).x - mainColRepeater.itemAt(0).x + mainColRepeater.itemAt(0).width

            for(let c = 0; c < colSpan; c++) {
                colHeight[colId + c] = item.height + 16 + maxHeight
            }

        }
    }

    Item {

        id: container

        Rectangle {

            property var column: [0, 0, 0, 0]
            property var columnSpan: [4, 4, 6, 4]

            color: Material.accent
            height: 400
            radius: 4
        }

        Rectangle {

            property var column: [0, 4, 4, 4]
            property var columnSpan: [4, 4, 6, 4]

            color: Material.secondaryTextColor
            height: 400
            radius: 4
        }
    }

    function id(viewportWidth) {
        if(viewportWidth < 600) {
            return 0;
        }
        else if(viewportWidth < 904) {
            return 1;
        }
        else if(viewportWidth < 1440) {
            return 2
        }
        else {
            return 3
        }
    }


    function margin(viewportWidth) {
        if(viewportWidth < 600) {
            return 16;
        }
        else if(viewportWidth < 904) {
            return 32;
        }
        else if(viewportWidth < 1440) {
            return 32 + (viewportWidth - 904)/540*200
        }
        else {
            return 200
        }
    }

    function ncol(viewportWidth) {
        if(viewportWidth < 600) {
            return 4;
        }
        else if(viewportWidth < 904) {
            return 8;
        }
        else if(viewportWidth < 1440) {
            return 12
        }
        else {
            return 12
        }
    }

    Component.onCompleted: {
        computeLayout()
    }

    onWidthChanged: {
        computeLayout()
    }

    onHeightChanged: {
        computeLayout()
    }

}
