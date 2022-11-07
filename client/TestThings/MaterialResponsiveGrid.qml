import QtQuick
import QtQuick.Layouts
import QtQuick.Controls.Material


Rectangle {

    id: root
    objectName: '__root__'
    color: Material.color(Material.LightGreen)
    property int margin: responsiveMargin(width)
    property alias spacing: mainRow.spacing
    property int columns: responsiveColumns(width)

    RowLayout {

        id: mainRow
        objectName: '__mainRow__'
        anchors.fill: parent
        anchors.leftMargin: root.margin
        anchors.rightMargin: root.margin
        anchors.topMargin: 16
        anchors.bottomMargin: 16
        spacing: 16

        Repeater {

            id: mainColRepeater
            model: root.columns

            Rectangle {

                height: root.height
                Layout.preferredWidth: 1
                Layout.fillWidth: true
                Layout.fillHeight: true
                color: Material.color(Material.Red)

                onXChanged: {
                    Qt.callLater(root.computeLayout)
                }

            }

        }

    }

    onChildrenChanged: {
        let item = children[children.length - 1]
        item.heightChanged.connect(root.computeLayout)
    }

    function computeLayout() {

        let ncol = root.responsiveColumns(root.width)

        let colHeight = new Array(ncol).fill(0);

        let maxHeight = 0

        let currentCol = 0

        for(let i = 0; i < children.length; i++) {

            let item = children[i]

            if(item.objectName === '__mainRow__') {
                continue
            }

            if(item instanceof Repeater) {
                continue
            }

            // get item's column span (defaults to 4)
            let colSpan = item.columnSpan || 4

            // get item's column id
            let colId = item.column || -1

            // if colId was -1, we compute it based on flow type
            if(colId === -1) {
                if(currentCol + colSpan <= ncol) {
                    colId = currentCol
                }
                else {
                    colId = 0
                }
            }

            // compute max y for all spanned columns
            let cols = colHeight.slice(colId, colId + colSpan)
            maxHeight = Math.max(...cols)

            // place item and compute its width
            item.y = maxHeight + mainRow.y + mainRow.childrenRect.y
            item.x = mainColRepeater.itemAt(colId).x + mainRow.x
            item.width = mainColRepeater.itemAt(colSpan-1).x - mainColRepeater.itemAt(0).x + mainColRepeater.itemAt(0).width

            // update current column
            currentCol += colSpan
            if(currentCol >= ncol) {
                currentCol = 0
            }

            // update max y of all spanned columns
            for(let c = 0; c < colSpan; c++) {
                colHeight[colId + c] = item.height + 16 + maxHeight
            }

            // set implicit height
            root.implicitHeight = maxHeight + item.height + 16

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


    function responsiveMargin(viewportWidth) {
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

    function responsiveColumns(viewportWidth) {
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

}
