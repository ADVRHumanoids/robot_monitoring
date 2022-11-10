import QtQuick
import QtQuick.Layouts
import QtQuick.Controls.Material


Rectangle {

    id: root
    objectName: '__root__'
    color: debug ? Material.color(Material.LightGreen) : Qt.rgba(0, 0, 0, 0)
    property int margin: responsiveMargin()
    property alias spacing: mainRow.spacing
    property int columns: responsiveColumns()
    property int sizeid: id(width)
    property int brSmall: 480
    property int brMedium: 768
    property int brLarge: 1200
    property bool debug: false

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

                Layout.preferredWidth: 1
                Layout.fillWidth: true
                Layout.fillHeight: true
                color: debug ? Material.color(Material.Red) : Qt.rgba(0, 0, 0, 0)

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

        let ncol = columns

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

            if(Array.isArray(colSpan)) {

                colSpan = colSpan[sizeid]
            }

            // get item's column id
            let colId = item.column || -1

            if(Array.isArray(colId)) {
                colId = colId[sizeid]
            }

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
            root.implicitHeight = maxHeight + item.height + 16 + 100

        }
    }

    function id(viewportWidth) {
        if(viewportWidth < brSmall) {
            return 0;
        }
        else if(viewportWidth < brMedium) {
            return 1
        }
        else if(viewportWidth < brLarge) {
            return 2
        }
        else {
            return 3
        }
    }


    function responsiveMargin() {
        let margins = [16, 32, 32 + (root.width - brMedium)/(brLarge-brMedium)*200, 200]
        return margins[sizeid]
    }

    function responsiveColumns() {
        let columns  = [4, 8, 12, 12]
        return columns[sizeid]
    }

}
