import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import Common

Item {

    property int columns

    property int rowSpacing: 24

    property int columnSpacing: 12

    default property alias contentData: contentOverlay.data

    //
    id: root

    implicitHeight: row.implicitHeight
    implicitWidth: row.implicitWidth
    height: scroll.height

    Item {

        objectName: 'MCL_IGNORE'

        id: scroll

        implicitHeight: row.implicitHeight
        implicitWidth: row.implicitWidth

        height: row.height
        width: parent.width



        Item {
            id: contentOverlay
            anchors.fill: row
            z: 10

            onChildrenChanged: {
                console.log(`children changed -> ${children.length}`)
                Qt.callLater(root.computeLayout)
            }
        }

        RowLayout {

            id: row
            spacing: root.rowSpacing
            width: parent.width

            Repeater {

                id: colRepeater
                model: root.columns

                Item {

                    id: col
                    Layout.preferredWidth: 1
                    Layout.fillWidth: true
                    required property int index

                }
            }
        }

    }

    function clear() {
        let cols = root.columns
        colRepeater.model = 0
        colRepeater.model = Qt.binding(() => { return root.columns })
    }

    function computeLayout() {

        clear()

        let currentColIdx = 0

        for(let i = 0; i < contentData.length; i++) {

            let item = contentData[i]

            if(item.objectName === "MCL_IGNORE") {
                continue
            }

            if(item instanceof Repeater) {
                continue
            }

            if(!item.visible) {
                continue
            }

            let colSpan = item.columnSpan === undefined ? 1 : item.columnSpan

            if(currentColIdx + colSpan > root.columns) {
                currentColIdx = 0
            }

            Object.defineProperty(item, '_currentColIdx',
                                  {
                                      enumerable: false,
                                      configurable: false,
                                      writable: true,
                                      value: currentColIdx
                                  })

            let currentCol = colRepeater.itemAt(currentColIdx)

            // compute x
            item.x = Qt.binding(
                        () => {
                            return currentCol.x
                        })

            item.width = Qt.binding(
                        () => {
                            return colSpan * currentCol.width + root.rowSpacing * (colSpan-1)
                        })

            // register height changed binding
            item.heightChanged.connect(updateVerticalPosition)

            // increment col idx
            currentColIdx += colSpan

        }

        updateVerticalPosition()

    }

    function updateVerticalPosition(istart, iend) {

        let colHeight = Array(columns).fill(0);

        for(let i = 0; i < contentData.length; i++) {

            let item = contentData[i]

            if(item.objectName === "MCL_IGNORE") {
                continue
            }

            if(item instanceof Repeater) {
                continue
            }


            // get item's column idx and span
            let currentColIdx = item._currentColIdx
            let colSpan = item.columnSpan === undefined ? 1 : item.columnSpan

            // compute height of spanning columns
            let maxHeight = 0

            for(let c = currentColIdx; c < currentColIdx + colSpan; c++) {
                maxHeight = Math.max(maxHeight, colHeight[c])
            }

            for(let c = currentColIdx; c < currentColIdx + colSpan; c++) {
                colHeight[c] = maxHeight + item.height + root.columnSpacing
            }

            item.y = maxHeight
        }

        for(let c = 0; c < columns; c++) {
            colRepeater.itemAt(c).Layout.preferredHeight = colHeight[c]
        }
    }

    Component.onCompleted: {
        computeLayout()
    }

    onColumnsChanged: {
        Qt.callLater(computeLayout)
    }


}
