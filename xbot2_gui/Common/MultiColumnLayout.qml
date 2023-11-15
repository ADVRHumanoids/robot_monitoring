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

    property int _nitems: 0

    property list<int> _colHeight

    Item {

        objectName: 'MCL_IGNORE'

        id: scroll
        anchors.fill: parent

        implicitHeight: row.implicitHeight
        implicitWidth: row.implicitWidth

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
            width: scroll.width

            Repeater {

                id: colRepeater
                model: root.columns

                Column {

                    id: col

                    spacing: root.columnSpacing

                    property list<Item> items

                    required property int index

                    property Component placeholder: Component {
                        Item {
                            required property Item target
                            implicitWidth: target.implicitWidth
                            width: col.width
                            height: target.height
                        }
                    }

                    function createPlaceholder(item) {
                        return placeholder.createObject(col,
                                               {
                                                    'target': item
                                               })
                    }

                    Layout.fillWidth: true
                    Layout.preferredWidth: 1
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

            console.log(`lay out object ${item.objectName}`)

            if(item.objectName === "MCL_IGNORE") {
                continue
            }

            if(item instanceof Repeater) {
                continue
            }

            let colSpan = item.columnSpan === undefined ? 1 : item.columnSpan

            if(currentColIdx + colSpan > root.columns) {
                currentColIdx = 0
            }

            let currentCol = colRepeater.itemAt(currentColIdx)

            // contruct placeholder item
            let placeholders = []

            for(let c = currentColIdx; c < currentColIdx + colSpan; c++) {
                let col = colRepeater.itemAt(c)
                let pl = col.createPlaceholder(item)
                placeholders.push(pl)

                console.log(`pl height = ${pl.height}`)
            }

            let pl0 = placeholders[0]
            let pl1 = placeholders[colSpan-1]

            // compute x
            item.x = Qt.binding(
                        () => {
                            return pl0.x + currentCol.x
                        })

            item.width = Qt.binding(
                        () => {
                            return colSpan * pl0.width + root.rowSpacing * (colSpan-1)
                        })

            // compute y
            item.y = Qt.binding(
                        () => {
                            let maxY = 0
                            for(const pl of placeholders) {
                                maxY = Math.max(maxY, pl.y)
                            }
                            return maxY
                        })

            // increment col idx
            currentColIdx += colSpan

        }

    }

    Component.onCompleted: {
        computeLayout()
    }

    onColumnsChanged: {
        computeLayout()
    }


}
