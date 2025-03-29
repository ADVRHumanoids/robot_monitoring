import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import Common

Control {

    property int columns: 1
    property int rowSpacing: 8
    property int columnSpacing: 8

    //
    id: root
    default property alias content: content.children
    property list<Item> _items_to_position
    property bool _layout_in_progress: false


    contentItem: RowLayout {

        id: row

        spacing: root.rowSpacing
        uniformCellSizes: true

        Repeater {

            model: root.columns

            id: colRepeater

            ColumnLayout {

                required property int index

                Layout.fillWidth: true
                Layout.fillHeight: true
                Layout.alignment: Qt.AlignTop

                spacing: root.columnSpacing

                // Text {
                //     text: `Column ${index}`
                //     color: palette.active.text
                // }

            }

            onItemAdded: Qt.callLater(computeLayout)
            onItemRemoved: Qt.callLater(computeLayout)
        }

    }

    Item {
        id: content
        onChildrenChanged: {
            console.log('onChildrenChanged')
            if(!_layout_in_progress) {
                _items_to_position = []
                console.log('scheduling computeLayout')
                Qt.callLater(computeLayout)
            }
        }
    }

    function computeLayout() {

        try {

        console.log('computeLayout START')
        _layout_in_progress = true

        for(let i = 0; i < content.children.length; i++) {
            let item = content.children[i]
            if(item instanceof Repeater) {
                continue
            }
            _items_to_position.push(item)
        }

        let item_parent_list = []
        let c = 0

        for(let i = 0; i < _items_to_position.length; i++) {
            let item = _items_to_position[i]
            let column = row.visibleChildren[c]
            // console.log(i)
            // console.log(c)
            // console.log(item)
            // console.log(column)
            item_parent_list.push([item, column])
            c = (c + 1) % root.columns
        }

        for(let i = 0; i < item_parent_list.length; i++) {
            let item = item_parent_list[i][0]
            let parent = item_parent_list[i][1]
            item.parent = parent
            // item.width = Qt.binding(() => {return parent.width})
            // item.anchors.left = parent.left
            // item.anchors.right = parent.right
            item.Layout.fillWidth = true
            item.Layout.preferredHeight = Qt.binding(() => { return item.height })
        }

        }
        catch(e) {
            console.log(`Exception during computeLayout: ${e}`)
            computeLayoutDeferred.start()
        }
        finally {
            _layout_in_progress = false
            console.log('computeLayout END')
        }

    }

    Timer {
        id: computeLayoutDeferred
        interval: 333
        onTriggered: root.computeLayout()
        running: false
        repeat: false
    }

    // onColumnsChanged: Qt.callLater(computeLayout)

    // Component.onCompleted: computeLayout()

}
