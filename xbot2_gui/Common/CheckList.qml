import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Common
import Font


Item {

    default property list<Item> checkListItems

    //
    id: root
    implicitHeight: grid.implicitHeight
    implicitWidth: grid.implicitWidth



    GridLayout {
        anchors.fill: parent
        id: grid
        columns: 2
    }

    property Component iconComponent: Component {
        Label {
            property bool checked: false
            text: checked ? sym.done : sym.cross
            color: checked ? 'green' : 'red'
            font.family: sym.font.family
            font.pointSize: CommonProperties.font.h4
            font.bold: checked
            MaterialSymbols {
                id: sym
            }
        }
    }

    property Component textComponent: Component {
        Label {
            property bool checked: false
            color: checked ? 'green' : 'red'
            font.bold: checked
            font.pointSize: CommonProperties.font.h4
            Layout.fillWidth: true
        }
    }

    Component.onCompleted: {
        for(let c of checkListItems) {
            if(c instanceof CheckListItem) {

                iconComponent.createObject(grid,
                                           {
                                               'checked': Qt.binding(() => { return c.checked; })
                                           })

                textComponent.createObject(grid,
                                           {
                                               'checked': Qt.binding(() => { return c.checked; }),
                                               'text': c.text
                                           })
            }
        }
    }



}
