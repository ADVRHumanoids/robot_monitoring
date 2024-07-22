import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Common

Control {

    function addChild(paramView) {
        paramView.parent = contentColumn
        console.log(`${viewName}: added child ${paramView}`)
        paramView.width = Qt.binding(() => { return contentColumn.width })
    }

    function hasChild(name) {
        for(let i = 0; i < contentColumn.children.length; i++) {
            let child = contentColumn.children[i]
            if(child instanceof ParameterView &&
                    child.viewName === name){
                return child
            }
        }
        return null
    }

    function clear() {
        contentColumn.children = []
    }

    property string viewName

    property int treeDepth: 0

    id: root



    leftPadding: 8*root.treeDepth
    rightPadding: 0

    contentItem: Column {

        spacing: 16

        SectionHeader {
            id: header
            width: parent.width
            text: root.viewName
            iconText: contentColumn.visible ? '\uf077' : '\uf078'
            pixelSize: CommonProperties.font.h3

            onClicked: {
                contentColumn.visible = !contentColumn.visible
            }
        }

        Column {
            id: contentColumn
            width: parent.width
            spacing: 8
        }

    }

}
