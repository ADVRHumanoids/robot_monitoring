import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Control {

    property int verbosity: 0

    property bool autoscroll: true

    function addMsg(txt, name, level = 0) {

        model.append({'txt': txt, 'name': name, 'level': level})

        if(level >= verbosity) {
            filteredModel.append({'txt': txt, 'name': name, 'level': level})
        }

        if(autoscroll) view.positionViewAtEnd()
    }

    onVerbosityChanged: {
        filteredModel.clear()
        for(let i = 0; i < model.count; i++) {
            let item = model.get(i)
            if(item.level >= verbosity) {
                filteredModel.append(item)
            }
        }
    }

    function clear() {
        model.clear()
        filteredModel.clear()
    }

    signal dismissRequested()

    //
    id: root
    padding: 10

    ListModel {
        id: model
    }

    ListModel {
        id: filteredModel
    }

    property Component delegate: Component {
        TextArea {

            MouseArea {
                anchors.fill: parent
                enabled: false
            }

            placeholderText: name
            width: view.width
            wrapMode: Text.WrapAnywhere
            readOnly: true
            textFormat: TextEdit.RichText
            text: txt
            visible: level >= root.verbosity
            color: levelToColor[level]
            property list<color> levelToColor: [palette.text, CommonProperties.colors.warn, CommonProperties.colors.err]
        }
    }

    background: Rectangle {
        color: root.palette.window
        border.color: root.palette.accent
        border.width: 1
        radius: 8
    }

    contentItem: ColumnLayout {

        id: mainCol

        ListView {
            id: view
            model: filteredModel
            delegate: root.delegate
            implicitHeight: contentHeight
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 6
            clip: true
        }

        Button {
            Layout.alignment: Qt.AlignRight
            text: 'Dismiss'
            onClicked: {
                root.dismissRequested()
            }
        }

    }

}
