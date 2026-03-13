import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Control {

    property alias text: textLabel.text
    property alias subtext: subtextLabel.text
    property bool horizontal: true
    property alias frameBackground: frame.background

    //
    id: root
    default property alias data_: row.data

    contentItem: Frame {
        id: frame
        Layout.columnSpan: 2
        Layout.fillWidth: true
        GridLayout {
            id: row
            anchors.fill: parent
            rows: horizontal ? 1 : undefined
            columns: horizontal ? undefined : 1
            columnSpacing: 6
            rowSpacing: 6
            Column {
                Layout.preferredWidth: Math.max(textLabel.implicitWidth, subtextLabel.implicitWidth)
                Layout.fillWidth: true
                Label {
                    id: textLabel
                    font.bold: true
                    width: parent.width
                }
                Label {
                    id: subtextLabel
                    font.pixelSize: 10
                    width: parent.width
                    wrapMode: Text.Wrap
                    visible: text !== ''
                }
            }
        }
    }

}
