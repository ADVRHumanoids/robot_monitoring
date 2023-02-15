import QtQuick
import QtQuick.Layouts
import QtQuick.Controls

Item {

    property int spacing: 16

    property alias mainConsole: mainConsole

    property alias secondaryConsole: secondaryConsole

    implicitWidth: 2*spacing +
                   Math.max(hdr.implicitWidth,
                            mainConsole.implicitWidth,
                            secondaryConsole.implicitWidth)

    implicitHeight: 4*spacing +
                    hdr.implicitHeight +
                    mainConsole.implicitHeight +
                    secondaryConsole.implicitHeight

    Item {
        property string textAggregated
        property var textMap
    }

    SectionHeader {
        id: hdr
        y: spacing
        width: parent.width

        text: 'Console'
    }

    ConsoleCard {
        id: mainConsole
        y: hdr.y + hdr.height + spacing
        width: parent.width
        height: hidden ? implicitHeight : parent.height - hdr.height - secondaryConsole.height - 4*spacing

        name: 'All processes'

    }

    ConsoleCard {
        id: secondaryConsole
        y: mainConsole.y + mainConsole.height + spacing
        width: parent.width

        Layout.fillWidth: true
        hidden: true
        processNames: ['xbot2', 'ecat_master', 'perception']
    }
}
