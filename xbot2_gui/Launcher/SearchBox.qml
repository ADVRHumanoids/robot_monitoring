import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Common
import Font

Frame {

    required property ListModel model

    //
    id: root

    RowLayout {

        TextField {
            id: searchField
            placeholderText: 'Search query'
        }

        SmallToolButton {
            text: MaterialSymbolNames.arrowUp
            font.family: 'Material Symbols Outlined'
            font.variableAxes: {'opsz': 48}
            font.pixelSize: 16
        }

        SmallToolButton {
            text: MaterialSymbolNames.arrowDown
            font.family: 'Material Symbols Outlined'
            font.variableAxes: {'opsz': 48}
            font.pixelSize: 16
        }

    }

}
