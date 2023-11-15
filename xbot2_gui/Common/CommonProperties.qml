pragma Singleton

import QtQuick
import QtQuick.Controls.Material


Item {

    property Item colors: Item {
        property color primary: Material.primaryColor
        property color accent: Material.accentColor
        property color ok: Material.color(Material.Green, Material.Shade900)
        property color warn: Material.color(Material.Yellow, Material.Shade900)
        property color err: Material.color(Material.Red, Material.Shade900)
        property color primaryText: Material.primaryTextColor
        property color secondaryText: Material.secondaryTextColor
        property color cardBackground: Qt.rgba(1, 1, 1, 0.075)
        property color frame: Material.frameColor
    }

    property Item geom: Item {
        property int mobileBreakpoint: 576
        property int cardRadius: 4
        property int spacing: 16
        property int margins: compactLayout ? 16 : 24
        property bool compactLayout: false
        property bool mediumLayout: false
        property bool expandedLayout: false
    }

    property Item font: Item {
        property int h1: 24
        property int h2: 20
        property int h3: 16
        property int h4: 14
    }

    property Item fontAwesome: Item {
        id: fontAwesome

        readonly property FontLoader fontAwesomeSolid: FontLoader {
            source:  "/Font/fontawesome-free-6.3.0-desktop/otfs/Font Awesome 6 Free-Solid-900.subset.otf"
        }

        readonly property FontLoader fontAwesomeRegular: FontLoader {
            source: "/Font/fontawesome-free-6.3.0-desktop/otfs/Font Awesome 6 Free-Regular-400.subset.otf"
        }



        property alias regular: fontAwesome.fontAwesomeRegular.font
        property alias solid: fontAwesome.fontAwesomeSolid.font

        property string hamburger: '\uf0c9'
        property string home: '\uf015'
        property string settings: '\uf013'
        property string play: '\uf04b'
        property string stop: '\uf04d'
        property string halt: '\uf256'
        property string gamepad: '\uf11b'

    }

    property Item materialSymbols: Item {

        id: materialSymbols

        readonly property FontLoader materialSymbols20_200_1_200: FontLoader {
            source: "/Font/materialsymbols/MaterialSymbolsOutlined[opsz,wght,FILL,GRAD@20,200,1,200].otf"
        }

        readonly property FontLoader materialSymbols20_200_0_200: FontLoader {
            source: "/Font/materialsymbols/MaterialSymbolsOutlined[opsz,wght,FILL,GRAD@20,200,0,200].otf"
        }

        property alias material_20_200_filled_200: materialSymbols.materialSymbols20_200_1_200.font
        property alias material_20_200_empty_200: materialSymbols.materialSymbols20_200_0_200.font

        property string hamburger: '\ue5d2'
        property string home: '\ue88a'
        property string settings: '\ue8b8'
        property string play: '\uf04b'
        property string stop: '\uf04d'
        property string halt: '\uf256'
        property string gamepad: '\uf11b'

    }


}
