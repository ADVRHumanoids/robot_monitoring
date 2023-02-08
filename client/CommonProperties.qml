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
        property int cardRadius: 4
        property int spacing: 16
    }

    property Item font: Item {
        property int h1: 28
        property int h2: 20
        property int h3: 16
        property int h4: 14
    }

    property Item fontAwesome: Item {
        id: fontAwesome

        readonly property FontLoader fontAwesomeSolid: FontLoader {
            source: "./Font/fontawesome-free-6.3.0-desktop/otfs/Font Awesome 6 Free-Solid-900.subset.otf"
        }

        readonly property FontLoader fontAwesomeRegular: FontLoader {
            source: "./Font/fontawesome-free-6.3.0-desktop/otfs/Font Awesome 6 Free-Regular-400.subset.otf"
        }

        property alias regular: fontAwesome.fontAwesomeRegular.font
        property alias solid: fontAwesome.fontAwesomeSolid.font

    }

}
