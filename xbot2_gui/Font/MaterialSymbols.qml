import QtQuick

QtObject {

    property bool filled: false

    property int weight: 200

    property font font: filled ? filledFont.font : emptyFont.font

    property string hamburger: '\ue5d2'
    property string home: '\ue88a'
    property string settings: '\ue8b8'
    property string joystick: '\uf5ee'
    property string controller: '\uf135'
    property string play: '\ue037'
    property string stop: '\ue047'
    property string plot: '\ue667'
    property string terminal: '\ueb8e'

    //
    id: root

    property FontLoader filledFont: FontLoader {
        source: `/Font/materialsymbols/MaterialSymbolsOutlined[opsz,wght,FILL,GRAD@20,${root.weight},1,200].otf`
    }

    property FontLoader emptyFont: FontLoader {
        source: `/Font/materialsymbols/MaterialSymbolsOutlined[opsz,wght,FILL,GRAD@20,${root.weight},0,200].otf`
    }

    // to get more variants:
    // (1) https://fonts.googleapis.com/css2?family=Material+Symbols+Outlined:opsz,wght,FILL,GRAD@20,200,0,200
    // (2) change the last four values
    // (3) convert the wotf2 to otf

}
