pragma Singleton

import QtQuick

Item {
    property string hamburger: '\ue5d2'
    property string home: '\ue88a'
    property string settings: '\ue8b8'
    property string joystick: '\uf5ee'
    property string controller: '\uf135'
    property string play: '\ue037'
    property string stop: '\ue047'
    property string plot: '\ue667'
    property string terminal: '\ueb8e'
    property string playground: '\uea52'
    property string done: '\ue876'
    property string cross: '\ue5cd'
    property string warning: '\ue002'
    property string error: '\ue000'
    property string vitals: '\ue650'
    property string log: '\uef42'
    property string launcher: '\ueb9b'
    property string box3d: '\uf720'
    property string analytics: '\uef3e'
    property string arrowBack: '\ue5e0'
    property string arrowForward: '\ue5e1'
    property string tableChart: '\uf6ef'
    property string gauge: '\ue9e4'
    property string replay: '\ue042'
    property string tools: '\uf10b'
    property string barchart: '\ue26b'
    property string drill: '\ue1e9'
    property string walker: '\uf8d5'
    property string wrench: '\ue869'
    property string plotSmall: '\ue922'
    property string weight: '\ue13d'
    property string dashboard: '\ue9b0'
    property string netSettings: '\ueb2f'
    property string tune: '\ue429'

    property FontLoader filledFont: FontLoader {
        source: `/Font/materialsymbols/MaterialSymbolsOutlined[opsz,wght,FILL,GRAD@20,200,1,200].otf`
    }

    property FontLoader emptyFont: FontLoader {
        source: `/Font/materialsymbols/MaterialSymbolsOutlined[opsz,wght,FILL,GRAD@20,200,0,200].otf`
    }

    property FontLoader emptyFont400: FontLoader {
        source: `/Font/materialsymbols/MaterialSymbolsOutlined[opsz,wght,FILL,GRAD@20,200,0,200].otf`
    }
}
