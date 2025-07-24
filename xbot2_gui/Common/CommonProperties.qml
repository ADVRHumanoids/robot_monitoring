pragma Singleton

import QtQuick
import QtQuick.Controls.Material
import QtCore

import Joy

Item {

    property Item colors: Item {
        property color primary: Material.primaryColor
        property color accent: Material.accentColor
        property color ok: Material.color(Material.Green, Material.Shade900)
        property color warn: Material.color(Material.Yellow, Material.Shade900)
        property color err: Material.color(Material.Red, Material.Shade300)
        property color primaryText: Material.primaryTextColor
        property color secondaryText: Material.secondaryTextColor
        property color cardBackground: Qt.rgba(1, 1, 1, 0.075)
        property color frame: Material.frameColor
    }

    property Item geom: Item {
        property int mobileBreakpoint: 576
        property int cardRadius: 4
        property int spacing: 8
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

    property Item notifications: Item {

        function info(txt, name = 'ui') {
            console.log(`[info][${name}] ${txt}`)
            newInfo(txt, name)
        }

        function warning(txt, name = 'ui') {
            console.log(`[warning][${name}] ${txt}`)
            newWarning(txt, name)
        }

        function error(txt, name = 'ui') {
            console.log(`[error][${name}] ${txt}`)
            newError(txt, name)
        }

        function message(txt, name = 'ui', severity = 0) {
            if(severity === 0) newInfo(txt, name)
            else if(severity === 1) newWarning(txt, name)
            else if(severity === 2) newError(txt, name)
        }

        signal newInfo(string txt, string name)

        signal newWarning(string txt, string name)

        signal newError(string txt, string name)

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

    property Item globalLivePlot

    property Item config: Item {
        id: config
        property bool testing: true
        property bool showSoftEmergency: false
        property bool showMonWidget: false
        property bool showLauncherDashboard: false
        property bool adminPwdOk: false
    }

    property Item gamepad: Item {

        id: gamepad

        function registerGamepad(name) {

            if(_gamepadNames.indexOf(name) === -1) {
                console.log(`Registering gamepad: ${name}`)
                _gamepadNames.push(name)
            }
            else {
                console.warn(`Gamepad name "${name}" already registered.`)

            }

            return gamepadRepeater.itemAt(_gamepadNames.indexOf(name))

        }

        function getEnabledGamepad() {
            for(let i = 0; i < gamepadRepeater.count; i++) {
                let g = gamepadRepeater.itemAt(i)
                if(g.enabled) {
                    return g
                }
            }
            return null
        }

        property list<string> _gamepadNames: []

        Repeater {
            id: gamepadRepeater
            model: gamepad._gamepadNames
            delegate: GamepadInterface {
                id: gamepadIfc
                required property string modelData
                property alias name: gamepadIfc.modelData
                enabled: false

                onEnabledChanged: {
                    if(!enabled) {
                        console.log(`Gamepad "${modelData}" disabled.`)
                        return
                    }

                    console.log(`Gamepad "${modelData}" enabled.`)

                    for(let i = 0; i < gamepadRepeater.count; i++) {
                        if(gamepadRepeater.itemAt(i).name !== name) {
                            gamepadRepeater.itemAt(i).enabled = false
                        }
                    }
                }
            }

            onItemAdded: function(item) {
                console.log(`Gamepad "${item.modelData}" added to repeater.`)
            }
        }
    }

    Settings {
        category: 'configuration_general'
        property alias showSoftEmergency: config.showSoftEmergency
        property alias showMonWidget: config.showMonWidget
    }

}
