import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import QtCore

import Common

GridLayout {

    columns: 3
    // uniformCellHeights: true


    Label {
        topPadding: 4
        Layout.columnSpan: 3
        text: 'General'
        font.pixelSize: CommonProperties.font.h4
        font.bold: true
    }

    Label {
        Layout.columnSpan: layout.expanded ? 1 : 3
        text: 'Admin Password'
    }


    TextField {
        Layout.preferredWidth: 150
        echoMode: TextInput.Password
        id: pwdText
    }

    Button {
        text: 'Save'
        onClicked: {
            let sha256 = appData.cryptoHash(pwdText.text)
            let expectedHash = 'iDI7Myu5OI1tJl3y5HnfHLg1PGr84DHamGBx6dEYbaA='
            if(sha256 !== expectedHash) {
                CommonProperties.notifications.error('Password is not correct')
            }
            CommonProperties.config.adminPwdOk = true
            CommonProperties.notifications.info('Password correct')
        }
    }



    Label {
        topPadding: 4
        Layout.columnSpan: 3
        text: 'GUI Overlay'
        font.pixelSize: CommonProperties.font.h4
        font.bold: true
    }

    Label {
        text: 'Show Soft Emergency Stop'
    }

    Switch {
        checked: CommonProperties.config.showSoftEmergency
        onClicked: {
            CommonProperties.config.showSoftEmergency = checked
        }
    }

    Item {}

    Label {
        text: 'Show Monitoring Widget'
    }

    Switch {
        checked: CommonProperties.config.showMonWidget
        onClicked: {
            CommonProperties.config.showMonWidget = checked
        }
    }

    Item {}

    Label {
        topPadding: 4
        Layout.columnSpan: 3
        text: 'Launcher'
        font.pixelSize: CommonProperties.font.h4
        font.bold: true
    }

    Label {
        text: 'Show Dashboard'
    }

    Switch {
        checked: CommonProperties.config.showLauncherDashboard
        onClicked: {
            CommonProperties.config.showLauncherDashboard = checked
        }
    }



}
