import QtQuick
import QtQuick.Layouts
import QtQuick.Controls

import Audio

GridLayout {

    signal closed()

    id: root

    columns: 1

    TabBar {
        id: bar
        Layout.fillWidth: true
        Layout.fillHeight: false

        TabButton {
            text: 'General'
        }

        TabButton {
            text: 'Audio'
        }
    }

    StackLayout {

        currentIndex: bar.currentIndex
        Layout.fillWidth: true
        Layout.fillHeight: true

        ConfigurationGeneral {
            Layout.fillWidth: true
            Layout.fillHeight: true
        }

        ConfigurationAudio {
            Layout.fillWidth: true
            Layout.fillHeight: true
        }
    }

    Button {
        text: 'Close'
        onClicked: root.closed()
        Layout.alignment: Qt.AlignRight
    }
}

