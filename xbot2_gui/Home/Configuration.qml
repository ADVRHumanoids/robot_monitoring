import QtQuick
import QtQuick.Layouts
import QtQuick.Controls

import Audio

GridLayout {

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
}

