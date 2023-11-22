import QtQuick
import QtCore
import QtQuick.Controls
import QtQuick.Layouts
import QtWebView

import Common
import Main
import ExpandableBottomBar
import Font
import Menu
import Joy

Item {

    property string pageName

    SwipeView {

        anchors.fill: parent

    }

    WebView {
        id: web
        anchors {
            top: parent.top
            left: parent.left
            right: parent.right
            bottom: parent.bottom
            margins: 8
        }

        Layout.fillHeight: true
        Layout.fillWidth: true
        Layout.preferredHeight: 1
        Layout.preferredWidth: 1
        url: appData.fromUserInput('iit.it')
        visible: Qt.platform.os == "android" ? !popup.opened : true

        onLoadingChanged: {

            console.log(loadRequest.url)

            if(loadRequest.status === WebView.LoadStartedStatus) {
                console.log('LoadStartedStatus')
            }

            if(loadRequest.status === WebView.LoadSucceededStatus) {
                console.log('LoadSucceededStatus')
            }

            if(loadRequest.status === WebView.LoadFailedStatus) {
                console.log('LoadFailedStatus ', loadRequest.errorString)
                CommonProperties.notifications.error(`URL ${loadRequest.url} error: ${loadRequest.errorString}`, 'webview')
            }


        }

    }


    Control {

        id: ctrl

        z: 1

        anchors {
            top: parent.top
            right: parent.left
            margins: 8
        }

        padding: 4

        background: Rectangle {
            color: palette.window
            radius: 4
        }

        contentItem: Column {
            spacing: 4
            ToolButton {
                text: MaterialSymbolNames.replay
                font.family: MaterialSymbolNames.emptyFont400.font.family
                font.pointSize: 18
                onClicked: web.reload()
            }
            ToolButton {
                text: MaterialSymbolNames.settings
                font.family: MaterialSymbolNames.emptyFont400.font.family
                font.pointSize: 18
                onClicked: popup.open()
            }
        }
    }

    Popup {
        id: popup
        anchors.centerIn: parent
        width: Math.min(parent.width, 600)
        //        height: Math.min(parent.width, 400)
        modal: true
        focus: true
        closePolicy: Popup.CloseOnEscape | Popup.CloseOnPressOutside
        padding: 24

        Column {

            width: parent.width

            RowLayout {

                spacing: 6
                width: parent.width

                TextField {
                    placeholderText: 'url'
                    inputMethodHints: Qt.ImhUrlCharactersOnly | Qt.ImhPreferLowercase
                    text: web.url
                    onAccepted: web.url = appData.fromUserInput(text)
                    Layout.fillWidth: true
                }

                Button {
                    text: 'Reload'
                    onClicked: web.reload()
                }
            }

            Item {
                width: parent.width
                height: 16
            }

            Button {
                anchors.right: parent.right
                text: 'Close'
                onClicked: popup.close()
            }

        }

    }

    Settings {
        id: settings
        category: pageName
        property alias webUrl: web.url
    }

}

