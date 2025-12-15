import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import Qt.labs.settings

import Common
import LivePlot
import Main
import Font

MultiPaneResponsiveLayout {


    property ClientEndpoint client: undefined

    Connections {

        target: client

        function onError(msg) {
            root.setError(msg)
        }

        function onConnected(msg) {
            root.setConnected(msg)
        }

        function onFinalized() {
            print('finalized!')
        }
    }

    Connections {

        target: CommonProperties.notifications

        function onNewInfo(txt, name) {
            notiPopup.addMsg(txt, name)
        }

        function onNewWarning(txt, name) {
            notiPopup.addMsg(txt, name, 1)
            root.numErrors += 1
        }

        function onNewError(txt, name) {
            notiPopup.addMsg(txt, name, 2)
            root.numErrors += 1
        }
    }

    property int numEvents: 0
    property int numErrors: 0

    signal restartUi()

    function setError(msg) {
        srvStatus.msgText.text = "Error: " + msg
        srvStatus.msgText.color = CommonProperties.colors.err
    }

    function setConnected(msg) {
        srvStatus.msgText.text = "Status OK: " + msg
        srvStatus.msgText.color = CommonProperties.colors.ok
    }

    function setProgress(msg) {
        statusText.text = msg
    }


    // private
    id: root



    MaterialSymbols {
        id: syms
    }



    Drawer {
        id: configPopup
        edge: Qt.BottomEdge

        width: parent.width
        height: parent.height * 0.9
        interactive: true
        Configuration {
            anchors.fill: parent
            onClosed: configPopup.close()
        }
        modal: true
        focus: true
        closePolicy: Popup.CloseOnEscape | Popup.CloseOnPressOutside
        padding: layout.expanded ? 16 : 4
        leftPadding: 16
        rightPadding: 16
        clip: true
    }


    ScrollView {

        property string iconText: 'Status'
        property string iconChar: MaterialSymbolNames.vitals

        id: scroll
        contentWidth: availableWidth

        Column {

            id: leftCol
            spacing: CommonProperties.geom.spacing
            width: scroll.contentWidth

            SectionHeader {
                text: `XBot2 GUI ${appData.version.join('.')}`
                SmallToolButton {
                    text: MaterialSymbolNames.settings
                    font.family: syms.font.family
                    font.pixelSize: 24
                    onClicked: configPopup.open()
                }
                width: leftCol.width
            }

            ServerStatusCard {
                id: srvStatus
                client: root.client
                width: scroll.availableWidth
            }

            StatisticsCard {
                client: root.client
                width: scroll.availableWidth

            }

        }
    }

    ColumnLayout {

        property string iconText: 'Server log'
        property string iconChar: MaterialSymbolNames.log

        id: textCol
        spacing: CommonProperties.geom.spacing

        SectionHeader {
            text: parent.iconText
            Layout.fillWidth: true
        }

        Row {

            CheckBox {
                id: verbosityCheck
                text: 'Verbose'
                checked: false
            }

            CheckBox {
                id: autoscrollCheck
                text: 'Autoscroll'
                checked: true
            }

        }

        NotificationPopup {
            id: notiPopup
            verbosity: verbosityCheck.checked ? 0 : 1
            autoscroll: autoscrollCheck.checked
            Layout.fillHeight: true
            Layout.fillWidth: true
            onDismissRequested: clear()
        }

    }

}
