import QtQuick
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

    property ClientEndpoint client
    property bool isCurrentPage

    id: root

    // lazy-loading of active page
    property Component pageLoader: Loader {

        id: stackPageLoader
        property string pageName: ''

        SplitView.fillHeight: true
        SplitView.fillWidth: true
        SplitView.preferredHeight: modelData.pageHeight
        active: true

        onLoaded: {

            console.log(`${modelData.name} loaded`)

            // items[modelData.name.toLowerCase()] = item

            try {
                item.pageSelected()
            }
            catch(err){}

            try {
                item.isCurrentPage = Qt.binding(() => root.isCurrentPage)
            }
            catch(err){}

            item.pageName = modelData.name

            pageName = modelData.name

        }

        Component.onCompleted: {
            // this is the "constructor"
            // each page has a .client elem
            setSource(modelData.page, {'client': root.client})
        }

    }

    SplitView {

        anchors.fill: parent

        SplitView {
            // left split
            SplitView.preferredWidth: 2/3 * root.width
            orientation: Qt.Vertical
            Repeater {


                delegate: pageLoader

                model: [
                    {name: 'Launcher', page: '/qt/qml/Launcher/Launcher.qml', pageHeight: root.height/3*2},
                    {name: 'Joy', page: '/qt/qml/Joy/Joy.qml', pageHeight: root.height/3}
                ]

            }

        }

        SplitView {
            // right split
            orientation: Qt.Vertical
            SplitView.preferredWidth: 1/3 * root.width
            Repeater {



                delegate: pageLoader

                model: [
                    {name: 'Monitoring', page: '/qt/qml/Monitoring/Monitoring.qml', pageHeight: root.width/3}
                ]

            }
        }



    }
}
