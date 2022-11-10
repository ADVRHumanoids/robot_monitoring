import QtQuick
import QtQuick.Layouts
import QtQuick.Controls.Material


Item {

    TabBar {
        id: bar
        width: parent.width
        TabButton {
            text: 'Process'
        }
        TabButton {
            text: 'Plugin'
        }
        y: mobileLayout ? 0 : -height
    }

    id: root
    property var client: undefined
    property bool mobileLayout: width < gridLeft.brSmall

    onWidthChanged: {
        if(mobileLayout) {
            mainRow.children = []
            mainSwipe.contentChildren = [scrollLeft, scrollRight]
        }
        else {
            mainSwipe.contentChildren = []
            mainRow.children = [scrollLeft, scrollRight]
        }
    }

    SwipeView {
        id: mainSwipe
        width: parent.width
        anchors {
            top: bar.bottom
            bottom: parent.bottom
        }

        clip: true
        currentIndex: bar.currentIndex
    }

    RowLayout {

        id: mainRow
        width: parent.width
        anchors {
            top: bar.bottom
            bottom: parent.bottom
        }

        ScrollView {

            id: scrollLeft

            Layout.fillWidth: true
            Layout.fillHeight: true

            contentWidth: availableWidth
            contentHeight: gridLeft.height

            MaterialResponsiveGrid {

                id: gridLeft
                width: scrollLeft.contentWidth

                Label {
                    property int columnSpan: parent.columns
                    text: "Process"
                    font.pixelSize: 40
                }

                Repeater{
                    model: 5

                    ProcessStatus {
                    }

                }

                Label {
                    property int columnSpan: parent.columns
                    text: "Console output"
                    font.pixelSize: 40
                }

                Console {
                    property int columnSpan: parent.columns
                }

            }

        }

        ScrollView {

            id: scrollRight

            Layout.fillWidth: true
            Layout.fillHeight: true

            contentWidth: availableWidth
            contentHeight: gridRight.height

            MaterialResponsiveGrid {

                id: gridRight
                width: scrollRight.contentWidth
                sizeid: gridLeft.sizeid

                Label {
                    property int columnSpan: parent.columns
                    text: "Plugin"
                    font.pixelSize: 40
                }

                PluginStatus {

                }
                PluginStatus {

                }
                PluginStatus {

                }
                PluginStatus {

                }

            }

        }
    }
}
