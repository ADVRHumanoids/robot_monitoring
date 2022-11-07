import QtQuick
import QtQuick.Layouts
import QtQuick.Controls.Material


Rectangle {

    property var client: undefined
    color: "green"

    ScrollView {

        id: scroll
        anchors.fill: parent
        contentWidth: availableWidth
        contentHeight: grid.height

        MaterialResponsiveGrid {

            id: grid
            width: scroll.contentWidth

//            PluginStatus {

//            }

            ProcessStatus {

            }

            ProcessStatus {

            }

            ProcessStatus {

            }

            ProcessStatus {

            }

            ProcessStatus {

            }

            Rectangle {
                property int columnSpan: grid.columns
                height: 300
            }

        }
    }
}
