import QtQuick 2.4
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Controls.Material

Item {

    id: root
    implicitHeight: col.implicitHeight
    implicitWidth: col.implicitWidth

    property var client: undefined
    property var model: []

    Column {
        id: col
        anchors {
            horizontalCenter: parent.horizontalCenter
            top: parent.top
        }

        Repeater {
            id: repeater
            model: root.model

            SinglePlugin {
                name: modelData
                period: 1.0
                onStart: {
                    client.doRequest('PUT',
                                     '/plugin/' + name + '/start',
                                     '',
                                     function(){})
                }
                onStop: {
                    client.doRequest('PUT',
                                     '/plugin/' + name + '/stop',
                                     '',
                                     function(){})

                }
                onAbort: {
                    client.doRequest('PUT',
                                     '/plugin/' + name + '/abort',
                                     '',
                                     function(){})
                }
            }
        }

        Component.onCompleted: {
            root.client.pluginStatMessageReceived.connect(function(msg){
                for(let i = 0; i < repeater.count; i++) {
                    let singlePlugin = repeater.itemAt(i)
                    let pluginMsg = msg[singlePlugin.name]
                    singlePlugin.period = pluginMsg.expected_period
                    singlePlugin.cputime = pluginMsg.run_time
                    singlePlugin.state = pluginMsg.state
                }
            }
            )
        }

    }

}
