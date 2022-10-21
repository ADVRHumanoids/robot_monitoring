import QtQuick 2.4
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Controls.Material

Item {

    id: root
    height: col.implicitHeight

    property var client: undefined
    property var model: []

    Column {
        id: col
        anchors.fill: parent

        Repeater {
            id: repeater
            model: root.model

            SinglePlugin {
                name: modelData
                period: 1.0
                width: col.width
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
