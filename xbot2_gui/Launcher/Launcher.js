

function construct(procRepeater, pluginRepeater) {

    requestProcessUpdate(procRepeater)

    requestPluginUpdate(pluginRepeater)

}


function requestProcessUpdate(procRepeater) {

    // update process cards when available
    let onProcessListReceived = function (msg) {
        // SharedData.processInfo = msg
        procRepeater.model = msg
    }

    client.doRequest('GET', '/process/get_list', '', onProcessListReceived)
}


function processCmd(name, cmd, opt) {

    let body = {
        name: name,
        cmd: cmd,
        options: opt
    }

    client.doRequest('PUT',
                     '/process/' + name + '/command/' + cmd,
                     JSON.stringify(body),
                     function(msg){console.log(JSON.stringify(msg))})
}


function onProcMessageReceived(procRepeater, consoleItem, msg) {

    // look for process with name msg.name
    for(let i = 0; i < procRepeater.count; i++) {

        var item_i = procRepeater.itemAt(i)

        // found!
        if(item_i.processName === msg.name) {

            if(msg.content === 'status')
            {
                item_i.processState = msg.status
            }

            if(msg.content === 'output')
            {
                let prefix = '[' + item_i.processName + '] '
                if(msg.stdout.length > 0) {
                    consoleItem.appendText(prefix+msg.stdout)
                }

                if(msg.stderr.length > 0) {
                    consoleItem.appendText('<font color="red">' + prefix+msg.stderr + '</>')
                }
            }

            break
        }
    }
}


function requestPluginUpdate(pluginRepeater) {
    // create plugin cards when available
    let onPluginListReceived = function (msg) {
        // SharedData.pluginNames = msg.plugins
        pluginRepeater.model = msg.plugins
    }

    client.doRequest('GET', '/plugin/get_list', '', onPluginListReceived)
}


function pluginCmd(name, cmd) {

    client.doRequest('PUT',
                     '/plugin/' + name + '/command/' + cmd,
                     '',
                     function(msg){console.log(JSON.stringify(msg))})
}


function onPluginMessageReceived(pluginRepeater, msg) {
    for(let i = 0; i < pluginRepeater.count; i++) {
        let singlePlugin = pluginRepeater.itemAt(i)
        let pluginMsg = msg[singlePlugin.pluginName]
        singlePlugin.pluginPeriod = pluginMsg.expected_period
        singlePlugin.pluginCpuTime = pluginMsg.run_time
        singlePlugin.pluginState = pluginMsg.state
    }
}
