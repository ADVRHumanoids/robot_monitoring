

function construct(procRepeater, pluginRepeater) {

    requestProcessUpdate(procRepeater)

    requestPluginUpdate(pluginRepeater)

}


function requestProcessUpdate(procRepeater) {

    // update process cards when available
    let onProcessListReceived = function (msg) {

        procRepeater.model = msg

        let availableMachines = []

        let processNames = []

        let hiddenProcessNames = []

        for(let item of msg) {
            availableMachines.push(item.machine)
            processNames.push(item.name)
            if(!item.visible) {
                hiddenProcessNames.push(item.name)
            }
        }

        customCmd.availableMachines = [... new Set(availableMachines)]

        consoleItem.hiddenProcessNames = hiddenProcessNames

        consoleItem.processNames = processNames

        console.log(hiddenProcessNames)

    }

    client.doRequest('GET', '/process/get_list', '', onProcessListReceived)
}


function customCommand(machine, command, timeout) {
    client.doRequestAsync('POST', '/process/custom_command',
                          JSON.stringify(
                              {
                                  'machine': machine,
                                  'command': command,
                                  'timeout': timeout
                              })
                          )
    .then((res) => {
              customCmd.setResult(res.retcode, res.stdout, res.stderr)
          })
    .catch((err) => {})
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

    // handle output
    if(msg.content === 'output')
    {
        let prefix = '[' + msg.name + '] '

        if(msg.stdout.length > 0) {
            consoleItem.appendText(msg.name, prefix + msg.stdout)
        }

        if(msg.stderr.length > 0) {
            consoleItem.appendText(msg.name, '<font color="red">' + prefix + msg.stderr + '</>')
            root.numErrors += 1
        }

        return;
    }

    // handle status
    for(let i = 0; i < procRepeater.count; i++) {

        var item_i = procRepeater.itemAt(i)

        // found!
        if(item_i.processName === msg.name) {

            if(msg.content === 'status')
            {
                item_i.processState = msg.status
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
        if(singlePlugin.pluginState !== pluginMsg.state) {
            root.numErrors += 1
        }
        singlePlugin.pluginState = pluginMsg.state
    }
}
