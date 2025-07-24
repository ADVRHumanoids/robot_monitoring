

function construct() {

    requestProcessUpdate()

    if(client.robotConnected) {
        requestPluginUpdate(pluginRepeater, true)
    }

}


function requestProcessUpdate() {

    // update process cards when available
    let onProcessListReceived = function (msg) {

        // procRepeater.model = msg

        let availableMachines = []

        let processNames = []

        let hiddenProcessNames = []

        let categoryNames = []

        for(let item of msg) {
            let category = item.category ?? 'none'
            availableMachines.push(item.machine)
            processNames.push(item.name)
            categoryNames.push(category)
            if(!item.visible) {
                hiddenProcessNames.push(item.name)
            }

            categoryToProcessModel[category] = categoryToProcessModel[category] ?? []
            categoryToProcessModel[category].push(item)

            processStatusMap[item.name] = item.status
        }

        console.log("categoryToProcessModel:")
        console.log(JSON.stringify(categoryToProcessModel))

        // customCmd.availableMachines = [... new Set(availableMachines)]

        processMainRepeater.model = [... new Set(categoryNames)]

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


function onProcessOutputReceived(consoleItem, msg) {

    let muted = root.processMutedState[msg.name]

    // handle output
    let prefix = '[' + msg.name + '] '

    if(msg.out.length > 0) {
        consoleItem.appendText(msg.name, prefix + msg.out, muted)
    }

    if(msg.err.length > 0) {
        consoleItem.appendText(msg.name, '<font color="red">' + prefix + msg.err + '</>', muted)
        root.numErrors += 1
    }

}

function onProcessStatusReceived(msg) {

    // handle status
    processStatusMap[msg.name] = msg.status
    processStatusMapChanged()
}


function requestPluginUpdate(pluginRepeater, quiet = false) {
    // create plugin cards when available
    let onPluginListReceived = function (msg) {
        // SharedData.pluginNames = msg.plugins
        pluginRepeater.model = msg.plugins
    }

    client.doRequest('GET', '/plugin/get_list', '', onPluginListReceived, quiet)
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
