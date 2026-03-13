

function construct() {

    requestProcessUpdate()

    if(client.robotConnected) {
        requestPluginUpdate(pluginRepeater, true)
    }

    if(dashboard.visible) {
        dashboard.refresh()
    }

}


function requestProcessUpdate() {

    // update process cards when available
    let onProcessListReceived = function (msg) {

        // procRepeater.model = msg

        let availableMachines = []

        let availableContainers = []

        let processNames = []

        let hiddenProcessNames = []

        let categoryNames = []

        categoryToProcessModel = Object()

        for(let item of msg) {

            let category = item.category ?? 'none'

            availableMachines.push(item.machine)

            availableContainers.push(item.docker)

            processNames.push(item.name)

            categoryNames.push(category)

            if(!item.visible) {
                hiddenProcessNames.push(item.name)
            }

            categoryToProcessModel[category] = categoryToProcessModel[category] ?? []
            categoryToProcessModel[category].push(item)

            processStatusMap[item.name] = item.status

            processInfoMap[item.name] = item
        }

        categoryToProcessModelChanged()

        custoCommand.availableMachines = [... new Set(availableMachines)]

        custoCommand.availableContainers = [... new Set(availableContainers)]

        processMainRepeater.model = [... new Set(categoryNames)]

        consoleItem.hiddenProcessNames = hiddenProcessNames

        consoleItem.processNames = processNames

        console.log(`categoryNames: ${JSON.stringify(categoryNames)}`)
        console.log(`categoryToProcessModel: ${JSON.stringify(categoryToProcessModel)}`)


    }

    client.doRequest('GET', '/process/get_list', '', onProcessListReceived)
}

function addCustomProcess(name, machine, container, cmd, visible, edit, prevName) {

    client.doRequestAsync('PUT', '/process/add_custom_command',
                          JSON.stringify(
                              {
                                  'process': name,
                                  'machine': machine,
                                  'docker': container,
                                  'cmd': cmd,
                                  'show_ui': visible,
                                  'edit': edit,
                                  'previous_name': prevName
                              })
                          )
    .then((res) => {
              requestProcessUpdate()
          })
    .catch((err) => {})

}

function deleteCustomProcess(name) {

    client.doRequestAsync('PUT', '/process/delete_custom_command/' + name, '')
    .then((res) => {
              requestProcessUpdate()
          })
    .catch((err) => {})

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
        consoleItem.appendText(msg.name, msg.out, muted)
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
    processStatusTimer.restart()
}


function requestPluginUpdate(pluginRepeater, quiet = false) {
    // create plugin cards when available
    let onPluginListReceived = function (msg) {
        // SharedData.pluginNames = msg.plugins
        pluginRepeater.model = msg.plugins
        pluginStack.currentIndex = 1
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
