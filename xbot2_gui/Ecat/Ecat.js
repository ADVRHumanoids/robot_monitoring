function construct() {
    updateIds()
}

function updateIds() {

    client.doRequestAsync('GET', '/ecat/get_slave_list', '')
        .then(msg => allId = ['ALL'].concat(msg.id))

    client.doRequestAsync('GET', '/ecat/get_cmd_list', '')
        .then(msg => sdoCmds = msg.cmd)

}

function updateSdos() {

    if(selectedId.length === 0) {
        return
    }

    client.doRequestAsync('GET', `/ecat/get_sdo_list?id=${selectedId.join(',')}`, '')
        .then(msg => allSdo = msg.sdo)

}

function readSdo(id, sdo) {

    let setSdoValues = function(msg) {
        for (const [id, sdoDict] of Object.entries(msg.sdo)) {
            if (!sdoValues[id]) {
              sdoValues[id] = {};
            }
            sdoValues[id][sdo] = sdoDict[sdo]
        }
        root.sdoValuesChanged()
    }

    client.doRequestAsync('GET', `/ecat/read_sdo?id=${id.join(',')}&sdo=${sdo}`, '')
        .then(setSdoValues)
}

function writeSdo(id, sdo, value) {

    client.doRequestAsync('POST', `/ecat/write_sdo?id=${id.join(',')}&sdo=${sdo}&value=${value}`, '')
        .then((msg) => readSdo(id, sdo))
}

function sdoCmd(id, cmd) {
    client.doRequestAsync('POST', `/ecat/write_sdo?id=${id.join(',')}&cmd=${cmd}`, '')
        .then((msg) => console.log(JSON.stringify(msg)))
}
