function changeState(self, state) {

    log.clear()

    self.activationInProgress = true

    client.doRequestAsync('POST', `/dashboard/${state}/start`, '')
        .then(function(res) {
            let prefix = res.success ? '[status] ' : '[fail] '
            log.append(prefix + res.message)
            self.activationInProgress = false
        })

}

function toTitleCase(phrase) {
  return phrase
    .toLowerCase()
    .split(' ')
    .map(word => word.charAt(0).toUpperCase() + word.slice(1))
    .join(' ');
};

function triggerSafety() {
    client.doRequest('POST',
                     '/joint/safety/set_enabled?enabled=false',
                     '')
}

function startRobot() {

    log.clear()

    client.doRequestAsync('POST', `/dashboard/robot_switch/start`, '')
        .then(function(res) {
            let prefix = res.success ? '[status] ' : '[fail] '
            log.append(prefix + res.message)
        })
}

function stopRobot() {

    log.clear()

    client.doRequestAsync('POST', `/dashboard/robot_switch/stop`, '')
        .then(function(res) {
            let prefix = res.success ? '[status] ' : '[fail] '
            log.append(prefix + res.message)
        })
}
