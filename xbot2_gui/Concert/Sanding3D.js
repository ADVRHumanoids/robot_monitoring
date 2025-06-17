.import "/qt/qml/Main/sharedData.js" as SharedData

function construct() {
    client.jointStateReceived.connect(jsCallback)
}

function startScanning(angle) {
    let radians = angle * 3.14/180
    console.log("Covering ", radians)
    currentStatus = "Scanning..."
    client.doRequestAsync('POST', '/sanding/start_scanning', JSON.stringify(radians))
        .then((response) => {
                console.log('Scan Completed')
                currentStatus = "Scan Completed"
              })
}

function upload(parameters) {
    // Function starts after sendButton
    console.log("Width: ", parameters.width)
    console.log("Height: ", parameters.height)
    console.log("Velocity: ", parameters.velocity)
    console.log("Force: ", parameters.force)
    console.log("ID", parameters.ID)
    console.log("x: ", parameters.x)
    console.log("y: ", parameters.y)
    console.log("index: ", parameters.index)

    client.doRequestAsync('POST', '/sanding/upload_params',
                          JSON.stringify(parameters))
    .then((response) => {
        console.log("Ros Parameter Loaded")
              currentStatus = "Approaching Wall..."
              client.doRequestAsync('POST', '/sanding/approach_wall',
                                    JSON.stringify(parameters.ID))
              .then((response) => {
                        console.log("Approach Wall Completed")
                        startSendingButton.readyToSand = true
                        currentStatus = "Ready to Sand"
                        console.log("Finished")
                        windowSettings.source = ""
              })
    })
}

function startMission() {
    startSendingButton.readyToSand = false
    currentStatus = "Preparing for Sanding..."
    client.doRequestAsync('PUT', '/process/sanding/command/start',
                          '').then((response) => {

                                   })
}

function startTool() {
    client.doRequestAsync('POST', '/concert/sanding/tool_started_ack', '')
    .then((response) => {

          })
}

function updateConcertPose(input) {
    mapPosition = Qt.vector3d(input.position.x*100,
                               (input.position.y + 0.75)*100,
                               input.position.z*100)

    mapOrientation = Qt.quaternion(input.orientation.w, input.orientation.x, input.orientation.y, input.orientation.z )
}

function toQMLObject(wall) {
    // todo adjust also
    wall.pose.position  = Qt.vector3d(wall.pose.position.x*100,
                                      (wall.pose.position.y+0.75)*100,
                                      wall.pose.position.z*100 )

    wall.pose.orientation = Qt.quaternion(wall.pose.orientation.w, wall.pose.orientation.x, wall.pose.orientation.y, wall.pose.orientation.z)
    return {
        "id": wall.id,
        "position" : wall.pose.position,
        "orientation": wall.pose.orientation,
        "l": wall.length,
        "index": wall.index
    }
}

function jsCallback(js) {
    concertModel.q = updateViewerQ(js,
                                   concertModel.jointNames,
                                   'motPos',
                                   [...concertModel.q])
}


function updateViewerQ(js, jointNames, fieldName, q) {

    let jsname = SharedData.jointNames
    for(let i = 0; i < jsname.length; i++) {

        let name = jsname[i]
        let idx = jointNames.indexOf(name)
        if(idx < 0)
        {
            continue
        }

        q[idx] = js[fieldName][i]

    }
    // console.log("Updating Q")
    return q
}
