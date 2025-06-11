function construct() {
    client.jointStateReceived.connect(jsCallback)
}

function startScanning(angle) {
    let radians = angle * 3.14/180
    console.log("Covering ", radians)
    client.doRequestAsync('POST', '/sanding/start_scanning', JSON.stringify(radians))
        .then((response) => {
                console.log('Scan Completed')
              })
}

function upload(parameters) {
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
              client.doRequestAsync('POST', '/sanding/approach_wall',
                                    JSON.stringify(parameters.ID))
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
                                      wall.pose.position.y*100 ,
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

    // root.updateConcertState(js,
    //                         concertModel,
    //                         'linkPos')
    concertModel.q = updateViewerQ(js,
                                   concertModel.jointNames,
                                   'linkPos',
                                   [...concertModel.q])
}


function updateViewerQ(js, jointNames, fieldName, q) {

    for(let i = 0; i < js.name.length; i++) {

        let name = js.name[i]
        let idx = jointNames.indexOf(name)
        if(idx < 0)
        {
            continue
        }

        q[idx] = js[fieldName][i]

    }

    return q
}
