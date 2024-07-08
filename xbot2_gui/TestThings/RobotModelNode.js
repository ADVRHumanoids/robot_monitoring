function makeModel(linkToUri) {

    let model = []

    for (const [key, value] of Object.entries(linkToUri)) {
        console.log(`${key}: ${JSON.stringify(value)}`);
        let obj = {}
        obj.linkName = key
        obj.filename = value.filename
        obj.scale = Qt.vector3d(value.scale[0]*100,
                                value.scale[1]*100,
                                value.scale[2]*100)
        obj.type = value.type
        obj.radius = value.radius
        obj.length = value.length
        obj.origin_xyz = Qt.vector3d(value.origin_xyz[0],
                                     value.origin_xyz[1],
                                     value.origin_xyz[2])
        obj.origin_rot = Qt.quaternion(value.origin_rot[3],
                                       value.origin_rot[0],
                                       value.origin_rot[1],
                                       value.origin_rot[2])
        model.push(obj)
    }

    return model
}


function updateTf() {
    client.doRequestAsync('GET', '/visual/get_mesh_tfs')
    .then((msg) =>
          {
              for(let i = 0; i < visualRepeater.count; i++) {
                  let obj = visualRepeater.objectAt(i)
                  let linkName = visualRepeater.model[i].linkName
                  let tf = msg[linkName]
                  obj.position = Qt.vector3d(tf[0][0]*100,
                                                tf[0][1]*100,
                                                tf[0][2]*100)
                  obj.rotation = Qt.quaternion(tf[1][3], tf[1][0], tf[1][1], tf[1][2])
              }
          })
    .catch((err) => { console.error(err) })
}


function getPose(model, linkName) {

    let pose = model.getPose(linkName)

    return {
        'translation': Qt.vector3d(pose.translation[0],
                                   pose.translation[1],
                                   pose.translation[2]),
        'rotation': Qt.quaternion(pose.rotation[0],
                                  pose.rotation[1],
                                  pose.rotation[2],
                                  pose.rotation[3],)
    }
}

function updateQ(q) {

    model.setJointPosition(q)

    for(let i = 0; i < visualRepeater.count; i++) {

        let obj = visualRepeater.objectAt(i)

        let linkName = visualRepeater.model[i].linkName || ''

        let pose = model.getPose(linkName)

        obj.position = Qt.vector3d(pose.translation[0]*100,
                                      pose.translation[1]*100,
                                      pose.translation[2]*100)

        obj.rotation = Qt.quaternion(pose.rotation[0],
                                     pose.rotation[1],
                                     pose.rotation[2],
                                     pose.rotation[3],)
    }
}


function createViewer() {

    // clear mesh repeater model
    visualRepeater.model = 0

    // get mesh and then robot
    client.doRequestAsync('GET', '/visual/get_mesh_entities', '')
    .then((response) =>
          {
              visualRepeater.model = makeModel(response)

              return client.doRequestAsync('GET', '/joint_states/urdf', '')
          })
    .then((response) =>
          {
              model.setUrdf(response.urdf, false)
          })

    .catch((err) => { console.error(err) })
}
