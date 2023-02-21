function makeModel(linkToUri) {

    let model = []

    for (const [key, value] of Object.entries(linkToUri)) {
        console.log(`${key}: ${value}`);
        let obj = {}
        obj.linkName = key
        obj.uri = value[0]
        obj.scale = value[1]
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
                  obj.translation = Qt.vector3d(tf[0][0], tf[0][1], tf[0][2])
                  obj.rotation = Qt.quaternion(tf[1][3], tf[1][0], tf[1][1], tf[1][2])
              }
          })
    .catch((err) => { console.error(err) })
}


function updateModel() {
    visualRepeater.model = 0
    client.doRequestAsync('GET', '/visual/get_mesh_entities', '')
        .then((msg) => {
              visualRepeater.model = Logic.makeModel(msg)
          })
        .then(() => {updateTf()})
        .catch((err) => { console.error(err) })
}
