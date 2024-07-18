function refresh() {

    client.doRequestAsync('GET', '/parameters/info', '')
    .then((res) => {

              repeater.model = res.name.length

              for(let i = 0; res.name.length; i++) {

                  let obj = Object()



                  obj.type = res.type[i]
                  obj.descriptor = JSON.parse(res.descriptor[i])
                  obj.name = res.name[i]
                  obj.value = JSON.parse(res.value[i])

                  console.log(JSON.stringify(obj))

                  let wrapperItem = repeater.itemAt(i)
                  let loader = wrapperItem.loader

                  wrapperItem.name = obj.name

                  if(obj.type === 'double') {
                      loader.sourceComponent = root.doubleDelegate
                      if(obj.descriptor.type === 'IsPositive') {
                          loader.item.min = 0
                          loader.item.max = 5 * obj.value
                      } else if(obj.descriptor === 'InRange') {
                          loader.item.min = obj.descriptor.min
                          loader.item.max = obj.descriptor.max
                      }
                      loader.item.setValue(obj.value)
                  }
                  else if(obj.type === 'string' &&
                          obj.descriptor.type === 'DiscreteValues') {
                      loader.sourceComponent = root.discreteValuesDelegate
                      loader.item.model = obj.descriptor.values
                      loader.item.currentIndex = loader.item.find(obj.value)
                  }
                  else if(obj.type.startsWith('Eigen::Matrix')) {
                      loader.sourceComponent = root.vectorDelegate
                      loader.item.value = obj.value.data
                  }
                  else if(obj.type === 'bool') {
                      loader.sourceComponent = root.boolDelegate
                      loader.item.checked = obj.value
                  }


              }
          })

}

function setParams() {

    let paramUpdate = Object()

    for(let i = 0; i < repeater.count; i++) {

        let item = repeater.itemAt(i)

        if(!item.checked) {
            continue
        }

        paramUpdate[item.name] = item.loader.item.value

    }

    console.log(JSON.stringify(paramUpdate))

}
