function construct() {

    root._grid.children = []

    for (const [key, value] of Object.entries(root.description)) {

        console.log('creating configure panel ' + key)

        var label = root.label.createObject(root._grid)
        label.text = key

        if(value.type === "check") {
            var check = root.check.createObject(root._grid)
            check.checked = value.default
        }

        if(value.type === "combo") {
            var combo = root.combo.createObject(root._grid)
            combo.model = value.options
        }


    }
}
