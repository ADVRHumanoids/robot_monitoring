function construct() {

    root._grid.children = []

    for (const [key, value] of Object.entries(root.description)) {

        console.log('creating configure panel ' + key)

        var label = root.label.createObject(root._grid)
        label.text = key

        var control = undefined

        if(value.type === "check") {
            control = root.check.createObject(root._grid)
            control.checked = value.default
        }

        if(value.type === "combo") {
            control = root.combo.createObject(root._grid)
            control.model = value.options
        }

        if(value.type === "text") {
            control = root.text.createObject(root._grid)
            control.placeholderText = value.help
        }

        root._controls[key] = control
    }

    root._cancelBtn.parent = root._grid
    root._okBtn.parent = root._grid
}

function apply() {
    for (const [key, value] of Object.entries(root.description)) {

        var control = root._controls[key]

        if(value.type === "check") {
            root.options[key] = control.checked
        }

        if(value.type === "combo") {
            root.options[key] = control.currentText
        }

        if(value.type === "text") {
            root.options[key] = control.text
        }
    }

    console.log(JSON.stringify(root.options))
}
