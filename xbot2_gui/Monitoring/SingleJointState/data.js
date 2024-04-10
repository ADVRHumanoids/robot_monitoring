.import QtQuick.Layouts as L // you can't import without "as" in .JS

var fieldNames = [
            'posRef', 'motPos', 'linkPos',
            '__sep__',
            'velRef', 'motVel', 'linkVel',
            '__sep__',
            'torFfwd', 'torRef', 'tor',
            '__sep__',
            'k', 'd',
            '__sep__',
            'aux',
            '__sep__',
            'motorTemp', 'driverTemp',
            '__sep__',
            'fault'
        ];


var shortToLongName = {
    'posRef': 'Position Ref',
    'motPos': 'Motor Position',
    'linkPos': 'Link Position',
    'velRef': 'Velocity Ref',
    'motVel': 'Motor Velocity',
    'linkVel': 'Link Velocity',
    'torFfwd': 'Torque Ffwd',
    'torRef': 'Torque Ref',
    'tor': 'Torque',
    'motorTemp': 'Motor Temp',
    'driverTemp': 'Driver Temp',
    'fault': 'Fault Code',
    'k': 'Stiffness',
    'd': 'Damping',
};




var skipPlotBtn = [
            'fault'
        ]

function buildValuePlot (fieldName, plotCallback, container) {

    var valueObj = valueComponent.createObject(container)

    fieldValueMap[fieldName] = valueObj

    if(skipPlotBtn.includes(fieldName))
    {
        valueObj.L.Layout.columnSpan = 2
        valueObj.L.fillWidth = true
        return valueObj
    }

    var plotBtnObj = plotBtnComponent.createObject(container)

    plotBtnObj.released.connect(plotCallback)

    plotBtnMap[fieldName] = plotBtnObj

    return valueObj
}

function buildFields (container) {

    var nFields = fieldNames.length

    for(var i = 0; i < nFields; i++)
    {
        let obj = undefined

        if(fieldNames[i] === '__sep__')
        {
            obj = sepComponent.createObject(container)
        }

        else if(fieldNames[i] === 'aux')
        {
            var auxObj = auxSelectorComponent.createObject(container)

            obj = auxObj

            let plotCallback = function() {
                singleJointState.plotAdded(jName, auxObj.currentText)
            }

            var valueObj = buildValuePlot(fieldNames[i], plotCallback, container)

            auxObj.implicitHeight = valueObj.implicitHeight

        }

        else
        {
            let plotCallback = function() {
                singleJointState.plotAdded(jName, fieldName)
            }

            obj = labelComponent.createObject(
                        container,
                        {text: shortToLongName[fieldNames[i]]})

            buildValuePlot(fieldNames[i], plotCallback, container)
        }

    }

}

function setJointStateMessage(msg)
{
    if(msg.name[jIndex] !== jName)
    {
        jIndex = msg.name.findIndex(function(elem){ return elem === jName; });
    }

    var nFields = fieldNames.length

    for(var i = 0; i < nFields; i++)
    {
        var fName = fieldNames[i]
        var fValue = fieldValueMap[fName]

        if(fName in msg)
        {
            fValue.setValue(msg[fName][jIndex])
            continue
        }

        if(fName === 'aux' && selectedAux in msg) {
            fValue.setValue(msg[selectedAux][jIndex])
        }
    }
}
