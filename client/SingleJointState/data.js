.import QtQuick.Layouts 1.3 as L // you can't import without "as" in .JS

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

function buildValuePlot (fieldName, container) {

    var valueObj = valueComponent.createObject(container)

    fieldValueMap[fieldName] = valueObj

    if(skipPlotBtn.includes(fieldName))
    {
        valueObj.L.Layout.columnSpan = 2
        valueObj.L.fillWidth = true
        return valueObj
    }

    var plotBtnObj = plotBtnComponent.createObject(container)

    plotBtnObj.released.connect( function() {
        if(plotBtnObj.checked) {
            singleJointState.plotAdded(jName, fieldName)
        }
        else {
            singleJointState.plotRemoved(jName, fieldName)
        }
    })

    plotBtnMap[fieldName] = plotBtnObj

    return valueObj
}

function buildFields (container) {

    var nFields = fieldNames.length

    for(var i = 0; i < nFields; i++)
    {
        if(fieldNames[i] === '__sep__')
        {
            sepComponent.createObject(container)
        }

        else if(fieldNames[i] === 'aux')
        {
            var auxObj = auxSelectorComponent.createObject(container)
            var valueObj = buildValuePlot(fieldNames[i], container)
            auxObj.implicitHeight = valueObj.implicitHeight

        }

        else
        {
            labelComponent.createObject(
                        container,
                        {text: shortToLongName[fieldNames[i]]})

            buildValuePlot(fieldNames[i], container)
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
            fValue.value = msg[fName][jIndex]
        }
    }
}
