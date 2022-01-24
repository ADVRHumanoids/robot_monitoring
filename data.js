var fieldNames = [
            'posRef', 'motPos', 'linkPos',
            '__sep__',
            'velRef', 'motVel', 'linkVel',
            '__sep__',
            'torFfwd', 'torRef', 'tor',
            '__sep__',
            'aux',
            '__sep__',
            'motorTemp', 'driverTemp',
            '__sep__',
            'fault'
        ];


var shortToLongName = {
    'posRef': 'Position ref.',
    'motPos': 'Motor position',
    'linkPos': 'Link position',
    'velRef': 'Velocity ref.',
    'motVel': 'Motor velocity',
    'linkVel': 'Link velocity',
    'torFfwd': 'Torque feedfwd',
    'torRef': 'Torque ref.',
    'tor': 'Torque',
    'motorTemp': 'Motor temperature',
    'driverTemp': 'Driver temperature',
    'fault': 'Fault code'
};

var skipPlotBtn = [
        'fault'
        ]

function buildValuePlot (fieldName, container) {

    var valueObj = valueComponent.createObject(container)

    fieldValueMap[fieldName] = valueObj

    if(fieldName in skipPlotBtn)
    {
        return valueObj
    }

    var plotBtnObj = plotBtnComponent.createObject(container)
    plotBtnObj.implicitHeight = valueObj.implicitHeight

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
    if(msg.names[jIndex] !== jName)
    {
        jIndex = msg.names.findIndex(function(elem){ return elem === jName; });
    }

    var nFields = fieldNames.length

    for(var i = 0; i < nFields; i++)
    {
        var fName = fieldNames[i]
        var fValue = fieldValueMap[fName]
        fValue.value = msg[fName][jIndex]
    }
}
