.import "/qt/qml/Main/sharedData.js" as SharedData
.import "../SingleJointState/data.js" as SjsData

// base value
function valueBase() {

    if(min > 0)
    {
        return min
    }

    if(max < 0)
    {
        return max
    }

    return 0
}

// normalized value
function normalize(value) {

    return (value - min)/(max - min)
}


function widthNormalized() {

    return (value - valueBase())/(max - min)
}


function xOffset() {

    var baseOffset = (valueBase() - min)/(max - min) * bar.parent.width

    if(widthNormalized() < 0)
    {
        baseOffset += widthNormalized() * bar.parent.width
    }

    return baseOffset
}


function width() {
    return Math.max(2, (bar.parent.width - 2 * barMargin) * Math.abs(widthNormalized()))
}


function setJointStateMessage(msg)
{
    let fieldMsg = msg[fieldName]

    if(fieldMsg === undefined) {
        console.log(`no field ${fieldName} in js msg`)
        return
    }

    let fieldRefMsg = undefined

    if(fieldNameRef !== '') {
        fieldRefMsg = msg[fieldNameRef]
        if(fieldRefMsg === undefined) {
            console.log(`no field ${fieldName} in js msg`)
            return
        }
    }

    for(var i = 0; i < jointNames.length; i++)
    {
        let bar = container.itemAt(i).bar

        bar.value = fieldMsg[i]

        if(fieldRefMsg !== undefined) {
            bar.valueRef = fieldRefMsg[i]
        }
        else {
            bar.refMarker.visible = false
        }
    }
}

var barPlotDefaultModel = [
            {
                'fieldName': 'motPos',
                'refName': 'posRef',
                'min': SharedData.qmin,
                'max': SharedData.qmax,
            },
            {
                'fieldName': 'motVel',
                'refName': 'velRef',
                'min': SharedData.vmax.map(x => -x),
                'max': SharedData.vmax,
            },
            {
                'fieldName': 'tor',
                'refName': 'torRef',
                'min': SharedData.taumax.map(x => -x),
                'max': SharedData.taumax,
            },
            {
                'fieldName': 'motorTemp',
                'refName': '',
                'min': Array(SharedData.jointNames.length).fill(20),
                'max': Array(SharedData.jointNames.length).fill(80),
            },
            {
                'fieldName': 'driverTemp',
                'refName': '',
                'min': Array(SharedData.jointNames.length).fill(20),
                'max': Array(SharedData.jointNames.length).fill(80),
            },
            {
                'fieldName': 'k',
                'refName': '',
                'min': Array(SharedData.jointNames.length).fill(0),
                'max': Array(SharedData.jointNames.length).fill(5000),
            },
            {
                'fieldName': 'd',
                'refName': '',
                'min': Array(SharedData.jointNames.length).fill(0),
                'max': Array(SharedData.jointNames.length).fill(100),
            },
        ]

var barPlotFields = ['motPos', 'linkPos', 'motVel', 'linkVel', 'tor', 'motorTemp', 'k', 'd']

var shortToLongName = SjsData.shortToLongName

function getLongName(shortName) {
    if(shortName in shortToLongName) {
        return shortToLongName[shortName]
    }
    else {
        return shortName
    }
}

