.import "../sharedData.js" as SharedData
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
    for(var i = 0; i < jointNames.length; i++)
    {
        container.itemAt(i).bar.value = msg[fieldName][i]
        container.itemAt(i).bar.valueRef = msg[fieldNameRef][i]
    }
}

var barPlotFields = ['motPos', 'linkPos', 'motVel', 'linkVel', 'tor', 'motorTemp']

var shortToLongName = SjsData.shortToLongName

var refName = ['posRef',
               'posRef',
               'velRef',
               'velRef',
               'torRef',
        ]

function barPlotMin(){
    return [SharedData.qmin,
            SharedData.qmin,
            SharedData.vmax.map(x => -x),
            SharedData.vmax.map(x => -x),
            SharedData.taumax.map(x => -x)]
}

function barPlotMax(){
    return [SharedData.qmax,
            SharedData.qmax,
            SharedData.vmax,
            SharedData.vmax,
            SharedData.taumax]
}
