function xZero() {

    var minNormalized = -min / (max - min)

    var zero = barMargin + Math.max(minNormalized * (bar.parent.width - 2*barMargin), 0)

    return zero

}


function xOffset() {

    var off = xZero()

    if(valueNormalized < 0)
    {
        off = off - bar.width
    }

    return off
}


function width() {
    return (bar.parent.width - 2 * barMargin) * Math.abs(valueNormalized)
}


function setJointStateMessage(msg)
{
    for(var i = 0; i < jointNames.length; i++)
    {
        container.itemAt(i).bar.value = msg[fieldName][i]
    }
}
