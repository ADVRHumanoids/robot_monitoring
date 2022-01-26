function xZero() {

    var minNormalized = -min / (max - min)

    var zero = barMargin + Math.max(minNormalized * (bar.parent.width - 2*barMargin), 0) + barMargin

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
    return (parent.width - 2 * barMargin) * Math.abs(valueNormalized)
}
