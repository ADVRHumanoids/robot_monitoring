import QtQuick

Item {

    visible: false

    required property int targetWidth
    required property int targetHeight
    property bool orientationBasedLayout: false

    signal beforeLayoutChange()
    signal afterLayoutChange()

    enum Class {
        Compact, Medium, Expanded
    }

    property int _layoutClass: _computeLayoutClass(targetWidth, targetHeight)

    property int layoutClass
    property bool compact: false
    property bool medium: false
    property bool expanded: false

    property bool _compact: targetWidth < 600
    property bool _medium: targetWidth < 840 && !compact
    property bool _expanded: !compact && !medium

    function _computeLayoutClass(w, h) {
        if (orientationBasedLayout) {
            return h > w ? LayoutClassHelper.Class.Compact : LayoutClassHelper.Class.Expanded
        } else {
            return w < 600 ?
                       LayoutClassHelper.Class.Compact :
                       (w < 840 ? LayoutClassHelper.Class.Medium :
                       LayoutClassHelper.Class.Expanded)
        }
    }

    function updateLayout() {
        beforeLayoutChange()
        compact = _layoutClass === LayoutClassHelper.Class.Compact
        medium = _layoutClass === LayoutClassHelper.Class.Medium
        expanded = _layoutClass === LayoutClassHelper.Class.Expanded
        Qt.callLater(afterLayoutChange)
    }

    on_LayoutClassChanged: {
        updateLayout()
    }

    Component.onCompleted: {
        updateLayout()
    }

}
