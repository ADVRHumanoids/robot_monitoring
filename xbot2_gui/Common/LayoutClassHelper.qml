import QtQuick

Item {

    required property int targetWidth

    property bool compact: targetWidth < 600
    property bool medium: targetWidth < 840 && !compact
    property bool expanded: !compact && !medium

}
