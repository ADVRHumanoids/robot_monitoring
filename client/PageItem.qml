import QtQuick

Item {
    property url page: ''
    property string name: ''
    property string iconText: name[0]
    property string iconFont: Qt.application.font
    property bool active: false
}
