import QtQuick

Item {

    signal pageSelected()

    property bool isCurrentPage: false

    property int numErrors: 0

    property string pageName: '--'

}
