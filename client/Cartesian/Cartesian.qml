import QtQuick 2.4
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Controls.Material

Page {

    Row {

        anchors.centerIn: parent
        spacing: 50

        Pad {

            onJoystickMoved: function (x, y) {
                console.log(x + ', ' + y)
            }
        }

        Pad {

            horizontalOnly: true

            onJoystickMoved: function (x, y) {
                console.log(x + ', ' + y)
            }
        }

    }

}
