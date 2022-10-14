import QtQuick 2.15

ProcessForm {

    id: root
    property string name: "InvalidName"
    property string status: "InvalidStatus"

    signal start
    signal stop
    signal kill

    startBtn.onReleased: {
        start()
    }

    stopBtn.onReleased: {
        stop()
    }

    killBtn.onReleased: {
        kill()
    }
}
