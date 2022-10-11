import QtQuick 2.15

ProcessForm {

    id: root
    property string name: "InvalidName"
    property string status: "InvalidStatus"
    property var cmdList: ["Command 1", "Command 2"]

    signal start
    signal stop
    signal kill
    signal sendCommand(string cmd)
    signal cancelCommand(string cmd)

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
