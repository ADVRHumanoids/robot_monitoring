import QtQuick
import QtMultimedia
import QtQuick.Controls
import QtTextToSpeech

import LivePlot
import Main

Item {

    property ClientEndpoint client

    AudioSource {

        id: audioSrc

        property bool enableSend: tts.state !== TextToSpeech.Speaking

        onLevelChanged: (level) => {

            // plot audio level

            if(plot.levelTimeSeries === undefined) {
                plot.levelTimeSeries = plot.addSeries('level', {}, false)
                plot.levelTimeSeries.axisValue.min = 0
                plot.levelTimeSeries.axisValue.max = 1.0
                plot.initialTime = appData.getTimeNs() * 1e-9
            }

            plot.addPoint(plot.levelTimeSeries, appData.getTimeNs() * 1e-9 - plot.initialTime, level)

        }

        onReadyRead: {

            if(bytesAvailable < 2048 || !enableSend) return

            let data = readString(2048)

            let msg = {
                'type': 'speech',
                'data': data
            }

            client.sendTextMessage(JSON.stringify(msg))
        }

    }


    TextToSpeech {

        id: tts

    }

    Column {

        spacing: 16

        ComboBox {

            model: audioSrc.devices

            width: parent.width

            onCurrentTextChanged: audioSrc.currentDevice = currentText

        }

        Button {
            text: 'Stop'
            onClicked: audioSrc.stop()
        }

        Plotter {

            id: plot

            plotterLegend: plotterLegend

            interactive: false

            chartView.title: 'Sound level'
            chartView.titleColor: palette.text
            chartView.margins {
                bottom: 6
                left: 6
                right: 6
                top: 6
            }

            property var levelTimeSeries: undefined

            property real initialTime: 0

        }

        Label {

            id: speechTextLabel

            font.pixelSize: 40

            text: 'niente'

            onTextChanged: timer.start()

            property Timer timer: Timer {
                interval: 2000
                repeat: false
                onTriggered: speechTextLabel.text = '--'
            }

        }

    }

    PlotterLegend {
        id: plotterLegend
        chart: plot.chartView
        visible:false
    }

    Connections {

        target: client

        function onObjectReceived(msg) {

            if(msg.type === 'speech_text') {

                speechTextLabel.text = msg.text

                return
            }

            if(msg.type === 'speech_cmd') {

                if(msg.cmd === '__start__') {
                    tts.say('waiting for command')
                }
                else if(msg.cmd === '__invalid__') {
                    tts.say('invalid command')
                }
                else if(msg.cmd === '__done__') {
                    tts.say('executing command')
                }
                else if(msg.cmd === '__timeout__') {
                    tts.say('no command received')
                }
                else if(msg.cmd === '__error__') {
                    tts.say('an error occurred')
                }
                else if(msg.cmd === '__end__') {
                }
                else {
                    tts.say('will execute command: ' + msg.cmd)
                }

                return
            }
        }
    }


}
