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
            text: 'Recognized text:'
        }

        Label {

            id: speechTextLabel

            font.pixelSize: 40

            text: '--'

            onTextChanged: timer.start()

            property Timer timer: Timer {
                interval: 2000
                repeat: false
                onTriggered: speechTextLabel.text = '--'
            }

        }

        Label {
            text: 'Status:'
        }


        Label {

            id: statusLabel

            font.pixelSize: 40

            text: 'waiting for magic prompt'
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
                    statusLabel.text = 'waiting for command'
                    tts.say(statusLabel.text)
                }
                else if(msg.cmd === '__invalid__') {
                    statusLabel.text = 'invalid command'
                    tts.say(statusLabel.text)
                }
                else if(msg.cmd === '__done__') {
                    statusLabel.text = 'command executed ok'
                }
                else if(msg.cmd === '__timeout__') {
                    statusLabel.text = 'no command received'
                    tts.say(statusLabel.text)
                }
                else if(msg.cmd === '__error__') {
                    statusLabel.text = 'an error occurred'
                    tts.say(statusLabel.text)
                }
                else if(msg.cmd === '__end__') {
                    statusLabel.text = 'waiting for magic prompt'
                }
                else {
                    statusLabel.text = 'will execute command: ' + msg.cmd
                    tts.say(statusLabel.text)
                }

                return
            }
        }
    }


}
