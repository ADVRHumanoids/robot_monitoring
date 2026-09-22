import QtQuick
import QtQuick.Controls

FocusScope {
    id: root

    property color pillColor: "#DCEBFF"
    property color pillTextColor: "#1D4E89"
    property color pillHoverColor: "#C8DFFF"

    property string placeholderText: "Add address..."

    readonly property int count: tokenModel.count

    signal tokenAdded(string text)
    signal tokenRemoved(string text)

    implicitWidth: 400
    implicitHeight: flow.implicitHeight

    function addToken(text) {
        const value = text.trim()

        if (value.length === 0)
            return

        tokenModel.append({ text: value })
        tokenAdded(value)
    }

    function removeToken(index) {
        if (index < 0 || index >= tokenModel.count)
            return

        const value = tokenModel.get(index).text
        tokenModel.remove(index)
        tokenRemoved(value)
    }

    function clear() {
        tokenModel.clear()
        editor.clear()
    }

    function values() {
        let result = []

        for (let i = 0; i < tokenModel.count; ++i)
            result.push(tokenModel.get(i).text)

        return result
    }

    ListModel {
        id: tokenModel
    }

    Flow {
        id: flow

        width: parent.width
        spacing: 6

        Repeater {
            model: tokenModel

            delegate: Rectangle {
                id: pill

                required property string text
                required property int index

                height: 30
                width: pillLabel.implicitWidth + closeButton.width + 24

                radius: height / 2
                color: pillMouse.containsMouse
                       ? root.pillHoverColor
                       : root.pillColor

                Text {
                    id: pillLabel

                    anchors {
                        left: parent.left
                        leftMargin: 12
                        verticalCenter: parent.verticalCenter
                    }

                    text: pill.text
                    color: root.pillTextColor
                    elide: Text.ElideRight
                    width: Math.min(implicitWidth, root.width - 80)
                }

                ToolButton {
                    id: closeButton

                    anchors {
                        right: parent.right
                        rightMargin: 3
                        verticalCenter: parent.verticalCenter
                    }

                    width: 26
                    height: 26

                    text: "×"

                    background: Item {}

                    contentItem: Text {
                        text: closeButton.text
                        color: root.pillTextColor
                        font.pixelSize: 18
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }

                    onClicked: root.removeToken(pill.index)
                }

                MouseArea {
                    id: pillMouse

                    anchors.fill: parent
                    acceptedButtons: Qt.NoButton
                    hoverEnabled: true
                }
            }
        }

        TextInput {
            id: editor

            // Tells Android which action to show on the virtual keyboard.
            EnterKey.type: Qt.EnterKeyNext

            width: Math.max(120, implicitWidth + 16)
            height: 30

            verticalAlignment: TextInput.AlignVCenter
            clip: true

            color: palette.active.text

            function commit() {
                const value = text.trim()

                if (value.length === 0)
                    return

                root.addToken(value)
                clear()

                // Keep typing into the token field.
                forceActiveFocus()
            }

            // Works for Android IME Enter / Next as well as physical Enter.
            onAccepted: commit()

            // Android virtual keyboards don't necessarily generate Qt.Key_Space.
            // Detect the space after the IME has inserted it into the text.
            onTextEdited: {
                if (!inputMethodComposing && /\s$/.test(text))
                    commit()
            }

            // Keep Keys handling for physical keyboards and Backspace.
            Keys.onPressed: event => {
                                if (event.key === Qt.Key_Backspace
                                    && text.length === 0
                                    && tokenModel.count > 0) {
                                    root.removeToken(tokenModel.count - 1)
                                    event.accepted = true
                                }
                            }

            Text {
                anchors {
                    left: parent.left
                    verticalCenter: parent.verticalCenter
                }

                visible: editor.text.length === 0 && tokenModel.count === 0
                text: root.placeholderText
                color: "#888888"
                enabled: false
            }
        }
    }

    MouseArea {
        anchors.fill: parent
        z: -1
        onClicked: editor.forceActiveFocus()
    }
}
