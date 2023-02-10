function appendText(text, consoleId, scrollOnOutput) {
    consoleId.append(text)
    if(scrollOnOutput) {
        consoleId.cursorPosition = consoleId.length - 1
    }
}
