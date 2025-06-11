function kill() {
    client.doRequestAsync('POST', '/emergency/kill', '')
        console.log('Killing')
        .then((resposne) => {
            console.log('Killed')
        })
}
