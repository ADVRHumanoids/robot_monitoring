function setStream(stream_name, video) {
    var msg = {
        'type': 'video_rtsp_request',
        'stream_name': stream_name,
        'operation': stream_name === '' ? 'disconnect' : 'connect'
    }
    client.sendTextMessage(JSON.stringify(msg))
    if(stream_name === '') {
       video.source = ''
    }

    video.restart()
}

function _parseHostPort(value, defaultPort) {
    value = value.trim();

    const i = value.lastIndexOf(":");

    if (i === -1) {
        return {
            host: value,
            port: defaultPort
        };
    }

    const host = value.slice(0, i).trim();
    const portText = value.slice(i + 1).trim();

    return {
        host: host,
        port: portText === "" ? defaultPort : Number(portText)
    };
}

function refreshNames(videoSources, cb) {

    for(const src of videoSources) {

        // try to split the source into host and port
        const { host, port } = _parseHostPort(src, 9997);

        // use mediamtx control api to get list of streams
        client.doRequest('GET', '/v3/paths/list', {},
                         function(msg) {
                            let streams = []
                            for(const item of msg.items) {
                                console.log(`found stream ${src}/${item.name}`)
                                streams.push(`rtsp://${host}:8554/${item.name}`)
                            }
                            cb(streams)
                         },
                         false, host, port
        )
    }

}
