function setStream(stream_name, video) {
    var msg = {
        'type': 'video_request',
        'stream_name': stream_name,
        'operation': stream_name === '' ? 'disconnect' : 'connect'
    }
    client.sendTextMessageUdp(JSON.stringify(msg))
}

function refreshNames(video = undefined, cb = undefined) {
    client.doRequest('GET', '/video/get_names', {},
                     function(msg) {

                         if(!msg.success) {
                             console.log('could not fetch stream list')
                             return;
                         }

                         if(cb === undefined) {
                             video.availableStreamIds = msg.topics
                         }
                         else {
                             cb(msg.topics)
                         }


                     }
                     )
}
