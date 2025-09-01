function _base64ToArrayBuffer(base64) {
    return appData.base64ToBytes(base64)
}

function setStream (stream_name, video) {

    // request disconnection from previous video stream
    console.log('requesting video stream disconnection via ws..')
    client.sendTextMessage(JSON.stringify({'type': 'video_request',
                                           'operation': 'disconnect',
                                           'stream_name': video.streamName}))
    console.log('..done')

    let body = JSON.stringify({'stream_name': stream_name})

    client.doRequest('PUT', '/video/set_stream', body,
                     function(msg) {

                         if(!msg.success) {
                             console.log('could not enable stream ' + stream_name)
                             return;
                         }

                         console.log(`enabled stream ${stream_name}, setting headers..`)

                         // convert hdr.data from base64
                         for(let i = 0; i < 3; i++) {
                             msg.hdr[i].data = _base64ToArrayBuffer(msg.hdr[i].data)
                         }

                         video.setTheoraHeader(msg.hdr)

                         video.streamName = stream_name

                         // request video messages via ws
                         console.log(`requesting video stream ${stream_name} via ws..`)
                         client.sendTextMessage(JSON.stringify({'type': 'video_request',
                                                                'stream_name': stream_name}))
                         console.log('..done')
                     }
                     )


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
