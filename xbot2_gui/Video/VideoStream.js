function setStream (stream_name, video) {
    let body = JSON.stringify({'stream_name': stream_name})
    client.doRequest('PUT', '/video/set_stream', body,
                     function(msg) {

                         if(!msg.success) {
                             console.log('could not enable stream ' + stream_name)
                             return;
                         }

                         console.log(`enabled stream ${stream_name}, setting headers..`)

                         video.setTheoraHeader(msg.hdr)
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
