function setStream (stream_name) {
    let body = JSON.stringify({'stream_name': stream_name})
    client.doRequest('PUT', '/video/set_stream', body,
                     function(msg) {

                         if(!msg.success) {
                             console.log('could not enable stream ' + stream_name)
                             return;
                         }

                         video.setTheoraHeader(msg.hdr)
                     }
                     )
}

function refreshNames() {
    client.doRequest('GET', '/video/get_names', {},
                     function(msg) {

                         if(!msg.success) {
                             console.log('could not fetch stream list')
                             return;
                         }

                         video.names = msg.topics

                     }
                     )
}
