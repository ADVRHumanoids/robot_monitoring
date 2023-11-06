function httpRequestRaw(verb, url, body, callback) {

    var xhr = new XMLHttpRequest();



    xhr.onreadystatechange = function()
    {
        if (xhr.readyState === XMLHttpRequest.HEADERS_RECEIVED)
        {
        }
        else if(xhr.readyState === XMLHttpRequest.DONE)
        {
            if(callback !== undefined)
            {
                callback(xhr.response)
            }

        }
    }

    xhr.open(verb, url);

    xhr.responseType = 'arraybuffer'

    xhr.send(body);
}

function httpRequest(verb, url, body, callback) {

    var xhr = new XMLHttpRequest();

    xhr.onreadystatechange = function()
    {
        if (xhr.readyState === XMLHttpRequest.HEADERS_RECEIVED)
        {
        }
        else if(xhr.readyState === XMLHttpRequest.DONE)
        {
            try {
                var object = JSON.parse(xhr.responseText.toString());
            }
            catch(err) {
                console.log(verb + ' ' + url + ': failed to parse message: ' + xhr.responseText.toString())
                return;
            }


            if(callback !== undefined)
            {
                callback(object)
            }

        }
    }

    xhr.open(verb, url);

    xhr.send(body);
}


function httpRequestAsync(verb, url, body) {

    let promise = new Promise(
            (resolve, reject) =>
            {
                let xhr = new XMLHttpRequest();
                xhr.open(verb, url);
                xhr.onload = () => {
                    if (xhr.status >= 200 && xhr.status < 300) {
                        var object = JSON.parse(xhr.responseText.toString());
                        resolve(object);
                    } else {
                        reject(xhr.statusText);
                    }
                };
                xhr.onerror = () => reject(xhr.statusText);
                xhr.send(body);
            });

    return promise
}
