.import Common 1.0 as Common

let error = Common.CommonProperties.notifications.error
let info = Common.CommonProperties.notifications.info

function notifyStatus(verb, url, xhr) {

    let statusText = xhr.statusText

    if(xhr.status === 0) {
        statusText = 'Server not found'
    }

    if(xhr.status < 200 || xhr.status >= 300) {
        error(`${verb} ${url} failed: ${statusText} (${xhr.status})`, 'http')
        return false
    }
    else {
        info(`${verb} ${url} succeeded (${xhr.status})`, 'http')
        return true
    }
}

function notifyResponseStatus(verb, url, res) {
    if('success' in res) {
        if(res.success) {
            info(`${verb} ${url} answered: <b>${res.message || 'no message'}</b>`, 'http')
        }
        else {
            error(`${verb} ${url} answered: <b>${res.message || 'no message'}</b>`, 'http')
        }
    }
}


function httpRequestRaw(verb, url, body, callback) {

    var xhr = new XMLHttpRequest();

    xhr.onreadystatechange = function()
    {
        if (xhr.readyState === XMLHttpRequest.HEADERS_RECEIVED)
        {
        }
        else if(xhr.readyState === XMLHttpRequest.DONE)
        {
            if(!notifyStatus(verb, url, xhr))
            {
                return;
            }

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
            if(!notifyStatus(verb, url, xhr))
            {
                return;
            }

            try {
                var object = JSON.parse(xhr.responseText.toString())
                notifyResponseStatus(verb, url, object)
            }
            catch(err) {
                error(verb + ' ' + url + ': failed to parse message: ' + xhr.responseText.toString())
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

                    if(!notifyStatus(verb, url, xhr))
                    {
                        reject(xhr.statusText);
                    }

                    try {
                        var object = JSON.parse(xhr.responseText.toString());
                        notifyResponseStatus(verb, url, object)
                        resolve(object);
                    }
                    catch(err) {
                        error(verb + ' ' + url + ': failed to parse message: ' + xhr.responseText.toString())
                        reject(verb + ' ' + url + ': failed to parse message: ' + xhr.responseText.toString())
                    }

                }

                xhr.onerror = () => {
                    notifyStatus(verb, url, xhr);
                    reject(xhr.statusText);
                }

                xhr.send(body);
            });

    return promise
}
