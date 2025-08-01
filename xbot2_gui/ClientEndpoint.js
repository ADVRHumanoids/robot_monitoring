.import Common 1.0 as Common
.import "sharedData.js" as SharedData

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

function httpRequest(verb, url, body, callback, quiet) {

    var xhr = new XMLHttpRequest();

    xhr.onreadystatechange = function()
    {
        if (xhr.readyState === XMLHttpRequest.HEADERS_RECEIVED)
        {
        }
        else if(xhr.readyState === XMLHttpRequest.DONE)
        {
            if(!quiet && !notifyStatus(verb, url, xhr))
            {
                return;
            }

            try {
                var object = JSON.parse(xhr.responseText.toString())
                if(!quiet) {
                    notifyResponseStatus(verb, url, object)
                }
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


function httpRequestAsync(verb, url, body, quiet = false) {

    let promise = new Promise(
            (resolve, reject) =>
            {
                let xhr = new XMLHttpRequest();

                xhr.open(verb, url);

                xhr.onload = () => {

                    if(!quiet && !notifyStatus(verb, url, xhr))
                    {
                        reject(xhr.statusText);
                    }

                    try {
                        var object = JSON.parse(xhr.responseText.toString());
                        if(!quiet) {
                            notifyResponseStatus(verb, url, object)
                        }
                        resolve(object);
                    }
                    catch(err) {
                        error(verb + ' ' + url + ': failed to parse message: ' + xhr.responseText.toString())
                        reject(verb + ' ' + url + ': failed to parse message: ' + xhr.responseText.toString())
                    }

                }

                xhr.onerror = () => {
                    if(!quiet) {
                        notifyStatus(verb, url, xhr);
                    }
                    reject(xhr.statusText);
                }

                xhr.send(body);
            });

    return promise
}

function detectObjectFields(obj, save=true, type=undefined) {

    // first check if we already know the fields
    if(objNumericFields.hasOwnProperty(obj.type)) {
        return objNumericFields[obj.type]
    }

    console.log(JSON.stringify(obj))

    // detect fields that are numeric (float)
    let fields = Object.keys(obj)
    let numericFields = []

    for(let i = 0; i < fields.length; i++) {
        let field = fields[i];
        let fieldType = typeof obj[field];
        if(fieldType === 'number') {
            numericFields.push({src: obj.type ?? type, name: field, type: 'number', length: 1});
            console.log(`NUMERIC FIELD ${obj.type ?? type}.${field}`)
        }
        else if(fieldType === 'object' &&
                Array.isArray(obj[field]) &&
                typeof obj[field][0] === 'number')
        {
            numericFields.push({src: obj.type ?? type, name: field, type: 'array', length: obj[field].length});
            console.log(`ARRAY FIELD ${obj.type ?? type}.${field}`)
        }
        else if(fieldType === 'object') {
            console.log(`calling recursively with type=${type ?? obj[type]} (${type} ${obj.type})`)
            let objNumericFields = detectObjectFields(obj[field], false, type ?? obj.type);
            for(let nf of objNumericFields) {
                console.log(JSON.stringify(nf))
                // prepend the field name to the numeric field
                nf.name = `${field}.${nf.name}`;
                numericFields.push(nf);
                console.log(`OBJECT FIELD ${nf.src}/${nf.name}`)
            }
        }
    }


    // store the numeric fields for this object type
    if(save) {
        objNumericFields[obj.type] = numericFields
        objNumericFieldsChanged()
    }

    return numericFields
}

let lastJsSeqId = -1

function handleMessage(obj) {

    detectObjectFields(obj)

    if(obj.type === "jpeg")
    {
        jpegReceived(obj)
    }
    // else if(obj.type === "theora")
    // {
    //     theoraPacketReceived(obj)
    // }
    else if(obj.type === "plugin_stats")
    {
        pluginStatMessageReceived(obj)
    }
    else if(obj.type === "heartbeat")
    {

    }
    else if(obj.type === 'ping')
    {
        root.srvRtt = (appData.getTimeNs() - obj.cli_time_ns)*1e-6
    }

    objectReceived(obj)
}
