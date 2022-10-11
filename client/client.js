function httpRequest(verb, url, body, callback) {
    var xhr = new XMLHttpRequest();
    xhr.onreadystatechange = function()
    {
        if (xhr.readyState === XMLHttpRequest.HEADERS_RECEIVED)
        {
            print('HEADERS_RECEIVED');
        }
        else if(xhr.readyState === XMLHttpRequest.DONE)
        {
            console.log(xhr.responseText.toString())

            var object = JSON.parse(xhr.responseText.toString());

            if(callback !== undefined)
            {
                callback(object)
            }

        }
    }
    xhr.open(verb, url);
    xhr.send(body);
}
