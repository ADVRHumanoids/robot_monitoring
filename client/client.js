function httpRequest(url, callback) {
    var xhr = new XMLHttpRequest();
    xhr.onreadystatechange = function()
    {
        if (xhr.readyState === XMLHttpRequest.HEADERS_RECEIVED)
        {
            print('HEADERS_RECEIVED');
        }
        else if(xhr.readyState === XMLHttpRequest.DONE)
        {
            var object = JSON.parse(xhr.responseText.toString());
            callback(object)
        }
    }
    xhr.open("GET", url);
    xhr.send();
}
