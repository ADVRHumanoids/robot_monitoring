#include "websocket.h"

WebSocketWorker::WebSocketWorker(QObject *parent)
    : QObject(parent)
{

}

void WebSocketWorker::initialize()
{
    qInfo("initializing websocket...");

    _ws = new QWebSocket("", QWebSocketProtocol::VersionLatest, this);

    connect(_ws, &QWebSocket::connected, this, &WebSocketWorker::connected);

    connect(_ws, &QWebSocket::disconnected, this, [this]() {
        emit disconnected();
        _active = false;
        emit activeChanged(_active);
    });

    connect(_ws, &QWebSocket::binaryMessageReceived, this, &WebSocketWorker::binaryMessageReceived);

    connect(_ws, &QWebSocket::stateChanged, this, [](QAbstractSocket::SocketState ss) {
        qInfo() << "ws state changed: " << ss;
    });

    connect(_ws, &QWebSocket::errorOccurred, this, [this](QAbstractSocket::SocketError se) {
        qInfo() << "socket error: " << _ws->errorString();
        emit errorOccurred(_ws->errorString());
    });

    qInfo("...done");
}

void WebSocketWorker::setUrl(QUrl url)
{
    if (!_ws)
    {
        qFatal("ws not initialized");
        return;
    }

    if (_url == url)
    {
        return;
    }

    _url = url;

    qInfo() << "set url" << _url;

    if (_active) {
        _ws->close();
        _ws->open(_url);
    }

    emit urlChanged(_url);
}

void WebSocketWorker::setActive(bool active)
{
    if (_active == active) {
        return;
    }

    qInfo() << "set active" << active;

    _active = active;

    if (_active)
    {
        _ws->open(_url);
    }
    else
    {
        _ws->close();
    }

    emit activeChanged(_active);

}

void WebSocketWorker::sendTextMessage(QString msg)
{
    if (!_ws) {
        qWarning("try to send text message from null websocket");
        return;
    }

    _ws->sendTextMessage(msg);
}

WebSocketAsync::WebSocketAsync(QObject *parent)
    : QObject(parent)
{
    // create worker
    auto worker = new WebSocketWorker;

    // move worker to thread
    worker->moveToThread(&_thread);

    // initialize
    connect(&_thread, &QThread::started,
            worker, &WebSocketWorker::initialize);

    // connect signals and slots
    connect(this, &WebSocketAsync::setUrlRequested, worker, &WebSocketWorker::setUrl);

    connect(worker, &WebSocketWorker::urlChanged, this, [this](QUrl url) {
        _url_cached = url;
        emit urlChanged();
    });

    connect(this, &WebSocketAsync::setActiveRequested, worker, &WebSocketWorker::setActive);

    connect(worker, &WebSocketWorker::activeChanged, this, [this](bool active) {
        _active_cached = active;
        emit activeChanged();
    });

    connect(worker, &WebSocketWorker::connected, this, &WebSocketAsync::connected);

    connect(worker, &WebSocketWorker::disconnected, this, &WebSocketAsync::disconnected);

    connect(worker, &WebSocketWorker::errorOccurred, this, &WebSocketAsync::errorOccurred);

    connect(worker,
            &WebSocketWorker::binaryMessageReceived,
            this,
            &WebSocketAsync::binaryMessageReceived);

    connect(this, &WebSocketAsync::sendTextMessage, worker, &WebSocketWorker::sendTextMessage);


    // start thread
#if QT_VERSION >= QT_VERSION_CHECK(6, 9, 0)
    _thread.setServiceLevel(QThread::QualityOfService::Eco);
#endif
    _thread.start(QThread::Priority::LowPriority);
}

void WebSocketAsync::setUrl(QUrl _url)
{
    emit setUrlRequested(_url);
}

void WebSocketAsync::setActive(bool value)
{
    emit setActiveRequested(value);
}

QUrl WebSocketAsync::url() const
{
    return _url_cached;
}

bool WebSocketAsync::active() const
{
    return _active_cached;
}

WebSocketAsync::~WebSocketAsync()
{
    _thread.quit();
    _thread.wait();
}
