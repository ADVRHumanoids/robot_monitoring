#ifndef WEBSOCKET_H
#define WEBSOCKET_H

#include <QObject>
#include <QQmlEngine>
#include <QtWebSockets/QtWebSockets>

class WebSocketWorker : public QObject
{

    Q_OBJECT
    QML_ELEMENT

public:

    WebSocketWorker(QObject *parent = nullptr): QObject(parent)
    {
        _thread = new QThread(this);
        _thread->start(QThread::Priority::LowPriority);
        _thread->setServiceLevel(QThread::QualityOfService::Eco);

        _ws.moveToThread(_thread);
    }

    Q_PROPERTY(QUrl url READ url WRITE setUrl NOTIFY urlChanged FINAL)

    Q_PROPERTY(bool active READ active WRITE setActive NOTIFY activeChanged FINAL)

    Q_INVOKABLE void sendTextMessage(QString msg)
    {
        _ws.sendTextMessage(msg);
    }

    void setUrl(QUrl __url)
    {
        if(__url == url())
        {
            return;
        }

        _url = __url;

        connect(&_ws, &QWebSocket::connected,
                    this, &WebSocketWorker::connected);

        connect(&_ws, &QWebSocket::disconnected,
                this, [this]()
                {
                    emit disconnected();
                    _active = false;
                    emit activeChanged();
                }
                );

        connect(&_ws, &QWebSocket::binaryMessageReceived,
                this, &WebSocketWorker::binaryMessageReceived);

        connect(&_ws, &QWebSocket::stateChanged,
                this, [](QAbstractSocket::SocketState ss)
                {
                    qInfo() << "ws state changed: " << ss;
                });

        connect(&_ws, &QWebSocket::errorOccurred,
                this, [this](QAbstractSocket::SocketError se)
                {
                    qInfo() << "socket error: " << _ws.errorString();
                    emit errorOccurred(_ws.errorString());
                });


        qInfo() << "set url" << _url;

        // _ws.moveToThread(nullptr);

        _ws.close();

        _ws.open(_url);

        emit urlChanged();
    }

    void setActive(bool value)
    {
        if(active() == value)
        {
            return;
        }

        qInfo() << "set active" << value;

        _active = value;

        // _ws.moveToThread(nullptr);

        if(_active)
        {
            _ws.open(_url);
        }
        else
        {
            _ws.close();
        }

        emit activeChanged();

        // _ws.moveToThread(_thread);
    }

    QUrl url() const
    {
        return _url;
    }

    bool active() const
    {
        return _active;
    }

signals:

    void activeChanged();

    void errorOccurred(QString error);

    void connected();

    void disconnected();

    void urlChanged();

    void binaryMessageReceived(QByteArray msg);

private:

    QUrl _url;

    bool _active = true;

    QWebSocket _ws;

    QThread * _thread;
};


#endif // WEBSOCKET_H
