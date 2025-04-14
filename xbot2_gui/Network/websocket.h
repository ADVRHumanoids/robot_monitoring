#ifndef WEBSOCKET_H
#define WEBSOCKET_H

#include <QObject>
#include <QQmlEngine>
#include <QtWebSockets/QtWebSockets>

class WebSocketWorker : public QObject
{
    Q_OBJECT

public:

    WebSocketWorker(QObject *parent = nullptr);

signals:

    void activeChanged(bool active);

    void errorOccurred(QString error);

    void connected();

    void disconnected();

    void urlChanged(QUrl url);

    void binaryMessageReceived(QByteArray msg);

public slots:

    void initialize();

    void setUrl(QUrl url);

    void setActive(bool active);

    void sendTextMessage(QString msg);


private:

    QWebSocket * _ws;

    bool _active = true;

    QUrl _url;
};

class WebSocketAsync : public QObject
{

    Q_OBJECT
    QML_ELEMENT

public:

    WebSocketAsync(QObject *parent = nullptr);

    Q_PROPERTY(QUrl url READ url WRITE setUrl NOTIFY urlChanged FINAL)

    Q_PROPERTY(bool active READ active WRITE setActive NOTIFY activeChanged FINAL)

    void setUrl(QUrl _url);

    void setActive(bool value);

    QUrl url() const;

    bool active() const;

    ~WebSocketAsync();

signals:

    void activeChanged();

    void errorOccurred(QString error);

    void connected();

    void disconnected();

    void urlChanged();

    void binaryMessageReceived(QByteArray msg);

    void sendTextMessage(QString msg);

    void setUrlRequested(QUrl url);

    void setActiveRequested(bool active);

private:

    QThread _thread;
    QUrl _url_cached;
    bool _active_cached = true;
};


#endif // WEBSOCKET_H
