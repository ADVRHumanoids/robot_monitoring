#ifndef UDPSOCKET_H
#define UDPSOCKET_H

#include <QObject>
#include <QQmlEngine>
#include <QUdpSocket>
#include <QNetworkDatagram>

class UdpSocket : public QObject
{
    Q_OBJECT
    QML_ELEMENT

public:

    explicit UdpSocket(QObject *parent = nullptr);

    Q_PROPERTY(QString hostname READ hostname WRITE setHostname NOTIFY hostnameChanged FINAL)

    Q_PROPERTY(int port READ port WRITE setPort NOTIFY portChanged FINAL)

    Q_PROPERTY(bool bound READ bound NOTIFY boundChanged FINAL)

    Q_INVOKABLE void sendTextMessage(QString msg);

    Q_INVOKABLE void rebind();

    /**
     * @return the remote hostname to send messages to (i.e., the gui server)
     */
    QString hostname() const;

    void setHostname(QString hostname);

    /**
     * @return the remote port
     */
    int port() const;

    void setPort(int port);

    bool bound() const;

signals:

    void hostnameChanged(QString);

    void portChanged(int);

    void addressChanged();

    void textMessageReceived(QString msg);

    void boundChanged(bool);

private:

    void readyRead();

    void socketStateChanged(QAbstractSocket::SocketState state);

    QUdpSocket _sock;

    QNetworkDatagram _dg;

    QString _hostname;

    int _port;

    QHostAddress _addr;
};

#endif // UDPSOCKET_H
