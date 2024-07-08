#include "udpsocket.h"
#include <QHostInfo>
#include <QEventLoop>

UdpSocket::UdpSocket(QObject *parent)
    : QObject{parent}, _port(0)
{

    connect(&_sock, &QUdpSocket::readyRead,
            this, &UdpSocket::readyRead);

    connect(&_sock, &QUdpSocket::stateChanged,
            this, &UdpSocket::socketStateChanged);

    _sock.bind();

}

void UdpSocket::sendTextMessage(QString msg)
{
    if(_hostname.size() == 0 || _port == 0)
    {
        return;
    }

    QNetworkDatagram dg;
    dg.setDestination(_addr, _port);
    dg.setData(msg.toUtf8());

    if(_sock.writeDatagram(dg) < 0)
    {
        qWarning("could not send udp datagram");
    }
}

void UdpSocket::rebind()
{
    _sock.abort();

    _sock.bind();
}

QString UdpSocket::hostname() const
{
    return _hostname;
}

void UdpSocket::setHostname(QString hostname)
{
    if(_hostname != hostname)
    {
        // auto info = QHostInfo::fromName(hostname);
        _hostname = hostname;
        _addr = QHostAddress(hostname == "localhost" ? "127.0.0.1" : hostname);
        emit hostnameChanged(hostname);
        emit addressChanged();
        qInfo() << "set udp remote hostname to " << hostname;
    }
}

int UdpSocket::port() const
{
    return _port;
}

void UdpSocket::setPort(int port)
{
    if(_port != port)
    {
        _port = port;
        emit portChanged(port);
        emit addressChanged();
        qInfo() << "set udp remote port to " << port;
    }
}

bool UdpSocket::bound() const
{
    return _sock.state() == QAbstractSocket::BoundState;
}

void UdpSocket::readyRead()
{
    while(_sock.hasPendingDatagrams())
    {
        _dg = _sock.receiveDatagram();

        QString decoded_data(_dg.data());

        emit textMessageReceived(decoded_data);

    }
}

void UdpSocket::socketStateChanged(QAbstractSocket::SocketState state)
{
    qInfo() << "udp socket entered state " << state;

    emit boundChanged(bound());
}
