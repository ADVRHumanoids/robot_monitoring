#include "websocket_receiver.h"


WebsocketReceiver::WebsocketReceiver(QObject *parent):
    QObject(parent)
{
}

void WebsocketReceiver::setMessage(QByteArray msg)
{
    if(!_js.deserialize(&_pb, msg))
    {
        qCritical("could not deserialize joint state message from proto");
    }


    emit jointStateReceived(_js);

}
