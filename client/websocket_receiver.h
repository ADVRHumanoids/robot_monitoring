#ifndef WEBSOCKETRECEIVER_H
#define WEBSOCKETRECEIVER_H

#include <QWebSocket>
#include <QtProtobuf/QtProtobuf>

#include "Protobuf/joint_states.qpb.h"


class WebsocketReceiver : public QObject
{

    Q_OBJECT
    QML_ELEMENT

public:

    WebsocketReceiver(QObject *parent = nullptr);

    void setMessage(QByteArray msg);

signals:

    void jointStateReceived(const xbot2_gui::msgs::joint_states::JointStates& js);

private:

    QProtobufSerializer _pb;

    xbot2_gui::msgs::joint_states::JointStates _js;



};

#endif // WEBSOCKETRECEIVER_H
