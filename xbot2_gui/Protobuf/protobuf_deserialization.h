#ifndef PROTOBUF_DESERIALIZATION_H
#define PROTOBUF_DESERIALIZATION_H

#include <QThread>
#include <QObject>
#include <QQmlEngine>

#include <QProtobufSerializer>

#include "generic.qpb.h"

class Counters {

    Q_GADGET

public:

    Q_PROPERTY(int all MEMBER all);
    Q_PROPERTY(int js MEMBER js);
    Q_PROPERTY(int text MEMBER text);
    Q_PROPERTY(int proc MEMBER proc);
    Q_PROPERTY(int video MEMBER video);
    Q_PROPERTY(int pointcloud MEMBER pointcloud);
    QML_ELEMENT

    int all = 0;
    int js = 0;
    int text = 0;
    int proc = 0;
    int video = 0;
    int pointcloud = 0;
};

class ProtobufDeserializationWorker : public QObject
{
    Q_OBJECT

public:

    void processBinaryMessage(const QByteArray& msg);

signals:

    void textMessageReceived(const QString&);

    void jointStateReceived(const JointState&);

    void processOutputReceived(const ProcessOutput&);

    void theoraPacketReceived(const TheoraPacket&);

    void pointCloudReceived(const PointCloud&);

    void countersUpdated(Counters c);

    void bytesRecvUpdated(Counters c);


private:


    QProtobufSerializer _serializer;

    Message _msg;

    Counters _counters;

    Counters _bytes_recv;

};

class ProtobufDeserialization : public QObject
{
    Q_OBJECT
    QML_ELEMENT

public:

    ProtobufDeserialization(QObject * parent = nullptr);

    Q_PROPERTY(Counters recvBytes READ recvBytes NOTIFY recvBytesChanged FINAL)

    Q_PROPERTY(Counters numMsg READ numMsg NOTIFY numMsgChanged FINAL)

    ~ProtobufDeserialization();

    Counters recvBytes() const;

    Counters numMsg() const;


signals:

    void processBinaryMessage(const QByteArray& msg);

    void textMessageReceived(const QString&);

    void jointStateReceived(const JointState&);

    void processOutputReceived(const ProcessOutput&);

    void theoraPacketReceived(const TheoraPacket&);

    void pointCloudReceived(const PointCloud&);

    void recvBytesChanged();

    void numMsgChanged();

private:

    QThread _thread;

    Counters _bytes_recv, _num_msg;





};

#endif // PROTOBUF_DESERIALIZATION_H
