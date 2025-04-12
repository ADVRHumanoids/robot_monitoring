#ifndef PROTOBUF_DESERIALIZATION_H
#define PROTOBUF_DESERIALIZATION_H

#include <QThread>
#include <QObject>
#include <QQmlEngine>

#include <QProtobufSerializer>

#include "jointstate.qpb.h"
#include "generic.qpb.h"
#include "text.qpb.h"

class ProtobufDeserializationWorker : public QObject
{
    Q_OBJECT

public:

    void processBinaryMessage(const QByteArray& msg)
    {
        if(!_msg.deserialize(&_serializer, msg))
        {
            qWarning().nospace() << "Unable to deserialize datagram ("
                                 << qToUnderlying(_serializer.lastError()) << ")"
                                 << _serializer.lastErrorString();
            return;
        }

        _counters.msg++;

        if(_msg.hasJointstate())
        {
            _counters.js++;
            emit jointStateReceived(_msg.jointstate());
        }
        else if(_msg.hasText())
        {
            // qInfo() << "got text" << _msg.text().text();
            emit textMessageReceived(_msg.text().text());
            _counters.text++;
        }
        else
        {
            qWarning("empty protobuf msg received");
        }
    }

signals:

    void textMessageReceived(const QString&);

    void jointStateReceived(const JointState&);


private:


    QProtobufSerializer _serializer;

    Message _msg;

    struct {
        int msg = 0;
        int js = 0;
        int text = 0;
    } _counters;

    struct {
        int msg = 0;
        int js = 0;
        int text = 0;
    } _bytes_recv;

};

class ProtobufDeserialization : public QObject
{
    Q_OBJECT
    QML_ELEMENT

public:

    ProtobufDeserialization(QObject * parent = nullptr)
    {
        auto worker = new ProtobufDeserializationWorker;

        connect(this, &ProtobufDeserialization::processBinaryMessage,
                worker, &ProtobufDeserializationWorker::processBinaryMessage);

        connect(worker, &ProtobufDeserializationWorker::textMessageReceived,
                this, &ProtobufDeserialization::textMessageReceived);

        connect(worker, &ProtobufDeserializationWorker::jointStateReceived,
                this, &ProtobufDeserialization::jointStateReceived);

        worker->moveToThread(&_thread);

        _thread.setServiceLevel(QThread::QualityOfService::Eco);

        _thread.start(QThread::Priority::LowPriority);

    }


signals:

    void processBinaryMessage(const QByteArray& msg);

    void textMessageReceived(const QString&);

    void jointStateReceived(const JointState&);

private:

    QThread _thread;



};

#endif // PROTOBUF_DESERIALIZATION_H
