#include "protobuf_deserialization.h"

void ProtobufDeserializationWorker::processBinaryMessage(const QByteArray &msg)
{
    if (!_msg.deserialize(&_serializer, msg))
    {
        qWarning().nospace() << "unable to deserialize datagram ("
                             << qToUnderlying(_serializer.lastError()) << ")"
                             << _serializer.lastErrorString();
        return;
    }

    _counters.all++;
    _bytes_recv.all += msg.size();

    if (_msg.hasJointstate())
    {
        _counters.js++;
        _bytes_recv.js += msg.size();
        emit jointStateReceived(_msg.jointstate());
    }
    else if (_msg.hasText())
    {
        _counters.text++;
        _bytes_recv.text += msg.size();
        emit textMessageReceived(_msg.text().text());
    }
    else if(_msg.hasProcessOutput())
    {
        _counters.proc++;
        _bytes_recv.proc += msg.size();
        emit processOutputReceived(_msg.processOutput());
    }
    else if(_msg.hasTheoraPacket())
    {
        _counters.video++;
        _bytes_recv.video += msg.size();
        emit theoraPacketReceived(_msg.theoraPacket());
    }
    else if(_msg.hasMpegTsDatagram())
    {
        _counters.video++;
        _bytes_recv.video += msg.size();
        emit mpegTsDatagramReceived(_msg.mpegTsDatagram());
    }
    else if(_msg.hasPointCloud())
    {
        _counters.pointcloud++;
        _bytes_recv.pointcloud += msg.size();
        emit pointCloudReceived(_msg.pointCloud());
    }
    else
    {
        qWarning("empty protobuf msg received");
    }

    emit countersUpdated(_counters);
    emit bytesRecvUpdated(_bytes_recv);
}

ProtobufDeserialization::ProtobufDeserialization(QObject *parent)
{
    auto worker = new ProtobufDeserializationWorker;

    connect(this,
            &ProtobufDeserialization::processBinaryMessage,
            worker,
            &ProtobufDeserializationWorker::processBinaryMessage);

    connect(worker,
            &ProtobufDeserializationWorker::textMessageReceived,
            this,
            &ProtobufDeserialization::textMessageReceived);

    connect(worker,
            &ProtobufDeserializationWorker::jointStateReceived,
            this,
            &ProtobufDeserialization::jointStateReceived);

    connect(worker,
            &ProtobufDeserializationWorker::processOutputReceived,
            this,
            &ProtobufDeserialization::processOutputReceived);

    connect(worker,
            &ProtobufDeserializationWorker::theoraPacketReceived,
            this,
            &ProtobufDeserialization::theoraPacketReceived);

    connect(worker,
            &ProtobufDeserializationWorker::mpegTsDatagramReceived,
            this,
            &ProtobufDeserialization::mpegTsDatagramReceived);

    connect(worker,
            &ProtobufDeserializationWorker::pointCloudReceived,
            this,
            &ProtobufDeserialization::pointCloudReceived);

    connect(worker,
            &ProtobufDeserializationWorker::bytesRecvUpdated,
            this,
            [this](Counters br)
            {
                _bytes_recv = br;
                emit recvBytesChanged();
            });

    connect(worker,
            &ProtobufDeserializationWorker::countersUpdated,
            this,
            [this](Counters br)
            {
                _num_msg = br;
                emit numMsgChanged();
            });

#ifndef __EMSCRIPTEN__
    worker->moveToThread(&_thread);
#if QT_VERSION >= QT_VERSION_CHECK(6, 9, 0)
    _thread.setServiceLevel(QThread::QualityOfService::Eco);
#endif

    _thread.start(QThread::Priority::LowPriority);
#endif

}

ProtobufDeserialization::~ProtobufDeserialization()
{
#ifndef __EMSCRIPTEN__
    _thread.quit();
    _thread.wait();
#endif
}

Counters ProtobufDeserialization::recvBytes() const
{
    return _bytes_recv;
}

Counters ProtobufDeserialization::numMsg() const
{
    return _num_msg;
}
