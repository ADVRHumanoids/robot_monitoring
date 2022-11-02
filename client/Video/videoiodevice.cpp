#include "videoiodevice.h"


VideoIoDevice::VideoIoDevice()
{

}


bool VideoIoDevice::isSequential() const
{
    return true;
}

qint64 VideoIoDevice::readData(char *data, qint64 maxlen)
{
    QMutexLocker(_mtx);

    qint64 bytes_to_read = std::min(maxlen, _buf.size());

    qInfo("readData(%ll), will read %ll", maxlen, bytes_to_read);

    std::memcpy(data, _buf.constData(), bytes_to_read);

    _buf.remove(0, bytes_to_read);

    return bytes_to_read;
}

qint64 VideoIoDevice::writeData(const char *data, qint64 len)
{
    QMutexLocker locker(_mtx);
    qInfo("writeData(%ll)", len);
    _buf.append(data, len);
    readyRead();
}
