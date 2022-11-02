#ifndef VIDEOIODEVICE_H
#define VIDEOIODEVICE_H

#include <QIODevice>
#include <QMutex>
#include <QMutexLocker>
#include <QByteArray>

class VideoIoDevice : public QIODevice
{

public:

    VideoIoDevice();

    // QIODevice interface
public:
    bool isSequential() const override;

protected:
    qint64 readData(char *data, qint64 maxlen) override;
    qint64 writeData(const char *data, qint64 len) override;

private:

    QMutex _mtx;
    QByteArray _buf;


};

#endif // VIDEOIODEVICE_H
