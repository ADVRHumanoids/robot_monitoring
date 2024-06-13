#ifndef AUDIOSOURCE_H
#define AUDIOSOURCE_H

#include <QObject>
#include <QQmlEngine>
#include <QAudioSource>
#include <QMediaDevices>
#include <QBuffer>

class AudioSource : public QObject
{
    Q_OBJECT
    QML_ELEMENT
    Q_PROPERTY(qreal level READ level NOTIFY levelChanged FINAL)
    Q_PROPERTY(QStringList devices READ devices NOTIFY devicesChanged FINAL)
    Q_PROPERTY(QString currentDevice READ currentDevice WRITE setCurrentDevice NOTIFY currentDeviceChanged FINAL)
    Q_PROPERTY(int bytesAvailable READ bytesAvailable NOTIFY bytesAvailableChanged FINAL)
    Q_PROPERTY(bool active MEMBER _active NOTIFY activeChanged FINAL)


public:

    AudioSource();

    qreal level() const
    {
        return _level;
    }

    QStringList devices() const
    {
        return _audio_dev_descr;
    }

    QString currentDevice() const
    {
        return _current_device;
    }

    int bytesAvailable() const
    {
        return _buf.size();
    }

    void setCurrentDevice(QString currentDevice)
    {
        int idx = _audio_dev_descr.indexOf(currentDevice);

        if(idx < 0)
        {
            qFatal() << "device '" << currentDevice << "' is undefined";
            return;
        }

        _current_device = currentDevice;

        emit currentDeviceChanged(currentDevice);

        if(_active)
        {
            initializeAudio(_audio_dev[idx]);
        }
    }

    Q_INVOKABLE QByteArray read(qint64 maxBytes)
    {
        auto ret = _buf.first(maxBytes <= 0 ? _buf.size() : std::min(_buf.size(), maxBytes));
        _buf.remove(0, ret.size());
        emit bytesAvailableChanged(_buf.size());
        return ret;
    }

    Q_INVOKABLE QString readBase64(qint64 maxBytes)
    {
        auto ret = _buf.first(maxBytes <= 0 ? _buf.size() : std::min(_buf.size(), maxBytes));
        _buf.remove(0, ret.size());
        emit bytesAvailableChanged(_buf.size());
        return ret.toBase64();
    }

    Q_INVOKABLE void start();

    Q_INVOKABLE void stop();



signals:

    void readyRead();

    void levelChanged(qreal level);

    void devicesChanged(QStringList devs);

    void currentDeviceChanged(QString dev);

    void bytesAvailableChanged(int bytes);

    void activeChanged(bool active);


public slots:


private:

    void checkMicrophonePermissions();

    bool _active = false;

    QBuffer _buf_dev;

    QByteArray _buf, _buf_internal;

    QAudioFormat _format;

    qreal _level = 0;

    void initializeDevices();

    qreal calculateLevel(const char *data, qint64 len) const;

    void initializeAudio(const QAudioDevice &deviceInfo);

    std::unique_ptr<QAudioSource> _audio_src;
    QIODevice * _io_dev = nullptr;
    QStringList _audio_dev_descr;
    QList<QAudioDevice> _audio_dev;
    std::unique_ptr<QMediaDevices> _media_devices;
    QString _current_device;

    QMetaObject::Connection _conn1, _conn2;
};

#endif // AUDIOSOURCE_H
