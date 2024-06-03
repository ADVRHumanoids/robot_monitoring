#include "audiosource.h"
#include <QPermissions>
#include <QCoreApplication>

AudioSource::AudioSource()
{
    checkMicrophonePermissions();

    _buf_dev.open(QBuffer::ReadWrite);

    _media_devices = std::make_unique<QMediaDevices>();

    connect(_media_devices.get(), &QMediaDevices::audioInputsChanged,
            [this]()
            {
                initializeDevices();

                qInfo() << "audio device list changed";

                emit devicesChanged(_audio_dev_descr);
            });

    initializeDevices();

    initializeAudio(_audio_dev.last());
}

void AudioSource::checkMicrophonePermissions()
{
    QMicrophonePermission microphonePermission;

    switch (qApp->checkPermission(microphonePermission)) {
    case Qt::PermissionStatus::Undetermined:
        qApp->requestPermission(microphonePermission, this,
                                &AudioSource::checkMicrophonePermissions);
        return;
    case Qt::PermissionStatus::Denied:
        qFatal() << "mic permission denied";
        return;
    case Qt::PermissionStatus::Granted:
        qInfo() << "mic permission granted";
    }

    QBluetoothPermission btPermission;

    switch (qApp->checkPermission(btPermission)) {
    case Qt::PermissionStatus::Undetermined:
        qApp->requestPermission(btPermission, this,
                                &AudioSource::checkMicrophonePermissions);
        return;
    case Qt::PermissionStatus::Denied:
        qFatal() << "bt permission denied";
        return;
    case Qt::PermissionStatus::Granted:
        qInfo() << "bt permission granted";
    }
}

void AudioSource::initializeDevices()
{
    _audio_dev_descr.clear();
    _audio_dev.clear();

    const QAudioDevice& defaultDeviceInfo = QMediaDevices::defaultAudioInput();
    _audio_dev_descr.append(defaultDeviceInfo.description());
    _audio_dev.append(defaultDeviceInfo);

    for(auto& deviceInfo : _media_devices->audioInputs())
    {
        if (deviceInfo != defaultDeviceInfo)
        {
            _audio_dev_descr.append(deviceInfo.description());
            _audio_dev.append(deviceInfo);
        }
    }
}

qreal AudioSource::calculateLevel(const char *data, qint64 len) const
{
    const int channelBytes = _format.bytesPerSample();
    const int sampleBytes = _format.bytesPerFrame();
    const int numSamples = len / sampleBytes;

    float maxValue = 0;
    auto *ptr = reinterpret_cast<const unsigned char *>(data);

    for (int i = 0; i < numSamples; ++i) {
        for (int j = 0; j < _format.channelCount(); ++j) {
            float value = _format.normalizedSampleValue(ptr);

            maxValue = qMax(value, maxValue);
            ptr += channelBytes;
        }
    }
    return maxValue;
}

void AudioSource::initializeAudio(const QAudioDevice &deviceInfo)
{
    _format.setSampleRate(16000);
    _format.setChannelCount(1);
    _format.setSampleFormat(QAudioFormat::Int16);

    if(!deviceInfo.isFormatSupported(_format))
    {
        qFatal() << "device does not support format";
    }

    // if(_io_dev)
    // {
    //     _io_dev->close();
    // }


    if(_audio_src)
    {
        _audio_src->stop();
        _audio_src.reset();
    }

    _audio_src = std::make_unique<QAudioSource>(deviceInfo, _format);

    _io_dev = _audio_src->start();

    // _io_dev = &_buf_dev;

    // _buf_dev.seek(0);

    // _buf_dev.buffer().clear();

    if(_audio_src->error() != QAudio::NoError)
    {
        qFatal() << "error opening audio device";
    }
    else
    {
        qInfo() << "open audio device ok: " << deviceInfo.description();
    }

    disconnect(_conn1);

    disconnect(_conn2);

    _conn1 = connect(_io_dev, &QIODevice::readyRead,
            this, &AudioSource::readyRead);

    _conn2 = connect(_io_dev, &QIODevice::readyRead,
            [this]()
            {
                // _buf_dev.seek(0);

                auto data = _io_dev->readAll();

                // _buf_dev.seek(0);

                // _buf_dev.buffer().clear();

                auto lvl = calculateLevel(data.constData(), data.size());

                emit levelChanged(lvl);

                _level = lvl;

                _buf.append(data);

                emit bytesAvailableChanged(_buf.size());

            });
}
