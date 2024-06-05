#include "audiosource.h"
#include <QPermissions>
#include <QCoreApplication>

#include <QJniObject>

std::function<void(QByteArray)> __audio_recv_cb = [](QByteArray){};

static void onAudioDataReceivedBase64(JNIEnv * env, jobject /*thiz*/, jbyteArray audio)
{
    jbyte* audio_data =  env->GetByteArrayElements(audio, 0);
    int size = env->GetArrayLength(audio);
    __audio_recv_cb(QByteArray(reinterpret_cast<char*>(audio_data), size));
}


AudioSource::AudioSource():
    _buf_dev(&_buf_internal)
{

#ifdef ANDROID
    QJniObject::callStaticMethod<void>(
        "it/iit/hhcm/xbot2_gui_client/AudioSource",
        "setContext",
        QNativeInterface::QAndroidApplication::context());


    QJniObject devs;

    devs = QJniObject::callStaticObjectMethod(
        "it/iit/hhcm/xbot2_gui_client/AudioSource",
        "getAudioInputDevices",
        "()[Ljava/lang/String;");


    if (devs.isValid()) {
        QJniEnvironment env;
        jobjectArray devsArray = static_cast<jobjectArray>(devs.object());
        const jint size = env->GetArrayLength(devsArray);
        for (int i = 0; i < size; ++i) {
            QString val = QJniObject(env->GetObjectArrayElement(devsArray, i)).toString();
            qInfo() << val;
        }
    }

    const JNINativeMethod methods[] = {
        { "onAudioDataReceivedBase64", "([B)V", (void *)onAudioDataReceivedBase64 }
    };

    bool registered = QJniEnvironment().registerNativeMethods(
        "it/iit/hhcm/xbot2_gui_client/AudioSource", methods,
        std::size(methods));

    if(!registered)
    {
        qFatal() << "error registering native methods";
    }
#endif

    checkMicrophonePermissions();

    _buf_dev.open(QBuffer::ReadOnly);

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

#ifdef ANDROID
void AudioSource::initializeAudio(const QAudioDevice &deviceInfo)
{
    _format.setSampleRate(16000);
    _format.setChannelCount(1);
    _format.setSampleFormat(QAudioFormat::Int16);

    __audio_recv_cb = [this](QByteArray data)
    {
        _buf.append(data);

        auto lvl = calculateLevel(data.constData(), data.size());

        emit levelChanged(lvl);

        _level = lvl;

        emit bytesAvailableChanged(_buf.size());
    };

    QJniObject::callStaticMethod<void>(
        "it/iit/hhcm/xbot2_gui_client/AudioSource",
        "startSoundRecording",
        deviceInfo.id().toInt());
}
#else
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
#endif
