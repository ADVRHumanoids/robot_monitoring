#ifndef MEDIAPLAYER_H
#define MEDIAPLAYER_H

#include <QtMultimedia/QMediaPlayer>
#include <QBuffer>
#include <QQmlEngine>

class IODeviceMediaPlayer : public QObject
{
    Q_OBJECT
    QML_ELEMENT

public:


    IODeviceMediaPlayer(QObject * parent = nullptr);

private:

    QMediaPlayer * _player;
    QBuffer _buffer;
};

#endif // MEDIAPLAYER_H
