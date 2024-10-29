#include "mediaplayer.h"

IODeviceMediaPlayer::IODeviceMediaPlayer(QObject * parent):
    QObject(parent)
{
    _player = new QMediaPlayer(this);
    _player->setSourceDevice(&_buffer);
}

void IODeviceMediaPlayer::sendPacked(const QString &pkt)
{
    auto bytes = QByteArray::fromBase64(pkt.toLatin1());
    _buffer.write(bytes);
}
