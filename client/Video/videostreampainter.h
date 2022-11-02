#ifndef VIDEOSTREAMPAINTER_H
#define VIDEOSTREAMPAINTER_H

#include <QObject>
#include <QtQuick/QQuickPaintedItem>
#include <QImage>
#include <QSGNode>
#include <QSGMaterial>
#include <QSGMaterialShader>
#include <QSGGeometry>

#include <theora/codec.h>
#include <theora/theoraenc.h>
#include <theora/theoradec.h>

#include <QMutex>
#include <QMutexLocker>
#include <QQueue>
#include <QThread>

//#define VSP_USE_PAINTER

#ifdef VSP_USE_PAINTER
#define BASE_CLASS QQuickPaintedItem
#else
#define BASE_CLASS QQuickItem
#endif

class VideoStreamPainter;

class Decoder : public QObject
{

    Q_OBJECT

public:

    Decoder(VideoStreamPainter* parent = nullptr);

    void initTextures(QSGTexture** y, QSGTexture** cb, QSGTexture** cr);

    void addPacket(const QByteArray& msg,
                   int b_o_s, int e_o_s,
                   long granulepos, long packetno);

public slots:

    void onPacketReady();

signals:

    void frameReady(QSGTexture* y, QSGTexture* cb, QSGTexture* cr);

private:

    VideoStreamPainter * _parent;

    struct Packet {
        QByteArray datab64;
        int b_o_s;
        int e_o_s;
        long granulepos;
        long packetno;
    };

    QMutex _mtx;
    QQueue<Packet> _q;

    bool received_header_ = 0;
    bool received_keyframe_ = 0;
    int pplevel_ = 0;
    th_dec_ctx* decoding_context_ = nullptr;
    th_info header_info_;
    th_comment header_comment_;
    th_setup_info* setup_info_ = nullptr;
    th_ycbcr_buffer _latest_decoded_frame;
    QImage _yImage, _cbImage, _crImage;
};

class VideoStreamPainter : public BASE_CLASS
{

    Q_OBJECT
    QML_ELEMENT

    QImage _img;

public:

    VideoStreamPainter();

#ifdef VSP_USE_PAINTER
public:
    void paint(QPainter *painter) override;
#endif

public slots:
    void setImage(const QByteArray& image);
    void setTheoraPacket(const QByteArray& msg,
                         int b_o_s, int e_o_s,
                         long granulepos, long packetno);

signals:
    void packetReady();

#ifndef VSP_USE_PAINTER
protected:
    QSGNode *updatePaintNode(QSGNode *, UpdatePaintNodeData *) override;
    void geometryChange(const QRectF &newGeometry, const QRectF &oldGeometry) override;
#endif

private:

    QThread _thread;
    bool _geometry_changed = true;
    bool _new_frame_available = false;
    QSGTexture *_yTex, *_cbTex, *_crTex;
    Decoder * _decoder;

    void onFrameReady(QSGTexture* y, QSGTexture* cb, QSGTexture* cr);



};

#endif // VIDEOSTREAMPAINTER_H
