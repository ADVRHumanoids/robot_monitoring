#ifndef VIDEOSTREAMPAINTER_H
#define VIDEOSTREAMPAINTER_H

#include <QtQuick/QQuickPaintedItem>
#include <QImage>
#include <QSGNode>
#include <QSGMaterial>
#include <QSGMaterialShader>
#include <QSGGeometry>

#include <theora/codec.h>
#include <theora/theoraenc.h>
#include <theora/theoradec.h>



class VideoStreamPainter : public QQuickItem
{

    Q_OBJECT
    QML_ELEMENT

    QImage _img;

public:

    VideoStreamPainter();

    // QQuickPaintedItem interface
public:
//    void paint(QPainter *painter) override;

public slots:
    void setImage(const QByteArray& image);
    void setTheoraPacket(const QByteArray& msg,
                         int b_o_s, int e_o_s,
                         long granulepos, long packetno);

protected:
    QSGNode *updatePaintNode(QSGNode *, UpdatePaintNodeData *) override;
    void geometryChange(const QRectF &newGeometry, const QRectF &oldGeometry) override;

private:

    bool _geometry_changed = true;
    bool _new_frame_available = false;


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

#endif // VIDEOSTREAMPAINTER_H
