#include "videostreampainter.h"
#include <QPainter>
#include <QQuickWindow>

class YuvShader : public QSGMaterialShader
{

public:

    YuvShader();

    bool updateUniformData(RenderState &state,
                           QSGMaterial *newMaterial,
                           QSGMaterial *oldMaterial) override;

    void updateSampledImage(RenderState &state,
                            int binding,
                            QSGTexture **texture,
                            QSGMaterial *newMaterial,
                            QSGMaterial *oldMaterial) override;
};


class YuvMaterial : public QSGMaterial
{

public:

    YuvMaterial();

    QSGMaterialType *type() const override;

    int compare(const QSGMaterial *other) const override;

    QSGMaterialShader* createShader(QSGRendererInterface::RenderMode) const override;

    QSGTexture *yTex = nullptr, *cbTex = nullptr, *crTex = nullptr;

};

class YuvNode : public QSGGeometryNode
{

public:

    YuvNode();

    void setRect(const QRectF &bounds);

    void setYuvBuffer(QSGTexture* y,
                      QSGTexture* cb,
                      QSGTexture* cr);



};


VideoStreamPainter::VideoStreamPainter()
{
    _img = QImage(640, 480, QImage::Format::Format_RGB888);
    _img.fill(QColor::fromRgb(100, 100, 100));

    _yImage = QImage(640, 480, QImage::Format::Format_Grayscale8);
    _cbImage = QImage(320, 240, QImage::Format::Format_Grayscale8);
    _crImage = QImage(320, 240, QImage::Format::Format_Grayscale8);
    _yImage.fill(64);
    _cbImage.fill(64);
    _crImage.fill(64);

    th_info_init(&header_info_);
    th_comment_init(&header_comment_);

    if(th_version_number() != (3<<16)+(2<<8)+1)
    {
        throw std::runtime_error("fuck");
    }

    qInfo("ALL OK");

    setFlag(ItemHasContents, true);
}


//void VideoStreamPainter::paint(QPainter *painter)
//{
//    QRect rect(x(), y(), width(), height());
//    painter->drawImage(rect, _img);
//}

void VideoStreamPainter::setImage(const QByteArray &image)
{
    if(!_img.loadFromData(QByteArray::fromBase64(image), "jpeg"))
    {
        qInfo("FAILED \n");
    }
}

namespace {

void YCrCb2RGB(uint8_t y, uint8_t cr, uint8_t cb,
               uint8_t& r, uint8_t& g, uint8_t& b)
{
    double Y = (double) y;
    double Cb = (double) cb;
    double Cr = (double) cr;

    int _r = (int) (Y + 1.40200 * (Cr - 0x80));
    int _g = (int) (Y - 0.34414 * (Cb - 0x80) - 0.71414 * (Cr - 0x80));
    int _b = (int) (Y + 1.77200 * (Cb - 0x80));

    r = std::max(0, std::min(255, _r));
    g = std::max(0, std::min(255, _g));
    b = std::max(0, std::min(255, _b));
}

void theora2qimage(const th_ycbcr_buffer& ycbcr_buffer,
                   QImage& qimg)
{
    auto& y_data = ycbcr_buffer[0];
    auto& cb_data = ycbcr_buffer[1];
    auto& cr_data = ycbcr_buffer[2];

    // to rgb
    for(int line = 0; line < y_data.height; line++)
    {
        for(int i = 0; i < y_data.width; i++)
        {
            uint8_t r, g, b;
            YCrCb2RGB(y_data.data[y_data.stride*line + i],
                    cr_data.data[cr_data.stride*(line/2) + i/2],
                    cb_data.data[cb_data.stride*(line/2) + i/2],
                    r, g, b);
            qimg.setPixel(i,
                          line,
                          // y_data.data[y_data.stride*line + i]
                          (r << 16) + (g << 8) + b
                          );
        }
    }
}

}

using hrc = std::chrono::high_resolution_clock;

void VideoStreamPainter::setTheoraPacket(const QByteArray &datab64,
                                         int b_o_s,
                                         int e_o_s,
                                         long granulepos,
                                         long packetno)
{
    auto tic = hrc::now();

    // create ogg packet
    auto data = QByteArray::fromBase64(datab64);
    ogg_packet oggpacket;
    oggpacket.bytes      = data.size();
    oggpacket.b_o_s      = b_o_s;
    oggpacket.e_o_s      = e_o_s;
    oggpacket.granulepos = granulepos;
    oggpacket.packetno   = packetno;
    oggpacket.packet = reinterpret_cast<unsigned char*>(data.data());

    qInfo("******\nogg_packet %f",
          std::chrono::duration<float>(hrc::now() - tic).count()*1e3);



    tic = hrc::now();

    // beginning of logical stream flag means we're getting new headers
    if (oggpacket.b_o_s == 1) {

        // clear all state, everything we knew is wrong
        received_header_ = false;
        received_keyframe_ = false;
        if (decoding_context_) {
            th_decode_free(decoding_context_);
            decoding_context_ = NULL;
        }
        th_setup_free(setup_info_);
        setup_info_ = NULL;
        th_info_clear(&header_info_);
        th_info_init(&header_info_);
        th_comment_clear(&header_comment_);
        th_comment_init(&header_comment_);
        // latest_image_.reset(); @al
    }

    // decode header packets until we get the first video packet
    if (received_header_ == false) {
        int rval = th_decode_headerin(&header_info_, &header_comment_, &setup_info_, &oggpacket);
        switch (rval) {
        case 0:
            // we've received the full header; this is the first video packet.
            decoding_context_ = th_decode_alloc(&header_info_, setup_info_);
            if (!decoding_context_) {
                qCritical("[theora] Decoding parameters were invalid");
                return;
            }
            received_header_ = true;
            // @al pplevel_ = updatePostProcessingLevel(pplevel_);
            break; // Continue on the video decoding
        case TH_EFAULT:
            qWarning("[theora] EFAULT when processing header packet");
            return;
        case TH_EBADHEADER:
            qWarning("[theora] Bad header packet");
            return;
        case TH_EVERSION:
            qWarning("[theora] Header packet not decodable with this version of libtheora");
            return;
        case TH_ENOTFORMAT:
            qWarning("[theora] Packet was not a Theora header");
            return;
        default:
            // If rval > 0, we successfully received a header packet.
            if (rval < 0)
                qWarning("[theora] Error code %d when processing header packet", rval);
            return;
        }
    }

    // wait for a keyframe if we haven't received one yet
    // delta frames are useless to us in that case
    received_keyframe_ = received_keyframe_ || (th_packet_iskeyframe(&oggpacket) == 1);
    if (!received_keyframe_)
        return;

    // we have a video packet we can handle, let's decode it
    int rval = th_decode_packetin(decoding_context_, &oggpacket, NULL);
    switch (rval) {
    case 0:
        break; // yay, we got a frame.. carry on below.
    case TH_DUPFRAME:
        // video data hasn't changed
        qDebug("[theora] Got a duplicate frame");
        return;
    case TH_EFAULT:
        qWarning("[theora] EFAULT processing video packet");
        return;
    case TH_EBADPACKET:
        qWarning("[theora] Packet does not contain encoded video data");
        return;
    case TH_EIMPL:
        qWarning("[theora] The video data uses bitstream features not supported by this version of libtheora");
        return;
    default:
        qWarning("[theora] Error code %d when decoding video packet", rval);
        return;
    }

    // we have a new decoded frame available
    th_decode_ycbcr_out(decoding_context_, _latest_decoded_frame);
    _new_frame_available = true;

    qInfo("theora_dec %f",
          std::chrono::duration<float>(hrc::now() - tic).count()*1e3);

    return;

//    // to qimage
//    tic = hrc::now();
//    theora2qimage(ycbcr_buffer, _img);
//    qInfo("theora2qimage %f",
//          std::chrono::duration<float>(hrc::now() - tic).count()*1e3);

//    // paint
//    tic = hrc::now();
//    update();
//    qInfo("update %f",
//          std::chrono::duration<float>(hrc::now() - tic).count()*1e3);

}

QSGNode *VideoStreamPainter::updatePaintNode(QSGNode * old, QQuickItem::UpdatePaintNodeData *)
{
    auto *node = static_cast<YuvNode *>(old);

    if(!node)
    {
        node = new YuvNode;
        auto yTex = window()->createTextureFromImage(_yImage);
        auto cbTex = window()->createTextureFromImage(_cbImage);
        auto crTex = window()->createTextureFromImage(_crImage);

        node->setYuvBuffer(yTex, cbTex, crTex);

    }

    if(_geometry_changed)
    {
        node->setRect(boundingRect());
        _geometry_changed = false;
    }

    if(_new_frame_available)
    {
        _yImage.loadFromData(_latest_decoded_frame[0].data);
        _cbImage.loadFromData(_latest_decoded_frame[1].data);
        _crImage.loadFromData(_latest_decoded_frame[2].data);

        auto yTex = window()->createTextureFromImage(_yImage);
        auto cbTex = window()->createTextureFromImage(_cbImage);
        auto crTex = window()->createTextureFromImage(_crImage);

        node->setYuvBuffer(yTex, cbTex, crTex);
        _new_frame_available = false;
    }


    return node;
}

void VideoStreamPainter::geometryChange(const QRectF &newGeometry, const QRectF &oldGeometry)
{
    _geometry_changed = true;
    update();
    QQuickItem::geometryChange(newGeometry, oldGeometry);
}


YuvShader::YuvShader()
{
    setShaderFileName(Stage::VertexStage, ":/Video/yuv2rgb.vert.qsb");
    setShaderFileName(Stage::FragmentStage, ":/Video/yuv2rgb.frag.qsb");
}

void YuvShader::updateSampledImage(RenderState &state,
                                   int binding,
                                   QSGTexture **texture,
                                   QSGMaterial *newMaterial,
                                   QSGMaterial *oldMaterial)
{
    auto mat = static_cast<YuvMaterial*>(newMaterial);

    if(binding == 1)  // y channel
    {
        texture[binding] = mat->yTex;
    }

    if(binding == 2)  // u channel
    {
        texture[binding] = mat->cbTex;
    }

    if(binding == 3) // v channel
    {
        texture[binding] = mat->crTex;
    }
}

bool YuvShader::updateUniformData(QSGMaterialShader::RenderState &state,
                                  QSGMaterial *newMaterial,
                                  QSGMaterial *oldMaterial)
{
    bool changed = false;

    QByteArray * buf = state.uniformData();
    Q_ASSERT(buf->size() >= 64);

    if (state.isMatrixDirty()) {
        const QMatrix4x4 m = state.combinedMatrix();
        memcpy(buf->data(), m.constData(), 64);
        changed = true;
    }

    return changed;
}

YuvMaterial::YuvMaterial()
{

}

QSGMaterialType* YuvMaterial::type() const
{
    static QSGMaterialType type;
    return &type;
}

int YuvMaterial::compare(const QSGMaterial *o) const
{
    Q_ASSERT(o && type() == o->type());
    const auto *other = static_cast<const YuvMaterial *>(o);
    return other == this ? 0 : 1; // ### TODO: compare state???
}

QSGMaterialShader *YuvMaterial::createShader(QSGRendererInterface::RenderMode) const
{
    return new YuvShader;
}

YuvNode::YuvNode()
{
    auto *m = new YuvMaterial;
    setMaterial(m);
    setFlag(OwnsMaterial, true);

    QSGGeometry *g = new QSGGeometry(QSGGeometry::defaultAttributes_TexturedPoint2D(), 4);
    QSGGeometry::updateTexturedRectGeometry(g, QRect(), QRect());
    setGeometry(g);
    setFlag(OwnsGeometry, true);

}

void YuvNode::setRect(const QRectF &bounds)
{
    QSGGeometry::updateTexturedRectGeometry(geometry(), bounds, QRectF(0, 0, 1, 1));
    markDirty(QSGNode::DirtyGeometry);
}

void YuvNode::setYuvBuffer(QSGTexture *y, QSGTexture *cb, QSGTexture *cr)
{
    auto yuvMaterial = static_cast<YuvMaterial*>(material());
    yuvMaterial->yTex = y;
    yuvMaterial->cbTex = cb;
    yuvMaterial->crTex = cr;
}

