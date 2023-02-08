#include "videostreampainter.h"
#include <QPainter>
#include <QQuickWindow>
#include <QSGTexture>

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

    void setYuvBuffer(QSGTexture* y,
                      QSGTexture* cb,
                      QSGTexture* cr);

    QSGTexture* getY() const { return yTex; }
    QSGTexture* getCb() const { return cbTex; }
    QSGTexture* getCr() const { return crTex; }
    bool tex_changed = false;

private:

    std::list<QSGTexture*> _tex_to_dispose;
    QSGTexture* yTex = nullptr;
    QSGTexture* cbTex = nullptr;
    QSGTexture* crTex = nullptr;

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
    _decoder = new Decoder(this);

    connect(_decoder, &Decoder::frameReady,
            this, &VideoStreamPainter::onFrameReady,
            Qt::QueuedConnection);

    connect(this, &VideoStreamPainter::packetReady,
            _decoder, &Decoder::onPacketReady,
            Qt::QueuedConnection);

    setImplicitSize(640, 480);
    setFlag(ItemHasContents, true);

    _decoder->moveToThread(&_thread);
    _thread.start(QThread::Priority::LowPriority);
}

#ifdef VSP_USE_PAINTER
void VideoStreamPainter::paint(QPainter *painter)
{
    QRect rect(x(), y(), width(), height());
    painter->drawImage(rect, _img);
}
#endif

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

    _decoder->addPacket(datab64, b_o_s, e_o_s, granulepos, packetno);
    packetReady();


#ifdef VSP_USE_PAINTER
    // to qimage
    tic = hrc::now();
    theora2qimage(_latest_decoded_frame, _img);
    qInfo("theora2qimage %f",
          std::chrono::duration<float>(hrc::now() - tic).count()*1e3);

    // paint
    tic = hrc::now();
    update();
    qInfo("update %f",
          std::chrono::duration<float>(hrc::now() - tic).count()*1e3);
#endif

}

#ifndef VSP_USE_PAINTER
QSGNode *VideoStreamPainter::updatePaintNode(QSGNode * old, QQuickItem::UpdatePaintNodeData *)
{
    auto *node = static_cast<YuvNode *>(old);

    if(!node)
    {
        node = new YuvNode;
        _decoder->initTextures(&_yTex, &_cbTex, &_crTex);
        node->setYuvBuffer(_yTex, _cbTex, _crTex);
    }

    if(_geometry_changed)
    {
        node->setRect(boundingRect());
        _geometry_changed = false;
    }

    if(_new_frame_available)
    {
        node->setYuvBuffer(_yTex, _cbTex, _crTex);
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

void VideoStreamPainter::onFrameReady(QSGTexture *y, QSGTexture *cb, QSGTexture *cr)
{
    _new_frame_available = true;

    _yTex = y;
    _cbTex = cb;
    _crTex = cr;

    setImplicitSize(y->textureSize().width(), y->textureSize().height());

    update();
}
#endif

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

    const bool should_commit_tex_op = mat->tex_changed;

    if(binding == 1)  // y channel
    {
        texture[0] = mat->getY();
    }

    if(binding == 2)  // u channel
    {
        texture[0] = mat->getCb();
    }

    if(binding == 3) // v channel
    {
        texture[0] = mat->getCr();
        mat->tex_changed = false;
    }

    if(should_commit_tex_op)
    {
        texture[0]->commitTextureOperations(state.rhi(),
                                            state.resourceUpdateBatch());
    }

}

bool YuvShader::updateUniformData(QSGMaterialShader::RenderState &state,
                                  QSGMaterial *newMaterial,
                                  QSGMaterial *oldMaterial)
{
    bool changed = false;

    QByteArray * buf = state.uniformData();
    Q_ASSERT(buf->size() >= 64);

    if(state.isMatrixDirty())
    {
        const QMatrix4x4 m = state.combinedMatrix();
        memcpy(buf->data(), m.constData(), 64);
        changed = true;
    }

    if(state.isOpacityDirty())
    {
        const float opacity = state.opacity();
        memcpy(buf->data() + 64, &opacity, 4);
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

void YuvMaterial::setYuvBuffer(QSGTexture *y, QSGTexture *cb, QSGTexture *cr)
{
    delete yTex;
    delete cbTex;
    delete crTex;

    yTex = y;
    cbTex = cb;
    crTex = cr;
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

    yuvMaterial->setYuvBuffer(y, cb, cr);

    yuvMaterial->tex_changed = true;
}


Decoder::Decoder(VideoStreamPainter *parent):
    QObject(nullptr)
{
    _parent = parent;

    _yImage = QImage(640, 480, QImage::Format::Format_Grayscale8);
    _cbImage = QImage(320, 240, QImage::Format::Format_Grayscale8);
    _crImage = QImage(320, 240, QImage::Format::Format_Grayscale8);
    _yImage.fill(29);
    _cbImage.fill(255);
    _crImage.fill(107);

    th_info_init(&header_info_);
    th_comment_init(&header_comment_);

    if(th_version_number() != (3<<16)+(2<<8)+1)
    {
        throw std::runtime_error("fuck");
    }

    qInfo("ALL OK");

}

void Decoder::initTextures(QSGTexture **y, QSGTexture **cb, QSGTexture **cr)
{
    auto * win = _parent->window();
    if(y) *y = win->createTextureFromImage(_yImage);
    if(cb) *cb = win->createTextureFromImage(_cbImage);
    if(cr) *cr = win->createTextureFromImage(_crImage);
}


void Decoder::addPacket(const QByteArray &msg, int b_o_s, int e_o_s, long granulepos, long packetno)
{

    Packet pkt
    {
        msg, b_o_s, e_o_s, granulepos, packetno
    };

    QMutexLocker lock(&_mtx);

    _q.enqueue(pkt);
}

void Decoder::onPacketReady()
{
    Packet pkt;

    {
        QMutexLocker lock(&_mtx);

        if(_q.empty())
        {
            return;
        }

        pkt = _q.dequeue();


        if(pkt.datab64.size() == 0)
        {
            return;
        }

    }


    auto tic = hrc::now();

    // create ogg packet
    auto data = QByteArray::fromBase64(pkt.datab64);
    ogg_packet oggpacket;
    oggpacket.bytes      = data.size();
    oggpacket.b_o_s      = pkt.b_o_s;
    oggpacket.e_o_s      = pkt.e_o_s;
    oggpacket.granulepos = pkt.granulepos;
    oggpacket.packetno   = pkt.packetno;
    oggpacket.packet = reinterpret_cast<unsigned char*>(data.data());

    tic = hrc::now();

    // beginning of logical stream flag means we're getting new headers
    if (oggpacket.b_o_s == 1)
    {
        qInfo("[theora] got b_o_s");
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
        qInfo("[theora] processed b_o_s");
    }

    // decode header packets until we get the first video packet
    if (received_header_ == false)
    {
        int rval = th_decode_headerin(&header_info_,
                                      &header_comment_,
                                      &setup_info_,
                                      &oggpacket);

        switch (rval)
        {

        case 0:
            // we've received the full header; this is the first video packet.
            decoding_context_ = th_decode_alloc(&header_info_, setup_info_);
            if (!decoding_context_) {
                qCritical("[theora] Decoding parameters were invalid");
                return;
            }
            received_header_ = true;
            qInfo("theora: got header");
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
    if(!received_keyframe_)
    {
        qInfo("[theora] waiting keyframe");
    }

    if (!received_keyframe_ &&
            th_packet_iskeyframe(&oggpacket) == 1)
    {
        received_keyframe_ = true;
        qInfo("[theora] got keyframe");
    }

    if(!received_keyframe_)
    {
        return;
    }

    // we have a video packet we can handle, let's decode it
    int rval = th_decode_packetin(decoding_context_, &oggpacket, NULL);
    switch (rval) {
    case 0:
        break; // yay, we got a frame.. carry on below.
    case TH_DUPFRAME:
        // video data hasn't changed
        // qDebug("[theora] Got a duplicate frame");
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

    // create texture from it

    // theora's image buffer is not continuous
    // (only line by line)
    auto copy_to_qimage = [](const th_img_plane& th_channel,
            QImage& qimg)
    {
        if(qimg.width() != th_channel.width ||
                qimg.height() != th_channel.height)
        {
            qInfo("resizing textures to %d x %d",
                  th_channel.width, th_channel.height);

            qimg = QImage(th_channel.width,
                          th_channel.height,
                          QImage::Format::Format_Grayscale8);
        }

        for(int line = 0; line < qimg.height(); line++)
        {
            std::memcpy(qimg.scanLine(line),
                        &th_channel.data[line*th_channel.stride],
                    th_channel.width);
        }
    };

    copy_to_qimage(_latest_decoded_frame[0], _yImage);
    copy_to_qimage(_latest_decoded_frame[1], _cbImage);
    copy_to_qimage(_latest_decoded_frame[2], _crImage);

    // note: texture are fetched from render thread
    QMutexLocker lock(&_mtx);

    auto yTex = _parent->window()->createTextureFromImage(_yImage);
    auto cbTex = _parent->window()->createTextureFromImage(_cbImage);
    auto crTex = _parent->window()->createTextureFromImage(_crImage);

    frameReady(yTex, cbTex, crTex);

}
