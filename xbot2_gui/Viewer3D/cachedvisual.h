#ifndef CACHEDVISUAL_H
#define CACHEDVISUAL_H

#include <QObject>
#include <QQmlEngine>
#include <QtNetwork>
#include <QTemporaryFile>

class CachedVisual : public QObject
{
    Q_OBJECT
    QML_ELEMENT

public:

    explicit CachedVisual(QObject *parent = nullptr);

    Q_INVOKABLE void clearCache();

    Q_INVOKABLE bool addMesh(QString name, QString url);

    Q_PROPERTY(QUrl file READ file NOTIFY fileChanged);

    Q_PROPERTY(QUrl file READ file NOTIFY fileChanged);

    QUrl file() const;

signals:

    void meshReady();

    void fileChanged(QUrl file);

private:

    void cancelDownload();
    void httpFinished();
    void httpReadyRead();

    QNetworkAccessManager _qnam;
    QScopedPointer<QNetworkReply, QScopedPointerDeleteLater> _reply;
    std::unique_ptr<QFile> _file;
    std::unique_ptr<QTemporaryFile> _tmpfile;
    bool _download_in_progress;
    QString _name;
};

#endif // CACHEDVISUAL_H
