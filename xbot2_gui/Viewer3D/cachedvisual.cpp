#include "cachedvisual.h"

CachedVisual::CachedVisual(QObject *parent)
    :
    QObject{parent},
    _download_in_progress(false)
{}

void CachedVisual::clearCache()
{
    auto path = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
    qInfo() << "clearing writable location at " << path + "/meshes";

    QDir dir(path + "/meshes");
    if(!dir.removeRecursively())
    {
        qWarning("could not remove folder");
    }
}

bool CachedVisual::addMesh(QString name, QString url)
{
    if(_download_in_progress)
    {
        qWarning() << "canceling in progress download";
        cancelDownload();
    }

    // check present in cache
    auto path = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
    qInfo() << "writable location at " << path;

    // create cache directory
    QDir dir;
    dir.mkpath(path + "/meshes");

    // create file
    QString fileName = path + "/meshes/" + name;

    _file = std::make_unique<QFile>(fileName);

    if(_file->exists() && _file->size() > 0)
    {
        qInfo() << _file->fileName() << " exists with size " << _file->size();

        emit fileChanged(file());
        emit meshReady();
        return true;
    }
    else
    {
        qInfo() << _file->fileName() << " not in cache, will download from server";
    }

    // we download to a temporary file to avoid race conditions
    _tmpfile = std::make_unique<QTemporaryFile>();

    // open for writing
    if(!_tmpfile->open())
    {
        qInfo() << "unable to open file " << _file->fileName();
        return false;
    }

    // start get request to server
    _reply.reset(_qnam.get(QNetworkRequest(url)));

    // request ready (finished)
    connect(_reply.get(), &QNetworkReply::finished, this, &CachedVisual::httpFinished);

    // data available for incremental download
    connect(_reply.get(), &QIODevice::readyRead, this, &CachedVisual::httpReadyRead);

    //
    _download_in_progress = true;

    return true;
}


QUrl CachedVisual::file() const
{
    if(!_file) return QUrl();

    return QUrl::fromLocalFile(_file->fileName());
}

void CachedVisual::cancelDownload()
{
    _reply->abort();

    _tmpfile.reset();

    _download_in_progress = false;

    _reply.reset();
}

void CachedVisual::httpFinished()
{
    qInfo() << _file->fileName() << " download finished";

    // copy tmp file to final destination
    _tmpfile->copy(_file->fileName());
    _tmpfile.reset();

    // check errors
    QNetworkReply::NetworkError error = _reply->error();
    const QString &errorString = _reply->errorString();

    if (error != QNetworkReply::NoError)
    {
        qWarning() << "server error: " << errorString;
        QFile::remove(_file->fileName());
    }
    else
    {
        qInfo() << "ok";
    }

    _reply.reset();

    emit fileChanged(file());
    emit meshReady();

    _download_in_progress = false;

}

void CachedVisual::httpReadyRead()
{
    _tmpfile->write(_reply->readAll());
}