#ifndef MESHGEOMETRY_H
#define MESHGEOMETRY_H

#include <QQuick3DGeometry>

class MeshGeometry : public QQuick3DGeometry
{
    Q_OBJECT
    QML_NAMED_ELEMENT(MeshGeometry)
    Q_PROPERTY(QByteArray meshFile READ meshFile WRITE setMeshFile NOTIFY meshFileChanged)

public:


    MeshGeometry();

    QByteArray meshFile() const;

    void setMeshFile(QByteArray meshFile);

signals:

    void meshFileChanged();

private:

    void updateData();

    QByteArray m_meshFile;
};

#endif // MESHGEOMETRY_H
