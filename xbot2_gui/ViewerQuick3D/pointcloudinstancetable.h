#ifndef POINTCLOUDINSTANCETABLE_H
#define POINTCLOUDINSTANCETABLE_H

#include <QtQuick3D/QQuick3DInstancing>

class PointCloudInstanceTable : public QQuick3DInstancing
{
    Q_OBJECT
    QML_ELEMENT

public:

    explicit PointCloudInstanceTable(QQuick3DObject *parent = nullptr);

    Q_INVOKABLE void clear();

    Q_INVOKABLE void addPoint(double x, double y, double z);

    Q_INVOKABLE void commit();


protected:

    QByteArray getInstanceBuffer(int *instanceCount) override;

private:
    int m_instanceCount = 0;
    QByteArray m_instanceData;
    bool m_dirty = true;
    QList<QVector3D> m_points;
};

#endif // POINTCLOUDINSTANCETABLE_H
