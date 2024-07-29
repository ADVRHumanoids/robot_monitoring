#include "pointcloudinstancetable.h"

PointCloudInstanceTable::PointCloudInstanceTable(QQuick3DObject *parent):
    QQuick3DInstancing(parent)
{
    m_points.emplace_back(1, 1, 1);
    m_points.emplace_back(1, 1, -1);
    m_points.emplace_back(1, -1, 1);
    m_points.emplace_back(1, -1, -1);
    m_points.emplace_back(-1, -1, 1);
    m_points.emplace_back(-1, 1, -1);
    m_points.emplace_back(-1, -1, -1);
    m_points.emplace_back(-1, 1, 1);
    commit();
}

void PointCloudInstanceTable::clear()
{
    m_points.clear();
}

void PointCloudInstanceTable::addPoint(double x, double y, double z)
{
    m_points.emplace_back(x, y, z);
}

void PointCloudInstanceTable::commit()
{
    markDirty();
    m_dirty = true;
}

QByteArray PointCloudInstanceTable::getInstanceBuffer(int *instanceCount)
{
    if(instanceCount)
    {
        *instanceCount = m_points.size();
    }

    if(!m_dirty)
    {
        return m_instanceData;
    }

    m_dirty = false;

    m_instanceData.clear();

    for(auto& p : m_points)
    {
        int hue = std::clamp<int>(180 + p.z() * 180, 0, 359);

        auto entry = calculateTableEntry(p*100,
                                         {1., 1., 1.},
                                         {0., 0., 0.},
                                         QColor::fromHsv(hue, 255, 255)
                                         );
        m_instanceData.append(reinterpret_cast<const char *>(&entry), sizeof(entry));

    }

    return m_instanceData;
}
