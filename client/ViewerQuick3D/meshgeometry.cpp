#include "meshgeometry.h"

MeshGeometry::MeshGeometry()
{

}

QByteArray MeshGeometry::meshFile() const
{
    return m_meshFile;
}

void MeshGeometry::setMeshFile(QByteArray meshFile)
{
    m_meshFile = meshFile;
    updateData();
    emit meshFileChanged();
}

void MeshGeometry::updateData()
{
    clear();

    // define stride: 1 vertex + 1 normal
    int stride = 6*sizeof(float);

    // STL format:
    // UINT8[80]    – Header                 -     80 bytes
    // UINT32       – Number of triangles    -      4 bytes
    // foreach triangle                      - 50 bytes:
    //     REAL32[3] – Normal vector             - 12 bytes
    //     REAL32[3] – Vertex 1                  - 12 bytes
    //     REAL32[3] – Vertex 2                  - 12 bytes
    //     REAL32[3] – Vertex 3                  - 12 bytes
    //     UINT16    – Attribute byte count      -  2 bytes
    // end

    qInfo("mesh size = %d bytes", m_meshFile.size());

    if(m_meshFile.size() < 84)
    {
        qWarning("Bad mesh file size: %d < 84",
                 m_meshFile.size());
        return;
    }

    int readIdx = 0;

    // skipheader
    auto bheader = m_meshFile.sliced(readIdx, 80);
    readIdx += 80;
    Q_UNUSED(bheader);

    // read number of triangles
    auto bnumTriangles = m_meshFile.sliced(readIdx, sizeof(float));
    readIdx += sizeof(float);
    uint32_t numTriangles = *reinterpret_cast<uint32_t*>(bnumTriangles.data());
    qInfo("mesh triangles = %d", numTriangles);

    QByteArray vertices;

    if(m_meshFile.size() < numTriangles*50 + 84)
    {
        qWarning("Bad mesh file size: %d < %d*50 + 84",
                 m_meshFile.size(), numTriangles);
        return;
    }

    for(int i = 0; i < numTriangles; i++)
    {
//        for(int j = 0; j < 12; j++)
//        {
//            float d = *reinterpret_cast<float*>(m_meshFile.data() + readIdx + j*4);
//            qInfo() << d << " ";
//        }

//        qInfo("*****");

        // extract normal
        auto bNormal = m_meshFile.sliced(readIdx, 3*sizeof(float));
        readIdx += 3*sizeof(float);

        // extract v1
        auto bV1 = m_meshFile.sliced(readIdx, 3*sizeof(float));
        readIdx += 3*sizeof(float);

        // extract v2
        auto bV2 = m_meshFile.sliced(readIdx, 3*sizeof(float));
        readIdx += 3*sizeof(float);

        // extract v3
        auto bV3 = m_meshFile.sliced(readIdx, 3*sizeof(float));
        readIdx += 3*sizeof(float);

        // append in vertex format n-v-n-v-n-v
        vertices.append(bV1);
        vertices.append(bNormal);

        vertices.append(bV2);
        vertices.append(bNormal);

        vertices.append(bV3);
        vertices.append(bNormal);

        // attribute byte count
        readIdx += 2;

    }

    setVertexData(vertices);

    setStride(stride);

    setPrimitiveType(QQuick3DGeometry::PrimitiveType::Triangles);

    addAttribute(QQuick3DGeometry::Attribute::PositionSemantic,
                 0,
                 QQuick3DGeometry::Attribute::F32Type);

    addAttribute(QQuick3DGeometry::Attribute::NormalSemantic,
                 3 * sizeof(float),
                 QQuick3DGeometry::Attribute::F32Type);


    update();
}
