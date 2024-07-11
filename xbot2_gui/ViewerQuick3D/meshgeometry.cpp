#include "meshgeometry.h"
#include <QVector3D>
#include <QFile>

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

void MeshGeometry::setUrl(QUrl meshUrl)
{
    QFile mesh(meshUrl.path());
    mesh.open(QFile::OpenModeFlag::ReadOnly);
    setMeshFile(mesh.readAll());
}

void MeshGeometry::updateData()
{
    clear();

    // define stride: 1 vertex + 1 normal
    int stride = 3*sizeof(float);

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

    QVector3D min(1e9, 1e9, 1e9), max(-1e9, -1e9, -1e9);

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

        // print
        auto bNf = (float*)bNormal.data();
        auto bV1f = (float*)bV1.data();
        auto bV2f = (float*)bV2.data();
        auto bV3f = (float*)bV3.data();

        for(auto v : std::vector<float*>{bV1f, bV2f, bV3f})
        {
            for(int i = 0; i < 3; i++)
            {
                min[i] = std::min(min[i], v[i]);
                max[i] = std::max(max[i], v[i]);
            }
        }

        // printf("N  = %f %f %f \n", bNf[0], bNf[1], bNf[2]);
        // printf("V1 = %f %f %f \n", bV1f[0], bV1f[1], bV1f[2]);
        // printf("V2 = %f %f %f \n", bV2f[0], bV2f[1], bV2f[2]);
        // printf("V3 = %f %f %f \n", bV3f[0], bV3f[1], bV3f[2]);

        // append in vertex format n-v-n-v-n-v
        vertices.append(bV1);
        // vertices.append(bNormal);

        vertices.append(bV2);
        // vertices.append(bNormal);

        vertices.append(bV3);
        // vertices.append(bNormal);

        // attribute byte count
        readIdx += 2;

    }

    setBounds(min, max);

    setVertexData(vertices);

    setStride(stride);

    setPrimitiveType(QQuick3DGeometry::PrimitiveType::Triangles);

    addAttribute(QQuick3DGeometry::Attribute::PositionSemantic,
                 0,
                 QQuick3DGeometry::Attribute::F32Type);

    // addAttribute(QQuick3DGeometry::Attribute::NormalSemantic,
    //              3 * sizeof(float),
    //              QQuick3DGeometry::Attribute::F32Type);


    update();
}
