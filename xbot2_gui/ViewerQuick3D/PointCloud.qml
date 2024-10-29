import QtQuick

import QtQuick3D
import QtQuick3D.Helpers

Node {

    property vector3d scale: Qt.vector3d(0.1, 0.1, 0.1)
    property alias instancing: instanceTable


    //
    id: root

    PointCloudInstanceTable {
        id: instanceTable
    }

    Model {
        id: cube
        source: '#Cube'
        scale: root.scale
        instancing: instanceTable
        materials: PrincipledMaterial { baseColor: "white" }
    }

}
