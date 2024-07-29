import QtQuick

import QtQuick3D
import QtQuick3D.Helpers

Node {

    property vector3d scale: Qt.vector3d(0.05, 0.05, 0.05)
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
