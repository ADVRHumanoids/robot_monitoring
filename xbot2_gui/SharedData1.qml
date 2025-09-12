pragma Singleton

import QtQuick

import xbot2_gui.msgs

Item {

    id: root

    property jointState latestJointState
    property list<string> jointNames

}
