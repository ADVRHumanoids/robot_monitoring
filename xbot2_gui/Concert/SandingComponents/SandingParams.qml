import QtQuick

ListModel {
    ListElement {
        name: "Velocity"
        icon: '../icons/speed.png'
        units: "[cm/s]"
        initialValue: 2
        min: 0.1
        max: 5
        increment: 0.1
    }
    ListElement {
        name: "Force"
        icon: "../icons/bolt.png"
        units: "[N]"
        initialValue: 30
        min: 10
        max: 100
        increment: 1
    }
}
