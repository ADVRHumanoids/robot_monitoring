import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Common
import "./RecursiveSplitView.js" as Logic

Control {

    property Component delegate: Item {}

    //
    id: root

    property alias orientation: split.orientation

    property int splitId: -1

    property RecursiveSplitView treeParent: null

    property Component wrappedDelegate: Item {
        property int splitId
        property Item treeParent
        SplitView.fillHeight: true
        SplitView.fillWidth: true
        SplitView.preferredHeight: treeParent.height / 2.
        SplitView.preferredWidth: treeParent.width / 2.

        Loader {
            id: delegateLoader
            active: true
            anchors.fill: parent
            Connections {
                target: delegateLoader.item
                ignoreUnknownSignals: true
                function onSplitVertical() {
                    treeParent.splitVertical(splitId)
                }
                function onSplitHorizontal() {
                    treeParent.splitHorizontal(splitId)
                }
                function onCloseSplit() {
                    treeParent.closeSplit(splitId)
                }
            }
        }

        Component.onCompleted: {
            treeParent = root
            delegateLoader.sourceComponent = root.delegate
        }

    }

    property Component self: Loader {
        id: loaderComponent
        property Item treeParent
        SplitView.fillHeight: true
        SplitView.fillWidth: true
        SplitView.preferredHeight: treeParent.height / 2.
        SplitView.preferredWidth: treeParent.width / 2.
    }


    function splitVertical(splitId) {
        _splitInternal(Qt.Vertical, splitId)
    }

    function splitHorizontal(splitId) {
        _splitInternal(Qt.Horizontal, splitId)
    }

    function closeSplit(splitId) {

        let keepId = [1, 0][splitId]

        let keepWdLoader = split.itemAt(keepId)
        split.removeItem(split.itemAt(splitId))

        if(treeParent === null) {

            if(keepWdLoader === null) {
                // recreate root item
                createDelegate(0)
                return
            }
            else {
                // we are the root item, just remove the requested item
                keepWdLoader.item.splitId = 0
                return
            }
        }

        // ask parent to replace us with the keep item
        treeParent.replaceItem(root.splitId, keepWdLoader)

    }

    function replaceItem(idToReplace, itemToReplace) {
        itemToReplace.item.treeParent = root
        itemToReplace.treeParent = root
        let itemToRemove = split.takeItem(idToReplace)
        split.addItem(itemToReplace)
        split.moveItem(1, idToReplace)
        split.itemAt(0).item.splitId = 0
        split.itemAt(1).item.splitId = 1
    }

    function _splitInternal(ori, splitId) {

        if(split.count === 1) {
            orientation = ori
            createDelegate(1)
            return
        }

        // take current delegate
        let delegateWrapperLoader = split.takeItem(splitId)

        // create new RecursiveSplitView, forwarding the delegate
        // it will also instantiate a new delegate upon creation
        let item1 = self.createObject(split, {'treeParent': root})
        item1.setSource('RecursiveSplitView.qml',
                        {
                            'treeParent': root,
                            'orientation': ori,
                            'delegate': root.delegate,
                            'splitId': splitId
                        }
                        )

        if(splitId === 0) {
            split.moveItem(1, 0)
        }

        // add the current delegate to the new RecursiveSplitView
        item1.item._addWrappedDelegateLoader(delegateWrapperLoader)
    }

    function _addWrappedDelegateLoader(loader) {
        split.addItem(loader)
        loader.item.treeParent = root
        loader.item.splitId = 1
        loader.treeParent = root
    }

    function createDelegate(splitId) {
        let wdloader = self.createObject(split, {'treeParent': root})
        wdloader.sourceComponent = root.wrappedDelegate
        wdloader.item.splitId = splitId
        wdloader.item.treeParent = root
    }


    Component.onCompleted: {
        createDelegate(0)
    }

    // contentItem: Item {
    //     implicitHeight: split.implicitHeight
    //     implicitWidth: split.implicitWidth
    //     SplitView {
    //         anchors.fill: parent
    //         anchors.margins: 16
    //         id: split
    //     }
    //     Label {
    //         height: 16
    //         text: `${root} <-- (${root.treeParent} @ ${root.splitId})`
    //     }

    // }

    contentItem: SplitView {
        id: split
    }

}
