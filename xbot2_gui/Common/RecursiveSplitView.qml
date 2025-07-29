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

    property int childId: -1

    property RecursiveSplitView treeParent: null

    property Component wrappedDelegate: Item {
        property int splitId
        property Item splitItem
        SplitView.fillHeight: true
        SplitView.fillWidth: true
        SplitView.preferredHeight: splitItem.height / 2.
        SplitView.preferredWidth: splitItem.width / 2.

        Loader {
            id: delegateLoader
            active: true
            anchors.fill: parent
            Connections {
                target: delegateLoader.item
                ignoreUnknownSignals: true
                function onSplitVertical() {
                    splitItem.splitVertical(splitId)
                }
                function onSplitHorizontal() {
                    splitItem.splitHorizontal(splitId)
                }
                function onCloseSplit() {
                    splitItem.closeSplit(splitId)
                }
            }
        }

        Component.onCompleted: {
            splitItem = root
            delegateLoader.sourceComponent = root.delegate
        }

    }

    property Component self: Loader {
        id: loaderComponent
        property Item splitItem
        SplitView.fillHeight: true
        SplitView.fillWidth: true
        SplitView.preferredHeight: splitItem.height / 2.
        SplitView.preferredWidth: splitItem.width / 2.
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
            // top level, just delete the requested item
            keepWdLoader.item.splitId = 0
            return
        }

        // ask parent to replace us with the keep item
        treeParent.replaceItem(childId, keepWdLoader)

    }

    function replaceItem(idToReplace, itemToReplace) {
        itemToReplace.item.splitItem = root
        itemToReplace.splitItem = root
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
        let item1 = self.createObject(split, {'splitItem': root})
        item1.setSource('RecursiveSplitView.qml',
                        {
                            'treeParent': root,
                            'orientation': ori,
                            'delegate': root.delegate,
                            'childId': splitId
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
        loader.item.splitItem = root
        loader.item.splitId = 1
        loader.splitItem = root
    }

    function createDelegate(splitId) {
        let wdloader = self.createObject(split, {'splitItem': root})
        wdloader.sourceComponent = root.wrappedDelegate
        wdloader.item.splitId = splitId
        wdloader.item.splitItem = root
    }


    Component.onCompleted: {
        createDelegate(0)
    }

    contentItem: SplitView {

        id: split

    }

}
