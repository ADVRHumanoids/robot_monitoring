import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Common

import "Diagnostics.js" as Logic

Control {

    signal focusActiveIssue(string path)

    id: root

    property var model

    contentItem: GridLayout {

        columns: layout.compact ? 1 : 2
        rows: 1

        columnSpacing: 12
        rowSpacing: 12

        IssueListView {
            Layout.fillHeight: true
            Layout.fillWidth: true
            model: root.model
            title: 'Errors'
            level: 2
            onFocusActiveIssue: function(path) {
                root.focusActiveIssue(path)
            }
        }

        IssueListView {
            Layout.fillHeight: true
            Layout.fillWidth: true
            model: root.model
            title: 'Warnings'
            level: 1
            showStale: true
            onFocusActiveIssue: function(path) {
                root.focusActiveIssue(path)
            }
        }
    }
}
