function onObjectReceived(msg) {
    if(msg.type !== "diagnostics") {
        return
    }
    const cp = currentPath()
    const ep = expandedPaths()
    const sp = selectedPaths()
    treeModel.loadFromDiagnostics(msg)
    restoreExpandedPaths(ep, function() {
        restoreSelectionState(sp, cp)
    })
}

function levelToColor(l) {
    if(l === 0) {
        return "#54E59A"
    }
    if(l === 1) {
        return "#FFC65A"
    }
    if(l === 2) {
        return "#FF647C"
    }
    if(l === 3) {
        return "#8EABB9"
    }
}

function levelToText(l) {
    if(l === 0) {
        return "OK"
    }
    if(l === 1) {
        return "WARNING"
    }
    if(l === 2) {
        return "ERROR"
    }
    if(l === 3) {
        return "STALE"
    }
}

function formatValue(value) {
    const trimmed = value.trim();

    if (/^-?\d+$/.test(trimmed)) {
        return trimmed;
    }

    if (/^-?(?:\d+\.\d*|\.\d+)$/.test(trimmed)) {
        return Number(trimmed).toFixed(2);
    }

    return value;
}

function selectedPaths() {
    const paths = []

    for (let row = 0; row < treeView.rows; ++row) {
        const index = treeView.index(row, 0)
        if (!treeView.selectionModel.isSelected(index))
            continue

        const path = treeView.model.data(index, TreeModel.PathRole)
        if (path)
            paths.push(path)
    }

    return paths
}


function currentPath() {
    if (treeView.currentRow < 0)
        return ""

    const index = treeView.index(treeView.currentRow, 0)
    return treeView.model.data(index, TreeModel.PathRole) || ""
}

function expandedPaths() {
    return Array.from(treeView.expandedPathSet)
}

function rememberVisibleExpandedPaths() {
    for (let row = 0; row < treeView.rows; ++row) {
        if (!treeView.isExpanded(row))
            continue

        const index = treeView.index(row, 0)
        const path = treeView.model.data(index, TreeModel.PathRole)
        if (path)
            treeView.expandedPathSet.add(path)
    }
}

function pathDepth(path) {
    return path.split("/").filter(function(segment) { return segment.length > 0 }).length
}

function ancestorsAreExpanded(path, pathSet) {
    const segments = path.split("/").filter(function(segment) { return segment.length > 0 })
    let ancestor = ""

    for (let i = 0; i < segments.length - 1; ++i) {
        ancestor += "/" + segments[i]
        if (!pathSet.has(ancestor))
            return false
    }

    return true
}

function restoreExpandedPaths(paths, finished) {
    const pathSet = new Set(paths)
    treeView.restoringExpansion = true
    const sortedPaths = paths.slice().sort(function(left, right) {
        return pathDepth(left) - pathDepth(right)
    })

    for (let i = 0; i < sortedPaths.length; ++i) {
        const path = sortedPaths[i]
        if (!ancestorsAreExpanded(path, pathSet))
            continue

        const index = filterModel.indexForPath(path)
        if (!index.valid)
            continue

        treeView.expandToIndex(index)
        const row = treeView.rowAtIndex(index)
        if (row >= 0 && !treeView.isExpanded(row))
            treeView.expand(row)
    }

    Qt.callLater(function() {
        treeView.restoringExpansion = false
        if (finished)
            finished()
    })
}

function restoreSelectionState(paths, current) {
    const pathSet = new Set(paths)
    let currentIndex = null

    treeView.selectionModel.clear()

    for (let row = 0; row < treeView.rows; ++row) {
        const index = treeView.index(row, 0)
        const path = treeView.model.data(index, TreeModel.PathRole)

        if (pathSet.has(path))
            treeView.selectionModel.select(index, ItemSelectionModel.Select)

        if (path === current)
            currentIndex = index
    }

    if (currentIndex)
        treeView.selectionModel.setCurrentIndex(currentIndex, ItemSelectionModel.NoUpdate)
}

function expandAllVisible() {
    const paths = filterModel.expandablePaths()
    for (let i = 0; i < paths.length; ++i)
        treeView.expandedPathSet.add(paths[i])

    treeView.expandRecursively()
}

function setMinimumLevelFilter(level) {
    filterModel.minimumLevel = filterModel.minimumLevel === level ? -1 : level
    Qt.callLater(expandAllVisible)
}

function rememberAncestorPaths(path) {
    const segments = path.split("/").filter(function(segment) { return segment.length > 0 })
    let ancestor = ""

    for (let i = 0; i < segments.length - 1; ++i) {
        ancestor += "/" + segments[i]
        treeView.expandedPathSet.add(ancestor)
    }
}

function focusItem(path, resetFilters) {
    const index = filterModel.indexForPath(path)
    if (!index.valid) {
        if (resetFilters) {
            searchField.text = ""
            filterModel.minimumLevel = -1
            Qt.callLater(function() { focusIssuePath(path, false) })
        }
        return
    }

    rememberAncestorPaths(path)
    treeView.expandToIndex(index)

    const row = treeView.rowAtIndex(index)
    if (row >= 0) {
        treeView.selectionModel.clear()
        treeView.selectionModel.select(index, ItemSelectionModel.Select)
        treeView.selectionModel.setCurrentIndex(index, ItemSelectionModel.NoUpdate)
        treeView.positionViewAtIndex(index, TableView.Contain)
    }
}
