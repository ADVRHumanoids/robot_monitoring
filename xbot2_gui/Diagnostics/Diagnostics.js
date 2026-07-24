function levelToColor(l) {
    if(l === 0) {
        return "green"
    }
    if(l === 1) {
        return "yellow"
    }
    if(l === 2) {
        return "red"
    }
    if(l === 3) {
        return "blue"
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

        const path = treeModel.data(index, TreeModel.PathRole)
        if (path)
            paths.push(path)
    }

    return paths
}


function currentPath() {
    if (treeView.currentRow < 0)
        return ""

    const index = treeView.index(treeView.currentRow, 0)
    return treeModel.data(index, TreeModel.PathRole) || ""
}

function expandedPaths() {
    const paths = []

    for (let row = 0; row < treeView.rows; ++row) {
        if (!treeView.isExpanded(row))
            continue

        const index = treeView.index(row, 0)
        const path = treeModel.data(index, TreeModel.PathRole)
        if (path)
            paths.push(path)
    }

    return paths
}

function restoreExpandedPaths(paths) {
    const pathSet = new Set(paths)
    let expandedAny = true

    while (expandedAny) {
        expandedAny = false

        for (let row = 0; row < treeView.rows; ++row) {
            const index = treeView.index(row, 0)
            const path = treeModel.data(index, TreeModel.PathRole)

            if (pathSet.has(path) && !treeView.isExpanded(row)) {
                treeView.expand(row)
                expandedAny = true
            }
        }
    }
}

function restoreSelectionState(paths, current) {
    const pathSet = new Set(paths)
    let currentIndex = null

    treeView.selectionModel.clear()

    for (let row = 0; row < treeView.rows; ++row) {
        const index = treeView.index(row, 0)
        const path = treeModel.data(index, TreeModel.PathRole)

        if (pathSet.has(path))
            treeView.selectionModel.select(index, ItemSelectionModel.Select)

        if (path === current)
            currentIndex = index
    }

    if (currentIndex)
        treeView.selectionModel.setCurrentIndex(currentIndex, ItemSelectionModel.NoUpdate)
}
