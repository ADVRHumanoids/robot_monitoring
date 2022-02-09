function handleResponsiveLayout() {
    if(mainWindow.width > 576)
    {
        layoutMode = "tablet"
    }
    else
    {
        layoutMode = "mobile"
    }
}

function setLayoutMode(mode) {
    if(mode === "tablet")
    {
        setTabletMode()
    }
    else if(mode === "mobile")
    {
        setMobileMode()
    }
}

function setTabletMode() {
    print('setting tablet..')
    swipeView.contentChildren = []
    rowLayout.children = [barPlot, jointState, plotter]
    mainWindow.footer.visible = false
}

function setMobileMode() {
    print('setting mobile..')
    swipeView.contentChildren.push(barPlot)
    swipeView.contentChildren.push(jointState)
    swipeView.contentChildren.push(plotter)
    rowLayout.children = []
    mainWindow.footer.visible = true

}
