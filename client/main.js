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

function setTabletMode() {
    print('setting tablet..')
    swipeView.contentChildren = []
    rowLayout.children = [barPlot, jointState]
    mainWindow.footer.visible = false
}

function setMobileMode() {
    print('setting mobile..')
    swipeView.contentChildren.push(barPlot)
    swipeView.contentChildren.push(jointState)
    rowLayout.children = []
    mainWindow.footer.visible = true

}
