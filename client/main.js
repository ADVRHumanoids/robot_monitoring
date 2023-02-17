function handleResponsiveLayout() {
    if(mainWindow.width > 576)
    {
        root.layoutMode = "tablet"
    }
    else
    {
        root.layoutMode = "mobile"
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
    gridLayout.children = root.items
    root.footer.visible = false
}

function setMobileMode() {
    print('setting mobile..')
    gridLayout.children = []
    swipeView.contentChildren = root.items
    root.footer.visible = true

}
