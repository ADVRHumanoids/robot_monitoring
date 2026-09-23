Unicode True
RequestExecutionLevel user
SetCompressor /SOLID lzma

!include "MUI2.nsh"
!include "LogicLib.nsh"

!ifndef APP_VERSION
    !error "APP_VERSION is required"
!endif
!ifndef STAGE_DIR
    !error "STAGE_DIR is required"
!endif
!ifndef OUTPUT_FILE
    !error "OUTPUT_FILE is required"
!endif
!ifndef ICON_FILE
    !error "ICON_FILE is required"
!endif

!define APP_NAME "XBot2 GUI"
!define APP_PUBLISHER "Istituto Italiano di Tecnologia"
!define APP_REG_KEY "Software\Microsoft\Windows\CurrentVersion\Uninstall\XBot2GUI"

Name "${APP_NAME} ${APP_VERSION}"
OutFile "${OUTPUT_FILE}"
InstallDir "$LOCALAPPDATA\Programs\XBot2 GUI"
InstallDirRegKey HKCU "${APP_REG_KEY}" "InstallLocation"
BrandingText "${APP_NAME}"
ShowInstDetails show
ShowUninstDetails show

VIProductVersion "${APP_VERSION}.0"
VIAddVersionKey /LANG=1033 "ProductName" "${APP_NAME}"
VIAddVersionKey /LANG=1033 "CompanyName" "${APP_PUBLISHER}"
VIAddVersionKey /LANG=1033 "FileDescription" "${APP_NAME} installer"
VIAddVersionKey /LANG=1033 "FileVersion" "${APP_VERSION}"
VIAddVersionKey /LANG=1033 "ProductVersion" "${APP_VERSION}"
VIAddVersionKey /LANG=1033 "LegalCopyright" "Istituto Italiano di Tecnologia"

!define MUI_ICON "${ICON_FILE}"
!define MUI_UNICON "${ICON_FILE}"
!define MUI_ABORTWARNING
!define MUI_FINISHPAGE_RUN "$INSTDIR\bin\xbot2_gui.exe"

!insertmacro MUI_PAGE_WELCOME
!insertmacro MUI_PAGE_DIRECTORY
!insertmacro MUI_PAGE_INSTFILES
!insertmacro MUI_PAGE_FINISH
!insertmacro MUI_UNPAGE_CONFIRM
!insertmacro MUI_UNPAGE_INSTFILES
!insertmacro MUI_LANGUAGE "English"

Function .onInit
    ReadRegStr $R0 HKCU "${APP_REG_KEY}" "UninstallString"
    StrCmp $R0 "" done
    IfSilent uninstall_existing prompt_upgrade

prompt_upgrade:
    MessageBox MB_OKCANCEL|MB_ICONINFORMATION \
        "An existing ${APP_NAME} installation must be removed before continuing." \
        IDOK uninstall_existing IDCANCEL cancel_install

uninstall_existing:
    ExecWait '$R0 /S' $R1
    ${If} $R1 != 0
        MessageBox MB_OK|MB_ICONSTOP "Unable to remove the existing installation (error $R1)."
        Abort
    ${EndIf}
    Goto done

cancel_install:
    Abort

done:
FunctionEnd

Section "XBot2 GUI" SecMain
    SetShellVarContext current
    SetOutPath "$INSTDIR"
    File /r "${STAGE_DIR}\*"
    WriteUninstaller "$INSTDIR\Uninstall.exe"

    CreateDirectory "$SMPROGRAMS\XBot2 GUI"
    CreateShortcut "$SMPROGRAMS\XBot2 GUI\XBot2 GUI.lnk" \
        "$INSTDIR\bin\xbot2_gui.exe" "" "$INSTDIR\bin\xbot2_gui.exe" 0
    CreateShortcut "$SMPROGRAMS\XBot2 GUI\Uninstall XBot2 GUI.lnk" \
        "$INSTDIR\Uninstall.exe"

    WriteRegStr HKCU "${APP_REG_KEY}" "DisplayName" "${APP_NAME}"
    WriteRegStr HKCU "${APP_REG_KEY}" "DisplayVersion" "${APP_VERSION}"
    WriteRegStr HKCU "${APP_REG_KEY}" "Publisher" "${APP_PUBLISHER}"
    WriteRegStr HKCU "${APP_REG_KEY}" "InstallLocation" "$INSTDIR"
    WriteRegStr HKCU "${APP_REG_KEY}" "DisplayIcon" "$INSTDIR\bin\xbot2_gui.exe"
    WriteRegStr HKCU "${APP_REG_KEY}" "UninstallString" "$\"$INSTDIR\Uninstall.exe$\""
    WriteRegDWORD HKCU "${APP_REG_KEY}" "NoModify" 1
    WriteRegDWORD HKCU "${APP_REG_KEY}" "NoRepair" 1
SectionEnd

Section "Uninstall"
    SetShellVarContext current
    Delete "$SMPROGRAMS\XBot2 GUI\XBot2 GUI.lnk"
    Delete "$SMPROGRAMS\XBot2 GUI\Uninstall XBot2 GUI.lnk"
    RMDir "$SMPROGRAMS\XBot2 GUI"
    DeleteRegKey HKCU "${APP_REG_KEY}"
    RMDir /r "$INSTDIR"
SectionEnd
