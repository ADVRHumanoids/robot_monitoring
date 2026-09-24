$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
Set-Location $RepoRoot

$CMakeText = Get-Content (Join-Path $RepoRoot 'CMakeLists.txt') -Raw
if ($CMakeText -notmatch 'project\(robot_monitoring VERSION ([0-9]+\.[0-9]+\.[0-9]+)\)') {
    throw 'Unable to read robot_monitoring version from CMakeLists.txt'
}
$Version = $Matches[1]

if ($env:GITHUB_REF_TYPE -eq 'tag' -and $env:GITHUB_REF_NAME -ne "v$Version") {
    throw "Release tag '$($env:GITHUB_REF_NAME)' does not match CMake project version '$Version'"
}

$BuildDir = Join-Path $RepoRoot 'build\windows'
$OutputDir = Join-Path $RepoRoot 'build_output'
$StageDir = Join-Path $OutputDir 'windows-stage'
$InstallerName = "xbot2-gui-$Version-windows-x86_64-setup.exe"
$InstallerPath = Join-Path $OutputDir $InstallerName
$IconPath = Join-Path $RepoRoot 'xbot2_gui\packaging\windows\xbot2-gui.ico'

Remove-Item $BuildDir, $StageDir -Recurse -Force -ErrorAction SilentlyContinue
New-Item $BuildDir, $StageDir -ItemType Directory -Force | Out-Null

foreach ($Tool in 'cl', 'ninja', 'qt-cmake', 'windeployqt', 'makensis', 'dumpbin') {
    if (-not (Get-Command $Tool -ErrorAction SilentlyContinue)) {
        throw "Required Windows build tool is unavailable: $Tool"
    }
}

& qt-cmake -S $RepoRoot -B $BuildDir -G Ninja `
    -DCMAKE_BUILD_TYPE=Release `
    -DXBOT2_GUI_WITH_WEBENGINE=ON `
    "-DCMAKE_INSTALL_PREFIX=$StageDir"
if ($LASTEXITCODE -ne 0) { throw 'CMake configuration failed' }

& cmake --build $BuildDir --parallel
if ($LASTEXITCODE -ne 0) { throw 'Windows build failed' }

& cmake --install $BuildDir --config Release
if ($LASTEXITCODE -ne 0) { throw 'Windows deployment failed' }

# Qt's generated deployment script does not pass --compiler-runtime to
# windeployqt. Bundle the MSVC redistributable DLLs app-locally so the
# installer also works on machines without a system-wide VC runtime.
$VCRuntimeDir = Join-Path $env:VCToolsRedistDir 'x64\Microsoft.VC143.CRT'
if (-not (Test-Path $VCRuntimeDir -PathType Container)) {
    throw "Unable to locate the MSVC runtime directory: $VCRuntimeDir"
}
Copy-Item (Join-Path $VCRuntimeDir '*.dll') (Join-Path $StageDir 'bin') -Force

& (Join-Path $PSScriptRoot 'Test-WindowsStage.ps1') -StageDir $StageDir -ExpectedVersion $Version

$Headers = (& dumpbin /headers (Join-Path $StageDir 'bin\xbot2_gui.exe') | Out-String)
if ($LASTEXITCODE -ne 0 -or $Headers -notmatch 'machine \(x64\)') {
    throw 'The staged executable is not an x64 PE binary'
}

& makensis "/DAPP_VERSION=$Version" "/DSTAGE_DIR=$StageDir" `
    "/DOUTPUT_FILE=$InstallerPath" "/DICON_FILE=$IconPath" `
    (Join-Path $RepoRoot 'xbot2_gui\packaging\windows\installer.nsi')
if ($LASTEXITCODE -ne 0 -or -not (Test-Path $InstallerPath -PathType Leaf)) {
    throw 'NSIS installer generation failed'
}

$InstallRoot = Join-Path $env:RUNNER_TEMP 'xbot2-gui-installed'
Remove-Item $InstallRoot -Recurse -Force -ErrorAction SilentlyContinue
$InstallProcess = Start-Process -FilePath $InstallerPath `
    -ArgumentList @('/S', "/D=$InstallRoot") -Wait -PassThru
if ($InstallProcess.ExitCode -ne 0) {
    throw "Silent installer failed with code $($InstallProcess.ExitCode)"
}

& (Join-Path $PSScriptRoot 'Test-WindowsStage.ps1') -StageDir $InstallRoot -ExpectedVersion $Version

$UninstallKey = 'HKCU:\Software\Microsoft\Windows\CurrentVersion\Uninstall\XBot2GUI'
if ((Get-ItemPropertyValue $UninstallKey -Name InstallLocation) -ne $InstallRoot) {
    throw 'The per-user uninstall registration has an unexpected install location'
}

$Shortcut = Join-Path $env:APPDATA 'Microsoft\Windows\Start Menu\Programs\XBot2 GUI\XBot2 GUI.lnk'
if (-not (Test-Path $Shortcut -PathType Leaf)) {
    throw 'The Start Menu shortcut was not created'
}

$Uninstaller = Join-Path $InstallRoot 'Uninstall.exe'
$UninstallProcess = Start-Process -FilePath $Uninstaller -ArgumentList '/S' -Wait -PassThru
if ($UninstallProcess.ExitCode -ne 0) {
    throw "Silent uninstaller failed with code $($UninstallProcess.ExitCode)"
}
Start-Sleep -Seconds 2

for ($Attempt = 0; $Attempt -lt 10 -and (Test-Path $InstallRoot); $Attempt++) {
    Start-Sleep -Milliseconds 500
}

if ((Test-Path $UninstallKey) -or (Test-Path $Shortcut) -or (Test-Path $InstallRoot)) {
    throw 'The uninstaller did not remove its per-user registration and shortcut'
}

$Hash = (Get-FileHash $InstallerPath -Algorithm SHA256).Hash.ToLowerInvariant()
Set-Content -Path "$InstallerPath.sha256" -Encoding ascii -NoNewline `
    -Value "$Hash  $InstallerName`n"

Write-Host "Created $InstallerPath"
