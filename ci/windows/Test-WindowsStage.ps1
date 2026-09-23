param(
    [Parameter(Mandatory = $true)]
    [string]$StageDir,

    [Parameter(Mandatory = $true)]
    [string]$ExpectedVersion
)

$ErrorActionPreference = 'Stop'
$StageDir = (Resolve-Path $StageDir).Path
$Executable = Join-Path $StageDir 'bin\xbot2_gui.exe'

$RequiredPaths = @(
    $Executable,
    (Join-Path $StageDir 'bin\Qt6Core.dll'),
    (Join-Path $StageDir 'bin\platforms\qwindows.dll')
)

foreach ($Path in $RequiredPaths) {
    if (-not (Test-Path $Path -PathType Leaf)) {
        throw "Missing deployed Windows runtime file: $Path"
    }
}

foreach ($Pattern in 'QtWebEngineProcess.exe', 'qtwebengine_resources.pak', 'vcruntime140*.dll', 'msvcp140*.dll') {
    if (-not (Get-ChildItem $StageDir -Recurse -File -Filter $Pattern | Select-Object -First 1)) {
        throw "Missing deployed Windows runtime component matching: $Pattern"
    }
}

$QtQuickModule = Get-ChildItem $StageDir -Recurse -Directory -Filter QtQuick |
    Where-Object { Test-Path (Join-Path $_.FullName 'qmldir') } |
    Select-Object -First 1
if (-not $QtQuickModule) {
    throw 'Missing deployed QtQuick QML module metadata'
}

$VersionOutput = (& $Executable --version 2>&1 | Out-String).Trim()
if ($LASTEXITCODE -ne 0 -or $VersionOutput -notmatch [regex]::Escape($ExpectedVersion)) {
    throw "Unexpected xbot2_gui --version output: $VersionOutput"
}

$EmbeddedVersion = (Get-Item $Executable).VersionInfo.ProductVersion
if ($EmbeddedVersion -notmatch "^$([regex]::Escape($ExpectedVersion))(\.0)?$") {
    throw "Unexpected embedded executable version: $EmbeddedVersion"
}

$Stdout = Join-Path $env:RUNNER_TEMP 'xbot2-gui-smoke.stdout.log'
$Stderr = Join-Path $env:RUNNER_TEMP 'xbot2-gui-smoke.stderr.log'
Remove-Item $Stdout, $Stderr -Force -ErrorAction SilentlyContinue

$Process = Start-Process -FilePath $Executable -PassThru `
    -RedirectStandardOutput $Stdout -RedirectStandardError $Stderr
Start-Sleep -Seconds 15

if ($Process.HasExited) {
    $Output = (Get-Content $Stdout, $Stderr -ErrorAction SilentlyContinue | Out-String)
    throw "XBot2 GUI exited during the startup smoke test with code $($Process.ExitCode).`n$Output"
}

Stop-Process -Id $Process.Id -Force
$Process.WaitForExit()

$Logs = (Get-Content $Stdout, $Stderr -ErrorAction SilentlyContinue | Out-String)
if ($Logs -match 'could not load the Qt platform plugin|module .* is not installed|error while loading shared libraries|The code execution cannot proceed') {
    throw "XBot2 GUI reported a missing runtime component.`n$Logs"
}
