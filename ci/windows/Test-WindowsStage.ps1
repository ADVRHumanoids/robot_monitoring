param(
    [Parameter(Mandatory = $true)]
    [string]$StageDir
)

$ErrorActionPreference = 'Stop'
$StageDir = (Resolve-Path $StageDir).Path
$Executable = Join-Path $StageDir 'bin\xbot2_gui.exe'
$DiagnosticDir = Join-Path $env:RUNNER_TEMP "xbot2-gui-diagnostics-$([IO.Path]::GetFileName($StageDir))"
New-Item $DiagnosticDir -ItemType Directory -Force | Out-Null

function Write-DependencyDiagnostics {
    $Report = Join-Path $DiagnosticDir 'dependencies.txt'
    $Missing = [System.Collections.Generic.List[string]]::new()
    $StageFiles = @{}
    Get-ChildItem $StageDir -Recurse -File | ForEach-Object { $StageFiles[$_.Name.ToLowerInvariant()] = $_.FullName }

    $Binaries = @(Get-Item $Executable) + @(Get-ChildItem $StageDir -Recurse -File -Filter '*.dll')
    foreach ($Binary in $Binaries) {
        Add-Content $Report "`n===== $($Binary.FullName) ====="
        $Dump = (& dumpbin /dependents $Binary.FullName 2>&1 | Out-String)
        Add-Content $Report $Dump
        foreach ($Match in [regex]::Matches($Dump, '(?im)^\s+([A-Za-z0-9_.-]+\.dll)\s*$')) {
            $Dependency = $Match.Groups[1].Value
            $LowerDependency = $Dependency.ToLowerInvariant()
            $SystemDependency = Join-Path $env:SystemRoot "System32\$Dependency"
            if (-not $StageFiles.ContainsKey($LowerDependency) -and
                -not (Test-Path $SystemDependency) -and
                $LowerDependency -notlike 'api-ms-win-*' -and
                $LowerDependency -notlike 'ext-ms-win-*') {
                $Missing.Add("$($Binary.Name) -> $Dependency")
            }
        }
    }

    $MissingPath = Join-Path $DiagnosticDir 'missing-dependencies.txt'
    if ($Missing.Count -gt 0) {
        $Missing | Sort-Object -Unique | Set-Content $MissingPath
        Write-Host 'Missing packaged dependencies:'
        $Missing | Sort-Object -Unique | ForEach-Object { Write-Host "  $_" }
    } else {
        'No missing dependencies detected by dumpbin.' | Set-Content $MissingPath
        Write-Host 'No missing dependencies detected by dumpbin.'
    }
    Write-Host "Dependency report: $Report"
}

function Write-ApplicationDiagnostics {
    param([string]$Reason)
    $EventReport = Join-Path $DiagnosticDir 'application-events.txt'
    "Reason: $Reason" | Set-Content $EventReport
    try {
        Get-WinEvent -FilterHashtable @{ LogName = 'Application'; StartTime = (Get-Date).AddMinutes(-10) } |
            Where-Object { $_.Message -match 'xbot2_gui|Qt|SideBySide|Application Error' } |
            Select-Object TimeCreated, ProviderName, Id, LevelDisplayName, Message |
            Format-List | Out-File $EventReport -Append -Width 240
    } catch {
        Add-Content $EventReport "Unable to read Application event log: $($_.Exception.Message)"
    }
    Write-Host "Application event report: $EventReport"
}

$RequiredPaths = @(
    $Executable,
    (Join-Path $StageDir 'bin\xbot2_gui_msgs.dll'),
    (Join-Path $StageDir 'bin\Qt6Core.dll'),
    (Join-Path $StageDir 'bin\Qt6WebView.dll'),
    (Join-Path $StageDir 'plugins\platforms\qwindows.dll')
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

$QtWebViewModule = Get-ChildItem $StageDir -Recurse -Directory -Filter QtWebView |
    Where-Object { Test-Path (Join-Path $_.FullName 'qmldir') } |
    Select-Object -First 1
if (-not $QtWebViewModule) {
    throw 'Missing deployed QtWebView QML module metadata'
}

Write-DependencyDiagnostics

$Stdout = Join-Path $env:RUNNER_TEMP 'xbot2-gui-smoke.stdout.log'
$Stderr = Join-Path $env:RUNNER_TEMP 'xbot2-gui-smoke.stderr.log'
Remove-Item $Stdout, $Stderr -Force -ErrorAction SilentlyContinue

$Process = Start-Process -FilePath $Executable -PassThru `
    -RedirectStandardOutput $Stdout -RedirectStandardError $Stderr
Start-Sleep -Seconds 15

if ($Process.HasExited) {
    $ExitCode = [int64]$Process.ExitCode
    $ExitCodeHex = ('{0:X8}' -f ($ExitCode -band 0xffffffff))
    $Output = (Get-Content $Stdout, $Stderr -ErrorAction SilentlyContinue | Out-String)
    Write-Host "XBot2 GUI exit code: $($Process.ExitCode) (0x$ExitCodeHex)"
    Write-ApplicationDiagnostics "Startup exit code $($Process.ExitCode) (0x$ExitCodeHex)"
    throw "XBot2 GUI exited during the startup smoke test with code $($Process.ExitCode) (0x$ExitCodeHex).`n$Output"
}

Stop-Process -Id $Process.Id -Force
$Process.WaitForExit()

$Logs = (Get-Content $Stdout, $Stderr -ErrorAction SilentlyContinue | Out-String)
if ($Logs -match 'could not load the Qt platform plugin|module .* is not installed|error while loading shared libraries|The code execution cannot proceed') {
    Write-ApplicationDiagnostics 'Runtime log matched a missing component'
    throw "XBot2 GUI reported a missing runtime component.`n$Logs"
}
