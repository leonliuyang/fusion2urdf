<#
.SYNOPSIS
Installs this collision-mesh fork alongside the original Fusion 360 exporter.

.DESCRIPTION
Copies URDF_Exporter from this repository to Fusion's Scripts folder as
fusion2urdf_collision. The original URDF_Exporter folder is never modified.
An existing collision-fork installation is moved to a timestamped backup before
the new version is activated.
#>

[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'

$repositoryRoot = $PSScriptRoot
$sourceDirectory = Join-Path $repositoryRoot 'URDF_Exporter'
$fusionScriptsDirectory = 'C:\Users\leonyliu\AppData\Roaming\Autodesk\Autodesk Fusion 360\API\Scripts'
$installationName = 'fusion2urdf_collision'
$targetDirectory = Join-Path $fusionScriptsDirectory $installationName
$stagingDirectory = Join-Path $fusionScriptsDirectory ('.{0}.staging.{1}' -f $installationName, $PID)

if (-not (Test-Path -LiteralPath $sourceDirectory -PathType Container)) {
    throw "Source exporter directory was not found: $sourceDirectory"
}

if (-not (Test-Path -LiteralPath $fusionScriptsDirectory -PathType Container)) {
    throw "Fusion Scripts directory was not found: $fusionScriptsDirectory"
}

if ((Split-Path -Leaf $targetDirectory) -eq 'URDF_Exporter') {
    throw 'Safety check failed: the target must not be the original URDF_Exporter directory.'
}

try {
    New-Item -ItemType Directory -Path $stagingDirectory | Out-Null
    Copy-Item -Path (Join-Path $sourceDirectory '*') -Destination $stagingDirectory -Recurse -Force

    $oldEntry = Join-Path $stagingDirectory 'URDF_Exporter.py'
    $oldManifest = Join-Path $stagingDirectory 'URDF_Exporter.manifest'
    $newEntry = Join-Path $stagingDirectory "$installationName.py"
    $newManifest = Join-Path $stagingDirectory "$installationName.manifest"

    if (-not (Test-Path -LiteralPath $oldEntry -PathType Leaf)) {
        throw "Expected entry script was not found: $oldEntry"
    }
    if (-not (Test-Path -LiteralPath $oldManifest -PathType Leaf)) {
        throw "Expected manifest was not found: $oldManifest"
    }

    Rename-Item -LiteralPath $oldEntry -NewName "$installationName.py"
    Rename-Item -LiteralPath $oldManifest -NewName "$installationName.manifest"

    # Make Fusion dialogs distinguish the fork from the original exporter.
    $entryContent = Get-Content -LiteralPath $newEntry -Raw
    $entryContent = $entryContent.Replace("title = 'Fusion2URDF'", "title = 'Fusion2URDF Collision'")
    Set-Content -LiteralPath $newEntry -Value $entryContent -Encoding utf8

    $backupDirectory = $null
    if (Test-Path -LiteralPath $targetDirectory) {
        $timestamp = Get-Date -Format 'yyyyMMdd-HHmmss'
        $backupDirectory = Join-Path $fusionScriptsDirectory ('.{0}.backup.{1}' -f $installationName, $timestamp)
        Move-Item -LiteralPath $targetDirectory -Destination $backupDirectory
    }

    try {
        Move-Item -LiteralPath $stagingDirectory -Destination $targetDirectory
    }
    catch {
        if ($backupDirectory -and (Test-Path -LiteralPath $backupDirectory)) {
            Move-Item -LiteralPath $backupDirectory -Destination $targetDirectory
        }
        throw
    }

    Write-Host "Installed collision exporter: $targetDirectory" -ForegroundColor Green
    if ($backupDirectory) {
        Write-Host "Previous collision exporter backup: $backupDirectory" -ForegroundColor Yellow
    }
    Write-Host 'Restart Fusion 360, then run fusion2urdf_collision from Scripts and Add-Ins.'
}
finally {
    if (Test-Path -LiteralPath $stagingDirectory) {
        Remove-Item -LiteralPath $stagingDirectory -Recurse -Force
    }
}
