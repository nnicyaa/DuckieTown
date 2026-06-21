param(
    [string]$ProjectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
)

$ErrorActionPreference = "Stop"

$ProjectRoot = (Resolve-Path $ProjectRoot).Path
$TargetRoot = Join-Path $PSScriptRoot "target_files"

if (!(Test-Path (Join-Path $ProjectRoot "GodotSimulation")) -or !(Test-Path (Join-Path $ProjectRoot "servers")) -or !(Test-Path (Join-Path $ProjectRoot "tasks"))) {
    throw "ProjectRoot does not look like KvatiTown: $ProjectRoot. Run this from/extract this under your KvatiTown project root, or pass -ProjectRoot C:\path\to\KvatiTown"
}

$Stamp = Get-Date -Format "yyyyMMdd_HHmmss"
$BackupRoot = Join-Path $ProjectRoot "kvati_backup_before_target_$Stamp"
New-Item -ItemType Directory -Force -Path $BackupRoot | Out-Null

$FilesToReplace = @(
    "GodotSimulation\ducky-bot\scenes\convoying.tscn",
    "GodotSimulation\ducky-bot\scripts\LeadTruckDriver.gd",
    "servers\convoying\real_server.py",
    "servers\convoying\virtual_server.py",
    "tasks\convoying\packages\convoy_controller_activity.py",
    "tasks\visual_lane_servoing\packages\agent.py"
)

$FilesToDelete = @(
    "GodotSimulation\ducky-bot\scripts\LeadTruckDriver.gd.uid"
)

foreach ($Rel in $FilesToReplace) {
    $Src = Join-Path $TargetRoot $Rel
    $Dst = Join-Path $ProjectRoot $Rel
    $BackupDst = Join-Path $BackupRoot $Rel

    if (!(Test-Path $Src)) {
        throw "Missing target file in package: $Src"
    }

    New-Item -ItemType Directory -Force -Path (Split-Path $BackupDst) | Out-Null
    if (Test-Path $Dst) {
        Copy-Item -Force $Dst $BackupDst
    }

    New-Item -ItemType Directory -Force -Path (Split-Path $Dst) | Out-Null
    Copy-Item -Force $Src $Dst
    Write-Host "replaced $Rel"
}

foreach ($Rel in $FilesToDelete) {
    $Dst = Join-Path $ProjectRoot $Rel
    $BackupDst = Join-Path $BackupRoot $Rel

    if (Test-Path $Dst) {
        New-Item -ItemType Directory -Force -Path (Split-Path $BackupDst) | Out-Null
        Copy-Item -Force $Dst $BackupDst
        Remove-Item -Force $Dst
        Write-Host "deleted $Rel"
    } else {
        Write-Host "already absent $Rel"
    }
}

Write-Host ""
Write-Host "Done. Backup saved at: $BackupRoot"
Write-Host "Check changes with: git diff --stat"
