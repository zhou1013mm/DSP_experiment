# run_sim_test.ps1 - run e5_sim_test.dxe on the CCES SC589 simulator and export results
# Requires: sim/build_sim_test.ps1 has produced e5_sim_test.dxe
# NOTE: CCES_runner / chipfactory write %LOCALAPPDATA%\Analog Devices, so this
#       must run outside the restricted sandbox.
param(
    [string]$Cces  = "C:\Analog Devices\CrossCore Embedded Studio 2.9.3",
    [string]$Src   = "D:\DSP_experiment\E5_FADF",
    [string]$Platform = "ADSP-SC589 Functional-Sim",
    [string]$DataDir = "",
    [int]$TimeoutMs = 300000
)
$ErrorActionPreference = "Continue"
$sim = Join-Path $Src "sim"
$dxe = Join-Path $sim "e5_sim_test.dxe"
if (-not (Test-Path $dxe)) { throw "missing $dxe, run build_sim_test.ps1 first" }
if ($DataDir -eq "") { $DataDir = Join-Path $Src "sim\data_sim" }
New-Item -ItemType Directory -Force -Path $DataDir | Out-Null

# CCES_runner creates this directory unconditionally at startup (TPSDK trace log)
$appData = Join-Path $env:LOCALAPPDATA "Analog Devices"
New-Item -ItemType Directory -Force -Path $appData | Out-Null

$runner = Join-Path $Cces "CCES_runner.exe"
$runArgs = @(
    "--target", "ChipFactory Simulator",
    "--platform", $Platform,
    "--processor", "ADSP-SC589",
    "-c", "1,$dxe",
    "-d", "1,out,$DataDir\gSimHhat.dat,hex,gSimHhat,1024,1,e5_sim_done",
    "-d", "1,out,$DataDir\gSimHtrueA.dat,hex,gSimHtrueA,256,1,e5_sim_done",
    "-d", "1,out,$DataDir\gSimHtrueB.dat,hex,gSimHtrueB,1024,1,e5_sim_done",
    "-d", "1,out,$DataDir\gSimMSE.dat,hex,gSimMSE,500,1,e5_sim_done",
    "-d", "1,out,$DataDir\gSimTiming.dat,hex,gSimTiming,16,1,e5_sim_done",
    "-d", "1,out,$DataDir\gSimDebug.dat,hex,gSimDebug,16,1,e5_sim_done",
    "--timeout", "$TimeoutMs"
)
Write-Host "RUN: $runner $($runArgs -join ' ')"
& $runner @runArgs 2>&1 | Select-Object -Last 30
Write-Host "EXIT=$LASTEXITCODE"
Get-ChildItem $DataDir -File | Select-Object Name,Length,LastWriteTime | Format-Table -AutoSize
