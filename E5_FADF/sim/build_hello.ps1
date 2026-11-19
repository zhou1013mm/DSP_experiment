<#
  build_hello.ps1 -- compile + link sim_hello.dxe for ADSP-SC589 SHARC Core1
  (CCES 2.9.3). Reuses the prebuilt EZ-Kit startup objects / LDF read-only.
  Usage:  powershell -ExecutionPolicy Bypass -File .\build_hello.ps1
#>
param(
  [string]$Cces = "C:\Analog Devices\CrossCore Embedded Studio 2.9.3",
  [string]$Ref  = "C:\Analog Devices\ADSP-SC5xx_EZ-KIT_Lite-Rel2.0.2\ADSP-SC5xx_EZ-KIT\Examples\drivers\adc\adau1979\adc_dac_playback\SC589\sharc\adc_dac_playback_Core1",
  [string]$SdkInc = "C:\Analog Devices\ADSP-SC5xx_EZ-KIT_Lite-Rel2.0.2\ADSP-SC5xx_EZ-KIT\include",
  [string]$Sim  = $PSScriptRoot
)
$ErrorActionPreference = "Stop"
$cc21k = Join-Path $Cces "cc21k.exe"
$sys   = Join-Path $Cces "SHARC\include"
Set-Location $Sim
Write-Host "[1/2] compile sim_hello.c"
$cargs = @(
  "-c","-proc","ADSP-SC589","-si-revision","any","-g",
  "-DCORE1","-D_DEBUG","-DADI_DEBUG",
  "-structs-do-not-overlap","-no-const-strings","-no-multiline","-warn-protos",
  "-double-size-32","-char-size-8","-swc",
  "-I$Ref\system","-I$SdkInc","-I$sys",
  "-o","sim_hello.doj","sim_hello.c"
)
& $cc21k @cargs
if ($LASTEXITCODE -ne 0) { throw "compile failed ($LASTEXITCODE)" }
Write-Host "[2/2] link sim_hello.dxe"
$largs = @(
  "-proc","ADSP-SC589","-si-revision","any",
  "-T","$Ref\system\startup_ldf\app.ldf","-no-mem",
  "-L","$Ref\Debug\system\startup_ldf",
  "-flags-link","-MDCORE1,-MDDEBUG,-MDOTHERCORE",
  "-flags-link","-i,$Ref\system",
  "-add-debug-libpaths",
  "-o","sim_hello.dxe","sim_hello.doj",
  "$Ref\Debug\system\startup_ldf\app_IVT.doj",
  "$Ref\Debug\system\startup_ldf\app_heaptab.doj",
  "$Ref\Debug\system\startup_ldf\app_startup.doj",
  "-flags-link","-od,."
)
& $cc21k @largs
if ($LASTEXITCODE -ne 0) { throw "link failed ($LASTEXITCODE)" }
Write-Host ("DXE ok: " + (Get-Item (Join-Path $Sim "sim_hello.dxe")).Length + " bytes")
