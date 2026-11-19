# build_sim_test.ps1 —— 编译并链接 E5 FDAF 算法的 SHARC 模拟器测试程序
#   复用参考工程已编好的 startup object + app.ldf（只读），不修改 C:\Analog Devices
param(
    [string]$Cces = "C:\Analog Devices\CrossCore Embedded Studio 2.9.3",
    [string]$Ref  = "C:\Analog Devices\ADSP-SC5xx_EZ-KIT_Lite-Rel2.0.2\ADSP-SC5xx_EZ-KIT\Examples\drivers\adc\adau1979\adc_dac_playback\SC589\sharc\adc_dac_playback_Core1",
    [string]$SdkInc = "C:\Analog Devices\ADSP-SC5xx_EZ-KIT_Lite-Rel2.0.2\ADSP-SC5xx_EZ-KIT\include",
    [string]$Src = "D:\DSP_experiment\E5_FADF",
    [int]$BenchBlocks = 100
)
$ErrorActionPreference = "Stop"
$cc  = Join-Path $Cces "cc21k.exe"
$sim = Join-Path $Src "sim"
$inc = Join-Path $Src "adc_dac_playback"
$doj = Join-Path $sim "e5_sim_test.doj"
$dxe = Join-Path $sim "e5_sim_test.dxe"

& $cc -c -proc ADSP-SC589 -si-revision any -g `
    -DCORE1 -D_DEBUG -DADI_DEBUG "-DE5_BENCH_BLOCKS=$BenchBlocks" `
    -structs-do-not-overlap -no-const-strings -no-multiline -warn-protos `
    -double-size-32 -char-size-8 -swc `
    "-I$($Ref)\system" "-I$SdkInc" "-I$Cces\SHARC\include" "-I$inc" `
    -o $doj (Join-Path $sim "e5_sim_test.c")
if ($LASTEXITCODE -ne 0) { throw "compile failed" }

& $cc -proc ADSP-SC589 -si-revision any `
    -T "$($Ref)\system\startup_ldf\app.ldf" -no-mem `
    -L "$($Ref)\Debug\system\startup_ldf" `
    -flags-link '-MDCORE1,-MDDEBUG,-MDOTHERCORE' `
    "-flags-link" "-i,$($Ref)\system" `
    -add-debug-libpaths `
    -o $dxe $doj `
    "$($Ref)\Debug\system\startup_ldf\app_IVT.doj" `
    "$($Ref)\Debug\system\startup_ldf\app_heaptab.doj" `
    "$($Ref)\Debug\system\startup_ldf\app_startup.doj" `
    -flags-link '-od,.\'
if ($LASTEXITCODE -ne 0) { throw "link failed" }

Write-Host "DXE ok: $dxe ($((Get-Item $dxe).Length) bytes)"
