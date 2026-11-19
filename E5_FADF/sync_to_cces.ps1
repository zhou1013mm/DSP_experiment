# sync_to_cces.ps1
# Copy the E5 sources from the workspace into the CCES project source directory.
# Run this in a normal PowerShell (the CCES project lives outside this repo).
$ErrorActionPreference = "Stop"

$src = "D:\DSP_experiment\E5_FADF\adc_dac_playback"
$dst = "C:\Analog Devices\ADSP-SC5xx_EZ-KIT_Lite-Rel2.0.2\ADSP-SC5xx_EZ-KIT\Examples\drivers\adc\adau1979\adc_dac_playback"

if (-not (Test-Path $dst)) {
    throw "CCES project source directory not found: $dst"
}

$files = @("adc_dac_playback.c", "adc_dac_playback.h", "fadf.h")
foreach ($f in $files) {
    $s = Join-Path $src $f
    if (-not (Test-Path $s)) { throw "missing source file: $s" }
    Copy-Item -Force $s (Join-Path $dst $f)
    Write-Host ("copied {0} -> {1}" -f $s, (Join-Path $dst $f))
}

Write-Host ""
Write-Host "Destination files:"
Get-ChildItem $dst -File | Where-Object { $_.Name -match '^(adc_dac_playback\.(c|h)|fadf\.h)$' } |
    Select-Object Name, Length, LastWriteTime | Format-Table -AutoSize

Write-Host "E5_FADF macro state in adc_dac_playback.h:"
Select-String -Path (Join-Path $dst "adc_dac_playback.h") -Pattern '^#define E5_|^//#define E5_|^#define E4_' |
    ForEach-Object { "  " + $_.Line.Trim() }

Write-Host ""
Write-Host "Next: in CCES run Project -> Clean -> Build (Debug), then download the .dxe."
