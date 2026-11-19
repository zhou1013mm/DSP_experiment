<#
  run_hello.ps1 -- load + run sim_hello.dxe on the CCES 2.9.3 ChipFactory
  simulator (SC589 SHARC Core1) and export the globals to a memory datafile.

  Backends
  --------
  chipfactory : standalone instruction-set simulator shipped as
                <CCES>\System\chipfactory.exe. Proves load+run (self-check via
                -exit-value) and prints nothing else with -quiet.
                -quiet is MANDATORY in environments where %LOCALAPPDATA% is not
                writable, otherwise chipfactory aborts while creating the trace
                log directory and dies with 0xC0000409.
  runner      : <CCES>\CCES_runner.exe, target "ChipFactory Simulator".
                This is the ONLY backend that can export memory (-d), because
                chipfactory.exe has no memory-dump option.
                It unconditionally creates %LOCALAPPDATA%\Analog Devices first
                and aborts (0xC0000409) if that is not writable.

  Usage
  -----
    powershell -ExecutionPolicy Bypass -File .\run_hello.ps1
    powershell -ExecutionPolicy Bypass -File .\run_hello.ps1 -Backend runner
    powershell -ExecutionPolicy Bypass -File .\run_hello.ps1 -SimType Cycle-Accurate-Sim
#>
param(
  [ValidateSet("auto","chipfactory","runner")] [string]$Backend = "auto",
  [string]$Cces = "C:\Analog Devices\CrossCore Embedded Studio 2.9.3",
  [string]$Sim = $PSScriptRoot,
  [ValidateSet("Functional-Sim","Cycle-Accurate-Sim")] [string]$SimType = "Functional-Sim",
  [string]$Processor = "ADSP-SC589",
  # core id: the SC589 Cycle-Accurate-Sim chip definition has a single SHARC
  # instance declared with coreid="1", so Core1 programs load on core 1.
  [int]$Core = 1,
  [int]$BoundCycles = 5000000,
  [int]$TimeoutMs = 30000
)
$ErrorActionPreference = "Stop"
$cf     = Join-Path $Cces "System\chipfactory.exe"
$runner = Join-Path $Cces "CCES_runner.exe"
$dxe    = Join-Path $Sim "sim_hello.dxe"
$out    = Join-Path $Sim "hello_out.txt"
if (-not (Test-Path $dxe)) { throw "missing $dxe -- run build_hello.ps1 first" }

Write-Host "== 1) chipfactory standalone: load + run self-check =="
& $cf -xml-path "$Cces" -sim-type $SimType -proc $Processor -quiet `
      -exit-value -bound-exe-time $BoundCycles $dxe
$rc = $LASTEXITCODE
switch ($rc) {
  0          { Write-Host "   self-check PASS (main returned 0)" }
  1          { Write-Host "   self-check: program ran, main returned NON-zero (see sim_hello.c codes)" }
  -1073740791{ Write-Host "   ABORT 0xC0000409: simulator needs a writable %LOCALAPPDATA%\Analog Devices / license storage" }
  default    { Write-Host "   simulator exit code $rc" }
}

Write-Host "== 2) CCES_runner: load + run + export memory (-d) =="
# 80 words from gOut covers gOut[64] (256 B) + the adjacent gCycles[4] (16 B).
$datafile = "$Core,out,$out,hex,gOut,80,1,sim_hello_done"
& $runner --target "ChipFactory Simulator" --platform "ADSP-SC589 $SimType" `
          --processor $Processor -c "$Core,$dxe" -d $datafile --timeout $TimeoutMs
$rc2 = $LASTEXITCODE
if (Test-Path $out) {
  Write-Host ("   exported: $out ({0} bytes)" -f (Get-Item $out).Length)
  python (Join-Path $Sim "verify_hello.py") $out
} else {
  Write-Host "   NO datafile produced (runner exit $rc2)."
  Write-Host "   If the log says 'Failed to enable tracing log ... Analog Devices',"
  Write-Host "   pre-create it once outside the sandbox:"
  Write-Host '     mkdir "%LOCALAPPDATA%\Analog Devices"'
  Write-Host "   CA-sim additionally needs FlexNet trusted storage (C:\ProgramData\FLEXnet)."
}
