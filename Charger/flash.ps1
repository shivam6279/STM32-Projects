# Build + flash the Charger firmware. Usage:  .\flash.ps1   (add -BuildOnly to skip flashing)
param([switch]$BuildOnly)

$ide  = 'C:\Program Files (x86)\STM32CubeIDE_1.12.1\STM32CubeIDE\plugins'
$gcc  = (Get-ChildItem $ide -Directory -Filter '*gnu-tools-for-stm32*').FullName + '\tools\bin'
$make = (Get-ChildItem $ide -Directory -Filter '*make.win32*').FullName + '\tools\bin\make.exe'
$prog = 'C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe'

$proj = $PSScriptRoot
$elf  = Join-Path $proj 'Debug\Charger.elf'

$env:Path = "$gcc;$env:Path"
& $make -C (Join-Path $proj 'Debug') all
if ($LASTEXITCODE -ne 0) { Write-Host 'BUILD FAILED' -ForegroundColor Red; exit 1 }
if ($BuildOnly) { return }

& $prog -c port=SWD mode=UR -w $elf -v -rst
