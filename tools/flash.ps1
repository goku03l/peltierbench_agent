# Compile and flash the v2 firmware over USB.   Usage:  powershell -File tools/flash.ps1 -Port COM5
param([Parameter(Mandatory=$true)][string]$Port, [switch]$CompileOnly,
      [string]$Cli = "C:\arduino-cli_1.5.2-rc.1_Windows_64bit\arduino-cli.exe")
$sketch = Join-Path $PSScriptRoot "..\firmware\peltier_logger_v2"
$fqbn = "esp32:esp32:esp32:PartitionScheme=min_spiffs"
& $Cli compile --fqbn $fqbn --warnings default $sketch
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
if (-not $CompileOnly) { & $Cli upload --fqbn $fqbn --port $Port $sketch }
