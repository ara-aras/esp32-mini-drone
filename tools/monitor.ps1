# Interactive serial monitor for the drone: resets the board, shows the boot log,
# then sends whatever you type (help, i2c, selftest, ...). Ctrl+C to quit.
#   powershell -File tools\monitor.ps1            (default COM3)
#   powershell -File tools\monitor.ps1 COM5
#   powershell -File tools\monitor.ps1 -Gyro      3D attitude window (tools\gyro3d.py) instead
param([string]$Port = "COM3", [int]$Baud = 115200, [switch]$Gyro)

if ($Gyro) { python "$PSScriptRoot\gyro3d.py" $Port; exit $LASTEXITCODE }

$p = New-Object System.IO.Ports.SerialPort $Port, $Baud, None, 8, One
$p.DtrEnable = $false; $p.RtsEnable = $false
$p.NewLine = "`n"
try { $p.Open() } catch { Write-Host "Cannot open $Port - close Arduino Serial Monitor first"; exit 1 }
$p.RtsEnable = $true; Start-Sleep -Milliseconds 100; $p.RtsEnable = $false   # pulse EN = reset
Write-Host "Connected to $Port @ $Baud. Board reset. Type a command and press Enter (Ctrl+C quits)."

$line = ""
try {
  while ($true) {
    $s = $p.ReadExisting()
    if ($s) { Write-Host -NoNewline $s }
    while ([Console]::KeyAvailable) {
      $k = [Console]::ReadKey($true)
      if ($k.Key -eq "Enter") { $p.WriteLine($line); Write-Host ""; $line = "" }
      elseif ($k.Key -eq "Backspace") { if ($line.Length) { $line = $line.Substring(0, $line.Length - 1); Write-Host -NoNewline "`b `b" } }
      else { $line += $k.KeyChar; Write-Host -NoNewline $k.KeyChar }
    }
    Start-Sleep -Milliseconds 20
  }
} finally { $p.Close() }
