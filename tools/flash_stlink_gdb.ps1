param(
    [string]$Elf = "build/Debug/drone-H743.elf",
    [int]$Port = 61234,
    [int]$Frequency = 4000
)

$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $PSScriptRoot
$elfPath = Join-Path $root $Elf
$server = "E:/ST/STM32CubeCLT_1.18.0/STLink-gdb-server/bin/ST-LINK_gdbserver.exe"
$gdb = "E:/ST/STM32CubeCLT_1.18.0/GNU-tools-for-STM32/bin/arm-none-eabi-gdb.exe"
$programmerPath = "E:/ST/STM32CubeCLT_1.18.0/STM32CubeProgrammer/bin"
$log = Join-Path $root "build/Debug/stlink_flash_gdbserver.log"

if (-not (Test-Path $elfPath)) {
    throw "ELF not found: $elfPath"
}
if (-not (Test-Path $server)) {
    throw "ST-LINK_gdbserver not found: $server"
}
if (-not (Test-Path $gdb)) {
    throw "arm-none-eabi-gdb not found: $gdb"
}

Get-Process -Name ST-LINK_gdbserver,arm-none-eabi-gdb -ErrorAction SilentlyContinue |
    Stop-Process -Force -ErrorAction SilentlyContinue

$serverArgs = @(
    "--swd",
    "--frequency", "$Frequency",
    "--port-number", "$Port",
    "--pend-halt-timeout", "8",
    "--initialize-reset",
    "--stm32cubeprogrammer-path", "$programmerPath",
    "--log-file", "$log"
)

$proc = Start-Process -FilePath $server -ArgumentList $serverArgs -PassThru -WindowStyle Hidden
try {
    $ready = $false
    for ($i = 0; $i -lt 30; $i++) {
        if ($proc.HasExited) {
            throw "ST-LINK_gdbserver exited early with code $($proc.ExitCode)"
        }
        try {
            $client = [System.Net.Sockets.TcpClient]::new()
            $async = $client.BeginConnect("127.0.0.1", $Port, $null, $null)
            if ($async.AsyncWaitHandle.WaitOne(200)) {
                $client.EndConnect($async)
                $client.Close()
                $ready = $true
                break
            }
            $client.Close()
        } catch {
        }
        Start-Sleep -Milliseconds 200
    }
    if (-not $ready) {
        throw "ST-LINK_gdbserver did not open port $Port"
    }

    $gdbOutput = & $gdb $elfPath `
        -batch `
        -ex "target extended-remote localhost:$Port" `
        -ex "monitor reset halt" `
        -ex "load" `
        -ex "monitor reset" `
        -ex "detach" `
        -ex "quit" 2>&1
    $gdbOutput | Write-Host
    if (($LASTEXITCODE -ne 0) -or ($gdbOutput -match "Connection timed out|not supported|You can't do that|No executable|Error")) {
        throw "gdb flash failed with exit code $LASTEXITCODE"
    }
}
finally {
    if (-not $proc.HasExited) {
        Stop-Process -Id $proc.Id -Force -ErrorAction SilentlyContinue
    }
}
