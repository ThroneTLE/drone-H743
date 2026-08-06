param(
  [Parameter(ValueFromRemainingArguments = $true)]
  [string[]] $ReplayArgs
)

$ErrorActionPreference = "Stop"

$RepoRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
$VenvDir = Join-Path $RepoRoot ".tmp\rerun_env"
$PythonExe = Join-Path $VenvDir "Scripts\python.exe"
$ReadyStamp = Join-Path $VenvDir ".deps_ready"
$ReplayScript = Join-Path $PSScriptRoot "flight_log_rerun_replay.py"

if (!(Test-Path $PythonExe)) {
  python -m venv $VenvDir
}

if (!(Test-Path $ReadyStamp)) {
  & $PythonExe -m pip install --upgrade pip
  & $PythonExe -m pip install pandas rerun-sdk
  New-Item -ItemType File -Force -Path $ReadyStamp | Out-Null
}

& $PythonExe $ReplayScript @ReplayArgs
exit $LASTEXITCODE
