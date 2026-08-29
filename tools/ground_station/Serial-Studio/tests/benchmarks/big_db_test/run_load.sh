#!/usr/bin/env bash
# Headless load run for the big_db_test project (POSIX: Linux + macOS).
#
# Starts the UDP frame simulator, runs the app headless loading the big project
# against a UDP source, and lets the app quit itself via --exit-after. Used by
# CI for PGO training (heavy transform / dashboard coverage) and as a non-crash
# verification of the optimized binary.
#
# The app must exit through its own event loop: PGO-instrumented builds flush
# profile data (.profraw/.gcda/.pgc) only on a normal exit, so killing it here
# used to silently discard the whole training run. A kill is now the failure
# path, taken only when the app misses its exit window.
#
# Usage: run_load.sh <app-binary> [seconds]
#
# Exits non-zero if the app crashes before the window elapses or has to be
# killed because it failed to quit gracefully within the grace period.
set -u

APP="${1:?app binary path required}"
SECS="${2:-25}"
DIR="$(cd "$(dirname "$0")" && pwd)"
PORT=8080
GRACE=30

# One faulted channel per board type keeps the diagnostics decode path under
# load: healthy boards suppress their diagnostics frame entirely.
python3 "$DIR/big_db_test.py" --host 127.0.0.1 --port "$PORT" --rate 50 \
  --faults 'TA:2:5,TB:1:3,TC:3:7' >/dev/null 2>&1 &
SIM=$!

"$APP" --headless --project "$DIR/big_db_test.ssproj" --udp "$PORT" \
  --exit-after "$SECS" >/dev/null 2>&1 &
APP_PID=$!

i=0
limit=$((SECS + GRACE))
while kill -0 "$APP_PID" 2>/dev/null && [ "$i" -lt "$limit" ]; do
  sleep 1
  i=$((i + 1))
done

rc=0
if kill -0 "$APP_PID" 2>/dev/null; then
  kill -9 "$APP_PID" 2>/dev/null || true
  wait "$APP_PID" 2>/dev/null || true
  echo "big_db_test: app failed to exit gracefully (killed after ${SECS}s + ${GRACE}s grace)"
  rc=1
else
  wait "$APP_PID"
  rc=$?
fi

kill "$SIM" 2>/dev/null || true
wait "$SIM" 2>/dev/null || true

if [ "$rc" -eq 0 ]; then
  echo "big_db_test: load run completed ${SECS}s and exited cleanly"
elif [ "$i" -lt "$SECS" ]; then
  echo "big_db_test: app exited early (rc=$rc)"
fi
exit "$rc"
