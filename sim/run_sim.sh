#!/usr/bin/env bash
# Simulate a Makerchip bundle locally with Verilator and report PASS / FAIL.
# Usage: bash sim/run_sim.sh <bundle.sv> <label> [chip|pose]
#   chip|pose also re-runs with a dump and cross-checks the independent Python model.
# Exit code: 0 pass, 1 fail, 2 skipped (no Verilator).
set -uo pipefail
bundle="$1"; label="$2"; mode="${3:-}"
here="$(cd "$(dirname "$0")" && pwd)"

if ! command -v verilator >/dev/null 2>&1; then
  (sudo -n apt-get install -y verilator || apt-get install -y verilator) >/dev/null 2>&1 || true
fi
command -v verilator >/dev/null 2>&1 || { echo "SIM $label: SKIPPED (Verilator not available)"; exit 2; }

work="$(mktemp -d)"
build() {   # $1 = extra defines
  verilator --binary --timing -Wno-fatal -Wno-WIDTH -Wno-UNOPTFLAT -Wno-TIMESCALEMOD \
            -Wno-CASEINCOMPLETE -Wno-LATCH --top-module tb $1 -Mdir "$work/obj$2" \
            "$here/tb.sv" "$bundle" -o simx >"$work/build$2.log" 2>&1
}
if ! build "" 0; then echo "SIM $label: BUILD ERROR"; grep -iE "error" "$work/build0.log" | head -5; exit 1; fi
result="$("$work/obj0/simx" | grep RESULT)"
echo "SIM $label: $result"
rc=0; case "$result" in *PASS*) ;; *) rc=1 ;; esac

if [ -n "$mode" ] && [ $rc -eq 0 ]; then
  if build "-DDUMP" 1; then
    "$work/obj1/simx" | grep '^D ' > "$work/dump.txt"
    python3 "$here/check_dump.py" "$mode" "$work/dump.txt" | sed "s/^/SIM $label: /" | tee "$work/check.txt"
    grep -q "RESULT: PASS" "$work/check.txt" || rc=1
  else echo "SIM $label: dump build failed"; rc=1; fi
fi
exit $rc
