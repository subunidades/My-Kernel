#!/usr/bin/env bash
# my-kernel: build and verify everything with one command.
#   bash run.sh                run everything
#   SKIP_BUILD=1 bash run.sh   skip the toolchain/Clash steps; re-bundle and re-simulate
#                              the SystemVerilog already in systemverilog/
set -euo pipefail
cd "$(dirname "$0")"

for f in Consensus.hs my-kernel.cabal cabal.project bin/Clash.hs \
         sim/wrapper.sv sim/tb.sv sim/run_sim.sh sim/check_dump.py sim/model.py sim/test_quat.py; do
  [ -f "$f" ] || { echo "ERROR: missing $f (unzip the whole repo and run from its folder)"; exit 1; }
done

# ------------------------------------------------------------ 1-3. toolchain, Clash, synthesis
if [ "${SKIP_BUILD:-0}" != 1 ]; then
  if [ ! -f "$HOME/.ghcup/env" ]; then
    export BOOTSTRAP_HASKELL_NONINTERACTIVE=1
    export BOOTSTRAP_HASKELL_GHC_VERSION=9.6.6
    curl --proto '=https' --tlsv1.2 -sSf https://get-ghcup.haskell.org | sh
  fi
  source "$HOME/.ghcup/env"
  ghcup install ghc 9.6.6 --set
  cabal update
  cabal build

  rm -rf systemverilog
  cabal run clash -- --systemverilog Consensus.hs            # required
  for d in PoseEngine MasterChip; do                        # optional: never stop the script
    if [ -f "$d.hs" ]; then
      echo; echo "=== $d.hs ==="
      if cabal run clash -- --systemverilog "$d.hs"; then echo "$d.hs: compiled."
      else echo "$d.hs: did NOT compile (messages above)."; fi
    fi
  done
fi

# ------------------------------------------------------------ 4. Makerchip bundles
bundle() {   # design  wrapper  output  module-name
  local design="$1" wrapper="$2" out="$3" mod="$4"
  [ -n "$(find systemverilog -path "systemverilog/$design.*" -name '*.sv' 2>/dev/null)" ] || return 1
  : > "$out"
  for f in $(find systemverilog -path "systemverilog/$design.*" -name '*types*.sv' | sort) \
           $(find systemverilog -path "systemverilog/$design.*" -name '*.sv' ! -name '*types*' | sort); do
    cat "$f" >> "$out"; echo >> "$out"
  done
  cat "$wrapper" >> "$out"
  grep -q "^module $mod" "$out" || { echo "ERROR: module $mod missing from $out"; return 1; }
}
bundle Consensus sim/wrapper.sv makerchip_bundle.sv kernel || { echo "ERROR: Consensus bundle failed"; exit 1; }
HAVE_POSE=0; HAVE_CHIP=0
[ -f sim/wrapper_pose.sv ] && bundle PoseEngine sim/wrapper_pose.sv makerchip_pose_bundle.sv pose_engine && HAVE_POSE=1 || true
[ -f sim/wrapper_chip.sv ] && bundle MasterChip sim/wrapper_chip.sv makerchip_chip_bundle.sv chip && HAVE_CHIP=1 || true

# ------------------------------------------------------------ 5. local simulation (Verilator)
echo; echo "=== verification ==="
FAIL=0
run() { local rc=0; bash sim/run_sim.sh "$@" || rc=$?; [ "$rc" -eq 1 ] && FAIL=1; return 0; }
python3 sim/test_quat.py | sed 's/^/MODEL quaternion product: /' || FAIL=1
run makerchip_bundle.sv consensus
[ "$HAVE_POSE" = 1 ] && run makerchip_pose_bundle.sv pose_engine pose
[ "$HAVE_CHIP" = 1 ] && run makerchip_chip_bundle.sv chip chip

echo
echo "Bundles written (each can also be pasted into https://www.makerchip.com/ide):"
ls makerchip_*bundle.sv
if [ "$FAIL" = 0 ]; then echo "ALL CHECKS PASSED."; else echo "SOME CHECKS FAILED (see lines above)."; exit 1; fi
