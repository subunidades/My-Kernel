#!/usr/bin/env bash
# Single repeatable process: install toolchain (first run only) -> build Clash
# -> synthesize Consensus.hs to SystemVerilog -> bundle for Makerchip.
# Usage: bash run.sh
set -euo pipefail
cd "$(dirname "$0")"

# 1. Toolchain (skipped if already installed)
if [ ! -f "$HOME/.ghcup/env" ]; then
  export BOOTSTRAP_HASKELL_NONINTERACTIVE=1
  curl --proto '=https' --tlsv1.2 -sSf https://get-ghcup.haskell.org | sh
fi
source "$HOME/.ghcup/env"
ghcup install ghc 9.6.6 --set
cabal update

# 2. Build Clash (slow the first time, cached afterwards)
cabal build

# 3. Synthesize to SystemVerilog
rm -rf verilog
cabal run clash -- --systemverilog Consensus.hs

# 4. Bundle generated .sv files (type files first) + test wrapper into one file
OUT=makerchip_bundle.sv
: > "$OUT"
for f in $(find verilog -name '*types*.sv' | sort) \
         $(find verilog -name '*.sv' ! -name '*types*' | sort); do
  cat "$f" >> "$OUT"; echo >> "$OUT"
done
cat sim/wrapper.sv >> "$OUT"

echo
echo "DONE. Open $OUT, copy everything, paste into https://www.makerchip.com/ide"
echo "and click Compile. Expected: the test passes (no 'failed')."
