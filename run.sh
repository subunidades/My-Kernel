#!/usr/bin/env bash
# my-kernel: one-file build. Needs only this file plus the .hs designs:
# Consensus.hs (required), PoseEngine.hs and MasterChip.hs (optional). Run from the folder that holds them:
#   bash run.sh
# Does everything: toolchain -> Clash -> SystemVerilog -> Makerchip bundle.
set -euo pipefail
cd "$(dirname "$0")"

[ -f Consensus.hs ] || { echo "Consensus.hs not found in $(pwd)"; exit 1; }

# ---------------------------------------------------------------- 1. toolchain
if [ ! -f "$HOME/.ghcup/env" ]; then
  export BOOTSTRAP_HASKELL_NONINTERACTIVE=1
  export BOOTSTRAP_HASKELL_GHC_VERSION=9.6.6
  curl --proto '=https' --tlsv1.2 -sSf https://get-ghcup.haskell.org | sh
fi
source "$HOME/.ghcup/env"
ghcup install ghc 9.6.6 --set
cabal update

# ---------------------------------------------------------------- 2. project files
mkdir -p bin sim

cat > my-kernel.cabal << '__CABAL__'
cabal-version: 2.4
name: my-kernel
version: 0.1
build-type: Simple

executable clash
  main-is: Clash.hs
  hs-source-dirs: bin
  default-language: Haskell2010
  build-depends: base, clash-ghc ^>=1.8, clash-prelude ^>=1.8,
                 ghc-typelits-natnormalise, ghc-typelits-extra,
                 ghc-typelits-knownnat
__CABAL__

cat > cabal.project << '__PROJECT__'
packages: .
write-ghc-environment-files: always
__PROJECT__

cat > bin/Clash.hs << '__LAUNCHER__'
import Clash.Main (defaultMain)
import System.Environment (getArgs)
main :: IO ()
main = getArgs >>= defaultMain
__LAUNCHER__

cat > sim/wrapper.sv << '__WRAPPER__'
// Makerchip test wrapper for the Consensus kernel (module "kernel").
// Pose bus layout (128 bits, MSB first):
//   [127:112] rW  [111:96] rX  [95:80] rY  [79:64] rZ
//   [63:48]   dW  [47:32]  dX  [31:16] dY  [15:0]  dZ
// Test: feed an off-manifold pose (r = identity, d.w = 0.5, so r.d = 0.5).
// After projection, r.d must be ~0 and r must be unchanged.
module top(input wire clk, input wire reset, input wire [31:0] cyc_cnt,
           output wire passed, output wire failed);

  wire [127:0] pose_in  = {16'sh7FFF, 16'sh0000, 16'sh0000, 16'sh0000,
                           16'sh4000, 16'sh4000, 16'sh0000, 16'sh0000};
  wire [127:0] pose_out;

  kernel dut (.clk(clk), .rst(reset), .en(1'b1),
              .pose_in(pose_in), .pose_out(pose_out));

  wire signed [15:0] o_rW = pose_out[127:112];
  wire signed [15:0] o_rX = pose_out[111:96];
  wire signed [15:0] o_rY = pose_out[95:80];
  wire signed [15:0] o_rZ = pose_out[79:64];
  wire signed [15:0] o_dW = pose_out[63:48];
  wire signed [15:0] o_dX = pose_out[47:32];
  wire signed [15:0] o_dY = pose_out[31:16];
  wire signed [15:0] o_dZ = pose_out[15:0];

  wire signed [31:0] dot_full = o_rW*o_dW + o_rX*o_dX + o_rY*o_dY + o_rZ*o_dZ;
  wire signed [31:0] dot      = dot_full >>> 15;

  wire ok = (o_rW == 16'sh7FFF) && (dot >= -4) && (dot <= 4);

  assign passed = (cyc_cnt >= 32'd10) &&  ok;
  assign failed = (cyc_cnt >= 32'd10) && !ok;
endmodule
__WRAPPER__

cat > sim/wrapper_chip.sv << '__CHIPWRAPPER__'
// Makerchip test wrapper for the combined chip (module "chip").
// twist_in (96 bits, MSB first): wx wy wz vx vy vz, 16 bits each.
// pose_out (128 bits): rW rX rY rZ dW dX dY dZ, 16 bits each.
// Test: constant twist (wx=1000, vy=100) for 120+ cycles. The rotation must
// have moved, |r| must stay within 0.2% of 1, and |r.d| must stay <= 8 LSB.
module top(input wire clk, input wire reset, input wire [31:0] cyc_cnt,
           output wire passed, output wire failed);

  wire [95:0]  twist_in = {16'sh03E8, 16'sh0000, 16'sh0000,
                           16'sh0000, 16'sh0064, 16'sh0000};
  wire [127:0] pose_out;

  chip dut (.clk(clk), .rst(reset), .en(1'b1),
            .twist_in(twist_in), .pose_out(pose_out));

  wire signed [15:0] o_rW = pose_out[127:112];
  wire signed [15:0] o_rX = pose_out[111:96];
  wire signed [15:0] o_rY = pose_out[95:80];
  wire signed [15:0] o_rZ = pose_out[79:64];
  wire signed [15:0] o_dW = pose_out[63:48];
  wire signed [15:0] o_dX = pose_out[47:32];
  wire signed [15:0] o_dY = pose_out[31:16];
  wire signed [15:0] o_dZ = pose_out[15:0];

  wire signed [31:0] nsq  = o_rW*o_rW + o_rX*o_rX + o_rY*o_rY + o_rZ*o_rZ;
  wire signed [31:0] nerr = nsq - 32'sd1073741824;
  wire norm_ok = (nerr > -2147483) && (nerr < 2147483);

  wire signed [31:0] dot_full = o_rW*o_dW + o_rX*o_dX + o_rY*o_dY + o_rZ*o_dZ;
  wire signed [31:0] dot      = dot_full >>> 15;
  wire dot_ok = (dot >= -8) && (dot <= 8);

  wire moved = (o_rW != 16'sh7FFF);
  wire ok = moved && norm_ok && dot_ok;

  assign passed = (cyc_cnt >= 32'd120) &&  ok;
  assign failed = (cyc_cnt >= 32'd120) && !ok;
endmodule
__CHIPWRAPPER__

# ---------------------------------------------------------------- 3. build Clash
cabal build

# ---------------------------------------------------------------- 4. synthesize
rm -rf systemverilog
cabal run clash -- --systemverilog Consensus.hs

# PoseEngine.hs (standalone update engine). Reports the result but never
# stops the script.
if [ -f PoseEngine.hs ]; then
  echo
  echo "=== PoseEngine.hs ==="
  if cabal run clash -- --systemverilog PoseEngine.hs; then
    echo "PoseEngine.hs: compiled."
  else
    echo "PoseEngine.hs: did NOT compile (messages above)."
  fi
fi

# Combined chip (PoseEngine update + Consensus projection). Reports the
# result but never stops the script, so Consensus is unaffected.
CHIP_OK=0
if [ -f MasterChip.hs ]; then
  echo
  echo "=== MasterChip.hs ==="
  if cabal run clash -- --systemverilog MasterChip.hs; then
    CHIP_OK=1
    echo "MasterChip.hs: compiled."
  else
    echo "MasterChip.hs: did NOT compile (messages above). Consensus is unaffected."
  fi
fi

# ---------------------------------------------------------------- 5. Makerchip bundle
OUT=makerchip_bundle.sv
: > "$OUT"
for f in $(find systemverilog/Consensus.* -name '*types*.sv' | sort) \
         $(find systemverilog/Consensus.* -name '*.sv' ! -name '*types*' | sort); do
  cat "$f" >> "$OUT"; echo >> "$OUT"
done
cat sim/wrapper.sv >> "$OUT"

grep -q '^module kernel' "$OUT" || { echo "ERROR: kernel module missing from $OUT"; exit 1; }

if [ "$CHIP_OK" = 1 ]; then
  CHIPOUT=makerchip_chip_bundle.sv
  : > "$CHIPOUT"
  for f in $(find systemverilog/MasterChip.* -name '*types*.sv' | sort) \
           $(find systemverilog/MasterChip.* -name '*.sv' ! -name '*types*' | sort); do
    cat "$f" >> "$CHIPOUT"; echo >> "$CHIPOUT"
  done
  cat sim/wrapper_chip.sv >> "$CHIPOUT"
  grep -q '^module chip' "$CHIPOUT" || { echo "ERROR: chip module missing from $CHIPOUT"; exit 1; }
  echo
  echo "CHIP bundle: $CHIPOUT  (paste into Makerchip; expect: Simulation PASSED!!!)"
fi

echo
echo "DONE. Open $OUT, copy everything, paste into https://www.makerchip.com/ide"
echo "Expected in the Makerchip LOG: Simulation PASSED!!!"
