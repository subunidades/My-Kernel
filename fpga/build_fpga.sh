#!/usr/bin/env bash
# FPGA build for the chip on the ULX3S board (Lattice ECP5 LFE5U-85F-6BG381C).
# Open-source flow: Yosys -> nextpnr-ecp5 -> ecppack.
# Run after run.sh (needs systemverilog/MasterChip.topEntity/):  bash fpga/build_fpga.sh
# Results: fpga/out/fpga.bit  and logs  fpga/out/{synth,pnr}.log
set -euo pipefail
cd "$(dirname "$0")/.."

for t in yosys nextpnr-ecp5 ecppack ecppll; do
  command -v "$t" >/dev/null 2>&1 || {
    echo "Installing Yosys / nextpnr-ecp5 / Trellis (needs apt) ..."
    (sudo -n apt-get install -y yosys nextpnr-ecp5 fpga-trellis || apt-get install -y yosys nextpnr-ecp5 fpga-trellis) >/dev/null
    break; }
done

SV=systemverilog/MasterChip.topEntity
[ -d "$SV" ] || { echo "ERROR: $SV not found. Run: bash run.sh"; exit 1; }
OUT=fpga/out; mkdir -p "$OUT"

# 25 MHz board clock -> 10 MHz PLL output (the PLL minimum is 10 MHz); fpga_top.v
# divides it by 3.5 to 2.857 MHz because the chip's combinational path needs ~242 ns
ecppll -i 25 -o 10 -f "$OUT/pll.v" >/dev/null

echo "== synthesis (Yosys) =="
yosys -p "read_verilog -sv $SV/chip_types.sv $SV/chip.sv; read_verilog $OUT/pll.v fpga/fpga_top.v; synth_ecp5 -top fpga_top -json $OUT/fpga.json" > "$OUT/synth.log" 2>&1 \
  || { echo "SYNTHESIS FAILED (see $OUT/synth.log)"; tail -5 "$OUT/synth.log"; exit 1; }
grep -E "^ +(CCU2C|L6MUX21|LUT4|MULT18X18D|PFUMX|TRELLIS_FF|EHXPLLL) " "$OUT/synth.log" | tail -7

echo "== place and route (nextpnr-ecp5, LFE5U-85F, CABGA381, speed 6) =="
if nextpnr-ecp5 --85k --package CABGA381 --speed 6 --json "$OUT/fpga.json" --lpf fpga/ulx3s.lpf \
     --textcfg "$OUT/fpga.cfg" --seed 1 > "$OUT/pnr.log" 2>&1; then PNR=PASS; else PNR=FAIL; fi
grep -E "TRELLIS_COMB:|TRELLIS_FF:|MULT18X18D:|EHXPLLL:|TRELLIS_IO:" "$OUT/pnr.log" | sort -u | sed 's/^Info: *//'
grep -E "Max frequency" "$OUT/pnr.log" | tail -1 | sed 's/^Info: *//; s/^ERROR: *//'
[ "$PNR" = PASS ] || { echo "PLACE AND ROUTE: timing FAILED (see $OUT/pnr.log)"; exit 1; }

echo "== bitstream (ecppack) =="
ecppack --compress "$OUT/fpga.cfg" "$OUT/fpga.bit"
ls -l "$OUT/fpga.bit" | awk '{print "fpga.bit:", $5, "bytes"}'
echo "FPGA BUILD OK. Check fpga/ulx3s.lpf pins against the board schematic before programming."
