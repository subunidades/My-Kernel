# my-kernel: SE(3) Consensus Kernel

A small Clash (Haskell) design that takes a dual-quaternion pose, projects it
back onto the SE(3) manifold, and registers the result. Everything here is
one repeatable process.

## What the design does (as written in `Consensus.hs`)

- Number format: Q1.15 (`Signed 16`; 32767 is just under 1.0).
- Pose = real quaternion `r` + dual quaternion `d`, 8 x 16 = 128 bits.
- `projectSE3` removes the part of `d` that is parallel to `r`
  (`d' = d - (r.d) r`), so that `r.d = 0` holds for the output.
- `topEntity` = one register on the projected input pose (1 cycle latency).
  It has no internal feedback loop, no saturating arithmetic, and no
  physics/dynamics; those appear in older notes but not in this code.

## Run it (GitHub Codespace)

```bash
bash run.sh
```

First run takes about 20-30 minutes (installs GHC, builds Clash). Re-runs
are fast. The script:

1. installs the toolchain if missing,
2. builds Clash (`cabal build`),
3. synthesizes `Consensus.hs` to SystemVerilog in `verilog/`,
4. writes `makerchip_bundle.sv` (generated code + `sim/wrapper.sv`).

## Verify in Makerchip

1. Open `makerchip_bundle.sv` in the Codespace, Ctrl+A, Ctrl+C.
2. Go to https://www.makerchip.com/ide, delete the sample code, paste.
3. Click Compile. The test feeds an off-manifold pose (r.d = 0.5) and
   passes if the output has |r.d| <= 4 raw units and r unchanged.
4. Open the waveform to inspect `pose_in` / `pose_out`.

## Layout

| Path | Purpose |
| :--- | :--- |
| `Consensus.hs` | The design (only file Clash synthesizes) |
| `sim/wrapper.sv` | Makerchip test wrapper |
| `run.sh` | The whole process |
| `my-kernel.cabal`, `cabal.project`, `bin/Clash.hs` | Build setup for Clash |

## If something fails

Paste the last 30 lines of terminal output; the failing step is named by
the `cabal` or `clash` message.
