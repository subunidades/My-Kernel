"""Cross-check a simulation dump against the independent Python model.
Usage: python3 sim/check_dump.py chip|pose dump.txt
Each line: D <twist hex> <en> <rst> <pose_out hex> <reference-state hex>
chip: pose_out is the registered pose.    pose: pose_out is the comb. next pose."""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from model import *

def s16(v): return v - 65536 if v >= 32768 else v
def unpack(h, n):
    v = int(h, 16); return [s16((v >> (16 * (n - 1 - i))) & 0xFFFF) for i in range(n)]
def P(a): return (tuple(a[:4]), tuple(a[4:]))
def flat(p): return list(p[0]) + list(p[1])

mode, path = sys.argv[1], sys.argv[2]
rows = []
for l in open(path):
    if not l.startswith("D "): continue
    _, tw, en, rst, out, ref = l.split()
    rows.append((unpack(tw, 6), int(en), int(rst), unpack(out, 8), unpack(ref, 8)))

IDENT8 = [MAX, 0, 0, 0, 0, 0, 0, 0]
step_fn = (lambda p, t: step(p, t, 4)) if mode == "chip" else pe_step
bad = steps = 0
for t in range(20, len(rows)):
    ptw, pen, prst, _, pref = rows[t - 1]
    tw, en, rst, out, ref = rows[t]
    # state: async reset, else update when enabled, else hold
    if rst or prst:  exp_state = IDENT8
    elif pen:        exp_state = flat(step_fn(P(pref), tuple(ptw))); steps += 1
    else:            exp_state = pref
    # output
    exp_out = exp_state if mode == "chip" else flat(pe_step(P(ref), tuple(tw)))
    if ref != exp_state or out != exp_out:
        bad += 1
        if bad <= 3: print("MISMATCH at row", t, "expected", exp_out, "got", out)

print(f"{mode}: {len(rows)-20} cycles, {steps} update steps checked against the Python model, {bad} mismatches")
if mode == "chip":
    wn = wr = 0; off = 0
    for r in rows[20:]:
        p = P(r[3]); wn = max(wn, abs(nerr(p[0])))
        if not onrail(p): off += 1; wr = max(wr, abs(rdot(p)))
    print(f"worst |r^2-1| = {wn:.5f}; worst |r.d| off-rail = {wr*32768:.1f} LSB; cycles off-rail {off}")
print("RESULT:", "PASS" if bad == 0 else "FAIL")
sys.exit(0 if bad == 0 else 1)
