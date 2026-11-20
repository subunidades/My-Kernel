"""Unit test of the Q1.15 quaternion product against exact floating-point maths.
Catches wrong signs (the original z term) and missing rescaling.
Usage: python3 sim/test_quat.py"""
import random, math, sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from model import qmul, mulq

def fmul(p, q):
    w1, x1, y1, z1 = p; w2, x2, y2, z2 = q
    return (w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2)

random.seed(2024)
worst = 0.0
for _ in range(20000):
    # random unit quaternions: the product of two unit quaternions is unit,
    # so no component can leave the Q1.15 range
    def runit():
        v = [random.gauss(0, 1) for _ in range(4)]
        n = math.sqrt(sum(a*a for a in v)); return [a / n for a in v]
    a, b = runit(), runit()
    qa = tuple(max(-32767, min(32767, round(c * 32768))) for c in a)
    qb = tuple(max(-32767, min(32767, round(c * 32768))) for c in b)
    hw = qmul(qa, qb)
    ex = fmul([c / 32768 for c in qa], [c / 32768 for c in qb])
    err = max(abs(h / 32768 - e) for h, e in zip(hw, ex))
    worst = max(worst, err)

# directed: the z term, with i*j = k  and  j*i = -k
I = (0, 32767, 0, 0); J = (0, 0, 32767, 0)
ij = qmul(I, J); ji = qmul(J, I)
ok_dir = ij[3] > 32000 and ji[3] < -32000
# a product of two mid-range values must be rescaled: 0.5 * 0.5 = 0.25
ok_scale = mulq(16384, 16384) == 8192

lsb = 1 / 32768
print(f"worst error vs exact float product over 20000 random unit pairs: {worst/lsb:.2f} LSB")
print("i*j = +k, j*i = -k:", "ok" if ok_dir else "WRONG")
print("0.5 * 0.5 = 0.25 (rescaled):", "ok" if ok_scale else "WRONG")
good = worst < 6 * lsb and ok_dir and ok_scale
print("RESULT:", "PASS" if good else "FAIL")
sys.exit(0 if good else 1)
