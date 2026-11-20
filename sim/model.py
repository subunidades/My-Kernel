# Bit-accurate Python model of PoseEngine.hs and MasterChip.hs (Q1.15).
# Semantics taken from the generated SystemVerilog:
#   mul : (a*b)>>>15, clamp to +-32767
#   add/sub (SatSymmetric): exact result kept if it fits in 16 bit signed
#       (so -32768 is possible); on overflow clamp to +-32767 by sign.
import random, math
MAX=32767
def clamp(v): return max(-MAX,min(MAX,v))
def mulq(a,b): return clamp((a*b)>>15)
def addq(a,b):
    s=a+b
    if -32768<=s<=32767: return s
    return MAX if s>0 else -MAX
def subq(a,b):
    s=a-b
    if -32768<=s<=32767: return s
    return MAX if s>0 else -MAX
def qmul(p,q):
    w1,x1,y1,z1=p; w2,x2,y2,z2=q; m=mulq
    w=subq(subq(subq(m(w1,w2),m(x1,x2)),m(y1,y2)),m(z1,z2))
    x=subq(addq(addq(m(w1,x2),m(x1,w2)),m(y1,z2)),m(z1,y2))
    y=addq(addq(subq(m(w1,y2),m(x1,z2)),m(y1,w2)),m(z1,x2))
    z=addq(subq(addq(m(w1,z2),m(x1,y2)),m(y1,x2)),m(z1,w2))
    return (w,x,y,z)
def qadd(p,q): return tuple(addq(a,b) for a,b in zip(p,q))
def expmap(tw):
    ax,ay,az,vx,vy,vz=tw
    return (MAX,ax>>1,ay>>1,az>>1),(0,vx>>1,vy>>1,vz>>1)
def normalize(r,iters=1):
    for _ in range(iters):
        n=sum(c*c for c in r)
        c=clamp((2**30-n)>>16)
        r=tuple(addq(a,mulq(a,c)) for a in r)
    return r
def dot(r,d): return clamp(sum(a*b for a,b in zip(r,d))>>15)
def project(r,d):
    dt=dot(r,d)
    return r, tuple(subq(di,mulq(dt,ri)) for di,ri in zip(d,r))
def step(pose,tw,iters=1):
    r,d=pose; rd,dd=expmap(tw)
    nr=qmul(r,rd); nd=qadd(qmul(r,dd),qmul(d,rd))
    return project(normalize(nr,iters),nd)
def nerr(r): return sum(c*c for c in r)/2**30-1
def rdot(p): return sum(a*b for a,b in zip(*p))/2**30
IDENT=((MAX,0,0,0),(0,0,0,0))
def onrail(p): return any(abs(c)>=MAX for c in p[1]) 

# ---------------------------------------------------------------- PoseEngine
def pe_step(pose, tw):
    """PoseEngine update: pose x twist step. No renormalization, no projection."""
    r, d = pose
    rd, dd = expmap(tw)
    return qmul(r, rd), qadd(qmul(r, dd), qmul(d, rd))

def to_float(q): return [c / 32768.0 for c in q]
