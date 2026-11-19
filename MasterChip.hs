{-# LANGUAGE NoImplicitPrelude #-}
{-# LANGUAGE DeriveGeneric #-}
{-# LANGUAGE DeriveAnyClass #-}
{-# LANGUAGE DataKinds #-}
{-# LANGUAGE TypeOperators #-}

{-|
Module      : MasterChip
Description : PoseEngine (update) + Consensus (projection) as one chip.
Precision   : Q1.15 fixed point (Signed 16), symmetric saturation.

Each clock cycle:
  1. delta  = expMap twist                  (small rotation + translation)
  2. pose'  = pose (x) delta                (dual quaternion product)
  3. r'     = four Newton steps to keep |r| = 1
  4. d'     = d - (r . d) r                 (projection onto the SE(3) manifold)
  5. register pose'                         (output = registered pose)

Range limit: translation components (d) saturate at +-1.0 (32767).
-}

module MasterChip where

import Clash.Prelude
import GHC.Generics

type Q15 = Signed 16

data Quaternion = Quaternion
    { qW :: Q15, qX :: Q15, qY :: Q15, qZ :: Q15 }
    deriving (Generic, NFDataX, Show, Eq)

data DualQuaternion = DualQuaternion
    { realPart :: Quaternion, dualPart :: Quaternion }
    deriving (Generic, NFDataX, Show, Eq)

-- | Angular (wx,wy,wz) and linear (vx,vy,vz) velocity, Q1.15 each.
data Twist = Twist
    { wx :: Q15, wy :: Q15, wz :: Q15
    , vx :: Q15, vy :: Q15, vz :: Q15 }
    deriving (Generic, NFDataX, Show, Eq)

-- =================================
-- Q1.15 ARITHMETIC (saturating)
-- =================================

-- | Clamp a wide value to the symmetric Q1.15 range.
sat16 :: Signed 34 -> Q15
sat16 v
  | v > 32767    = 32767
  | v < (-32767) = (-32767)
  | otherwise    = resize v

-- | Exact 32-bit product, widened for summing.
prod34 :: Q15 -> Q15 -> Signed 34
prod34 a b = resize (a `mul` b)

-- | Q1.15 multiply: full product, shift back by 15, saturate.
mulQ15 :: Q15 -> Q15 -> Q15
mulQ15 a b = sat16 (shiftR (prod34 a b) 15)

addQ15 :: Q15 -> Q15 -> Q15
addQ15 = satAdd SatSymmetric

subQ15 :: Q15 -> Q15 -> Q15
subQ15 = satSub SatSymmetric

-- =================================
-- QUATERNION ALGEBRA
-- =================================

qAdd :: Quaternion -> Quaternion -> Quaternion
qAdd (Quaternion w1 x1 y1 z1) (Quaternion w2 x2 y2 z2) =
    Quaternion (addQ15 w1 w2) (addQ15 x1 x2) (addQ15 y1 y2) (addQ15 z1 z2)

-- | Hamilton product.
qMul :: Quaternion -> Quaternion -> Quaternion
qMul (Quaternion w1 x1 y1 z1) (Quaternion w2 x2 y2 z2) =
    Quaternion w x y z
  where
    m = mulQ15
    w = subQ15 (subQ15 (subQ15 (m w1 w2) (m x1 x2)) (m y1 y2)) (m z1 z2)
    x = subQ15 (addQ15 (addQ15 (m w1 x2) (m x1 w2)) (m y1 z2)) (m z1 y2)
    y = addQ15 (addQ15 (subQ15 (m w1 y2) (m x1 z2)) (m y1 w2)) (m z1 x2)
    z = addQ15 (subQ15 (addQ15 (m w1 z2) (m x1 y2)) (m y1 x2)) (m z1 w2)

-- | Dot product: 4 multipliers, sum, shift by 15, saturate.
dotQ15 :: Quaternion -> Quaternion -> Q15
dotQ15 (Quaternion w1 x1 y1 z1) (Quaternion w2 x2 y2 z2) =
    sat16 (shiftR summed 15)
  where
    summed = prod34 w1 w2 + prod34 x1 x2 + prod34 y1 y2 + prod34 z1 z2

-- | One Newton step toward |r| = 1:  r' = r + r * (1 - |r|^2) / 2
-- A single step only converges for small twists. Four steps keep |r| within
-- 0.02% of 1 even for full-range (+-32767) twists.
normalizeR :: Quaternion -> Quaternion
normalizeR (Quaternion w x y z) =
    Quaternion (fixup w) (fixup x) (fixup y) (fixup z)
  where
    n        = prod34 w w + prod34 x x + prod34 y y + prod34 z z   -- Q2.30
    c        = sat16 (shiftR (1073741824 - n) 16)                  -- (1-n)/2 in Q1.15
    fixup a  = addQ15 a (mulQ15 a c)

-- | Four Newton steps in a row.
normalizeR4 :: Quaternion -> Quaternion
normalizeR4 = normalizeR . normalizeR . normalizeR . normalizeR

-- | The Stability Gate (same maths as Consensus.projectSE3).
projectSE3 :: DualQuaternion -> DualQuaternion
projectSE3 (DualQuaternion r d) =
    DualQuaternion r (Quaternion dW' dX' dY' dZ')
  where
    dt  = dotQ15 r d
    dW' = subQ15 (qW d) (mulQ15 dt (qW r))
    dX' = subQ15 (qX d) (mulQ15 dt (qX r))
    dY' = subQ15 (qY d) (mulQ15 dt (qY r))
    dZ' = subQ15 (qZ d) (mulQ15 dt (qZ r))

-- =================================
-- POSE UPDATE
-- =================================

-- | First-order exponential map: small rotation and translation step.
expMap :: Twist -> DualQuaternion
expMap t =
    DualQuaternion
      (Quaternion 32767 (shiftR (wx t) 1) (shiftR (wy t) 1) (shiftR (wz t) 1))
      (Quaternion 0     (shiftR (vx t) 1) (shiftR (vy t) 1) (shiftR (vz t) 1))

-- | One full update: multiply, renormalize r (4 steps), project d.
stepPose :: DualQuaternion -> Twist -> DualQuaternion
stepPose pose t =
    projectSE3 (DualQuaternion (normalizeR4 newReal) newDual)
  where
    delta   = expMap t
    r       = realPart pose
    d       = dualPart pose
    rd      = realPart delta
    dd      = dualPart delta
    newReal = qMul r rd
    newDual = qAdd (qMul r dd) (qMul d rd)

-- =================================
-- THE CHIP (TOP ENTITY)
-- =================================

{-# ANN topEntity
  (Synthesize
    { t_name   = "chip"
    , t_inputs = [ PortName "clk", PortName "rst", PortName "en"
                 , PortName "twist_in" ]
    , t_output = PortName "pose_out"
    }) #-}
{-# OPAQUE topEntity #-}
topEntity
    :: Clock System
    -> Reset System
    -> Enable System
    -> Signal System Twist
    -> Signal System DualQuaternion
topEntity = exposeClockResetEnable (moore stepPose id initialPose)
  where
    initialPose = DualQuaternion (Quaternion 32767 0 0 0) (Quaternion 0 0 0 0)
