{-# LANGUAGE NoImplicitPrelude, MagicHash, TypeFamilies, DeriveGeneric, DeriveAnyClass, FlexibleContexts #-}

{-|
Module      : PoseEngine
Description : Pose update engine (pose x twist step), Q1.15 fixed point.

Fixes relative to the original:
  * products are rescaled to Q1.15 (shift right by 15, then saturate); the
    original multiplied plain integers, so almost every product saturated;
  * the z component of the quaternion product now has the correct signs:
        z = w1 z2 + x1 y2 - y1 x2 + z1 w2
  * the operation order of every component matches MasterChip.qMul, so
    PoseEngine's update is bit-identical to MasterChip's update before
    MasterChip's renormalization and projection.

This engine has no renormalization and no projection. |r| and r.d drift;
MasterChip.hs adds both.
-}
module PoseEngine where

import Clash.Prelude
import Control.DeepSeq
import GHC.Generics

data Quaternion a = Quaternion { qW :: a, qX :: a, qY :: a, qZ :: a }
    deriving (Show, Generic, NFDataX, Functor)

data DualQuaternion a = DualQuaternion { realPart :: Quaternion a, dualPart :: Quaternion a }
    deriving (Show, Generic, NFDataX)

data Twist a = Twist { angular :: (a, a, a), linear :: (a, a, a) }
    deriving (Show, Generic, NFDataX)

-- | Clamp a wide value to the symmetric Q1.15 range.
sat16 :: Signed 34 -> Signed 16
sat16 v
  | v > 32767    = 32767
  | v < (-32767) = (-32767)
  | otherwise    = resize v

-- | Exact 32-bit product, widened.
prod34 :: Signed 16 -> Signed 16 -> Signed 34
prod34 a b = resize (a `mul` b)

-- | Q1.15 multiply: full product, shift back by 15, saturate.
mulQ15 :: Signed 16 -> Signed 16 -> Signed 16
mulQ15 a b = sat16 (shiftR (prod34 a b) 15)

-- | Hamilton product in Q1.15.
mulQ :: Quaternion (Signed 16) -> Quaternion (Signed 16) -> Quaternion (Signed 16)
mulQ (Quaternion w1 x1 y1 z1) (Quaternion w2 x2 y2 z2) =
    Quaternion w x y z
  where
    m = mulQ15
    sAdd a b = satAdd SatSymmetric a b
    sSub a b = satSub SatSymmetric a b
    w = sSub (sSub (sSub (m w1 w2) (m x1 x2)) (m y1 y2)) (m z1 z2)
    x = sSub (sAdd (sAdd (m w1 x2) (m x1 w2)) (m y1 z2)) (m z1 y2)
    y = sAdd (sAdd (sSub (m w1 y2) (m x1 z2)) (m y1 w2)) (m z1 x2)
    z = sAdd (sSub (sAdd (m w1 z2) (m x1 y2)) (m y1 x2)) (m z1 w2)

addQ :: Quaternion (Signed 16) -> Quaternion (Signed 16) -> Quaternion (Signed 16)
addQ (Quaternion w1 x1 y1 z1) (Quaternion w2 x2 y2 z2) =
    Quaternion (satAdd SatSymmetric w1 w2) (satAdd SatSymmetric x1 x2) 
               (satAdd SatSymmetric y1 y2) (satAdd SatSymmetric z1 z2)

-- | First-order exponential map: small rotation and translation step.
expMap :: Twist (Signed 16) -> DualQuaternion (Signed 16)
expMap (Twist (ax, ay, az) (lx, ly, lz)) = 
    let r = Quaternion 32767 (ax `shiftR` 1) (ay `shiftR` 1) (az `shiftR` 1)
        d = Quaternion 0 (lx `shiftR` 1) (ly `shiftR` 1) (lz `shiftR` 1)
    in DualQuaternion r d

kernelTransition :: DualQuaternion (Signed 16) -> Twist (Signed 16) 
                 -> (DualQuaternion (Signed 16), DualQuaternion (Signed 16))
kernelTransition current flow = 
    let delta = expMap flow
        newReal = realPart current `mulQ` realPart delta
        newDual = addQ (realPart current `mulQ` dualPart delta) 
                       (dualPart current `mulQ` realPart delta)
        nextState = DualQuaternion newReal newDual
    in (nextState, nextState)

{-# ANN topEntity
  (Synthesize
    { t_name   = "pose_engine"
    , t_inputs = [ PortName "clk", PortName "rst", PortName "en"
                 , PortName "twist_in" ]
    , t_output = PortName "pose_out"
    }) #-}
{-# OPAQUE topEntity #-}
topEntity :: Clock System -> Reset System -> Enable System 
          -> Signal System (Twist (Signed 16)) -> Signal System (DualQuaternion (Signed 16))
topEntity = exposeClockResetEnable (mealy kernelTransition initialPose)
  where initialPose = DualQuaternion (Quaternion 32767 0 0 0) (Quaternion 0 0 0 0)
