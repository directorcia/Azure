# Bug Fix Report - Wrist Compensation Not Working

## Problem Found

The wrist compensation system was implemented in the code but **not actually moving the wrist** when placing pieces.

## Root Cause

**Line that was wrong:**
```cpp
if (orientAboveBoard) {
  target[wristMotor] = constrain(poses[homeTarget][wristMotor], ...);
}
```

This line was **overriding the target wrist angle** to the home position (70°) instead of keeping the board square angle (73°).

**What this caused:**
- Wrist was supposed to go: 73° → compensated angle → back to 73°
- But actually went: 73° → compensated angle → back to 70° (HOME angle, wrong!)
- The compensation happened but then wrist moved back to wrong angle

## The Fix

**Removed lines 489-491:**
```cpp
if (orientAboveBoard) {
  target[wristMotor] = constrain(poses[homeTarget][wristMotor], minAngle(wristMotor), maxAngle(wristMotor));
}
```

**Replaced with comment:**
```cpp
// For ToCell, DON'T override the target wrist angle - use the board square angle
// target[wristMotor] is already set correctly from the pose parameter
```

## Why This Works

1. **Line 476** already sets target angles from the pose parameter:
   ```cpp
   for (uint8_t m = 0; m < motorCount; m++) 
     target[m] = constrain(pose[m], minAngle(m), maxAngle(m));
   ```
   This sets `target[wristMotor]` = 73° (board square angle)

2. **We should NOT override it** with home angle (70°)

3. **Now the wrist motion is:**
   - Current position: ~73°
   - Moves to: 73° + compensation[square]
   - Final position: 73° (stays at board angle, not home!)

## Expected Result After Fix

### Before Fix
- Wrist doesn't move (or moves but ends at wrong angle)
- Pieces still rotated

### After Fix  
- Wrist applies compensation at clearance height ✓
- Wrist returns to board square angle ✓
- Pieces arrive straighter ✓

## Debug Output Added

Also added debug messages so you can see what's happening:

```cpp
say(String("[Wrist] ToCell square ") + String(pendingSquare) + 
    " compensation=" + String(wristCompensation[cellIndex]));

say(String("[Wrist] compMs=") + String(compensationMs) + 
    " wristMs=" + String(wristMs));
```

Check the Serial Monitor output when placing pieces - you should now see:
```
[Wrist] ToCell square 1 compensation=-8 current=73 clearance=65
[Wrist] compMs=320 wristMs=320
```

## How to Test

1. **Recompile the code** (this fix is now in src/main.cpp)
2. **Upload to device**
3. **Open Serial Monitor**
4. **Place an X piece** - you should see debug output
5. **Watch the wrist** - it should now rotate at clearance height!
6. **Check the piece** - should be much straighter

## Files Modified

- `src/main.cpp` (lines 481-491)
  - Removed home angle override
  - Added debug output
  - Added explanatory comments

## Status

✅ **Bug identified and fixed**  
✅ **Ready to recompile and upload**  
✅ **Debug output added for verification**  

**Next Step:** Compile and upload the fixed code, then test!

---

## Technical Details

### The Board Poses (M1 = Wrist Motor, Column 1)

```
Board squares 1-9: M1 = 73°
Pickup position:   M1 = 73°
Home position:     M1 = 70°  ← We were incorrectly using this as target
```

The bug was using the home position wrist angle (70°) as the target for board placements, when we should be using the board square angle (73°).

### Motion Sequence Now

**Before fix (broken):**
```
Start at: 73° (from previous position or pickup)
Lift to clearance, apply compensation
Wrist: 73° → 65° (compensation) → 70° (HOME - WRONG!)
Result: Weird wrist movement, compensation doesn't help
```

**After fix (correct):**
```
Start at: 73° (from previous position or pickup)
Lift to clearance, apply compensation
Wrist: 73° → 65° (compensation) → 73° (BOARD - CORRECT!)
Result: Clean compensation motion, piece placed straight
```

## Why This Wasn't Caught Earlier

The compensation was implemented (segments being created), but since the target was wrong, the effect was canceled out or hidden. The wrist would move to compensation angle, but then move back to a different angle (home instead of board), making the compensation ineffective.
