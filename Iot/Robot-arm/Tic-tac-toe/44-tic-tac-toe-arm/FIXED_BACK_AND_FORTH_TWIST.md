# ✅ FIXED: Back-and-Forth Wrist Twist

## The Problem

The wrist was twisting **back and forth** instead of smoothly compensating once:
- Move 1: 73° → 65° (compensation)
- Move 2: 65° → 73° (back!)

This happened because we had **two separate wrist segments**.

## Root Cause

The code was:
1. Calculate `wristAtClearance` (compensation angle, e.g., 65°)
2. Move from current → `wristAtClearance` 
3. Then move from `wristAtClearance` → `target[wristMotor]` (which was 73°, the original board angle)

So it always moved back to where it started!

## The Solution

**Make the compensated angle the TARGET angle, not an intermediate step:**

### Change 1: Set compensation as the target (line 484)
```cpp
if (orientAboveBoard && destSquare > 0 && destSquare <= cellCount) {
    int cellIndex = destSquare - 1;
    wristAtClearance = constrain(wristAtClearance + wristCompensation[cellIndex], ...);
    // NEW: Set this as the final target, not a waypoint
    target[wristMotor] = wristAtClearance;  // ← KEY FIX
}
```

### Change 2: Remove the second wrist movement (line 520)
```cpp
// OLD: Two movements
uint32_t compensationMs = ...;
uint32_t wristMs = moveDuration(fabsf(target[wristMotor] - wristAtClearance), ...);

// NEW: One movement only
uint32_t compensationMs = ...;
uint32_t wristMs = 0;  // No second movement
```

### Change 3: Single wrist segment (line 527)
```cpp
// OLD: Two segments
addSegment(next, wristMotor, compensationStart, ..., positions[wristMotor], wristAtClearance);
addSegment(next, wristMotor, wristStart, ..., wristAtClearance, target[wristMotor]);

// NEW: One smooth movement
addSegment(next, wristMotor, compensationStart, ..., positions[wristMotor], target[wristMotor]);
```

## Motion Sequence Now

✅ Lift to clearance (90°)
✅ Travel to square above board
✅ **Single wrist movement:** current angle → compensated angle (e.g., 73° → 65°)
✅ **Hold compensated angle while lowering**
✅ Lower to board with piece straight

No more back-and-forth! Just one smooth compensation movement at clearance. 🎯

## Deploy & Test

```bash
platformio run -e uno_r4_wifi --target upload
```

You should see:
```
[Wrist] ToCell square 1 compensation=-8 current=73 clearance=65
[Wrist] compMs=320 (single movement to compensated angle)
```

Watch for:
- Wrist rotates ONCE at clearance
- No back-and-forth twist
- Piece placed straight on board ✓
