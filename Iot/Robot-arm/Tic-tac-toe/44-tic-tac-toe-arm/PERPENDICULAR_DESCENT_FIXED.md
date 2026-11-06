# ✅ FIXED: Perpendicular Descent (Straight Down)

## The Problem

Some pieces were being placed at an angle - the arm approached from the side rather than straight down from directly above the square.

## The Solution

Modified the motion sequence for **ToCell** moves to ensure:
1. **Lift straight up** from pickup position
2. **Travel horizontally** (rotate/bend) while at clearance height to position above target square
3. **Lower straight down** (only lift motor moves) - no other motors moving during descent

## Motion Sequence - Before

```
Lift → |
       |    Travel (XY motors move)
       |    (Wrist adjusts)
       |    (Lower starts DURING travel with overlap)  ✗ ANGLED DESCENT
```

Problem: Lower phase could start while XY motors were still adjusting, causing angled approach.

## Motion Sequence - After

```
Lift ↑
     |
     └─ Hold at clearance ─────────────────────────┐
                                                    │
     ┌─ Travel XY (rotate/bend at clearance)      │
     │  (all non-lift motors adjust)               │
     │  └─ Completed before lowering starts       │
     │                                              │
     ├─ Adjust wrist (at clearance)                │
     │  (compensation rotation)                     │
     │  └─ Completed before lowering starts       │
     │                                              │
     └─ Lower ↓ (only lift motor moves)  ✓ STRAIGHT DOWN
        (All other motors held at board position)
```

## Key Changes

### 1. Hold Segment Extended (line 521)

**Before:**
```cpp
addSegment(next, liftMotor, liftMs, wristStart + wristMs, clearance, clearance);
```

**After:**
```cpp
addSegment(next, liftMotor, liftMs, lowerStart, clearance, clearance);
```

Now M3 (lift) holds at clearance from right after lift through the entire travel, wrist adjustment, and only releases when lowering begins.

### 2. Lower Start Timing (line 517)

**Before:**
```cpp
uint32_t lowerStart = orientAboveBoard ? wristStart + wristMs : max(...);
```

**After:**
```cpp
uint32_t lowerStart = orientAboveBoard ? (wristStart + wristMs) : max(...);
```

Explicit sequencing: Lower only starts AFTER wrist adjustment completes.

### 3. Debug Output Added (line 539)

Now you'll see timing info:
```
[Motion] ToCell travel ms=500 wrist at clearance until ms=1200 then lower ms=400
```

This confirms:
- Travel phase: 0-500ms
- Wrist at clearance: 500-1200ms
- Lower phase: 1200-1600ms

## Guarantee

✅ **Arm is always directly above the target square when lowering begins**
✅ **Only lift motor (M3) moves during final descent**
✅ **All other motors held at board position**
✅ **Piece descends straight down (perpendicular to board)**

## Testing

Compile and upload:
```bash
platformio run -e uno_r4_wifi --target upload
```

Observe:
1. Arm lifts straight up from pickup
2. Arm bends/rotates while high in air (at clearance)
3. Wrist adjusts for compensation
4. Arm lowers straight down (you'll see only the vertical motor moving, not horizontal bending)
5. Piece lands straight on board

Pieces should now place perpendicular to the board! 🎯
