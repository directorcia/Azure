# Example Wrist Compensation Calibration

## Scenario

You have a tic-tac-toe arm that places X pieces from a fixed pickup position onto a 3x3 board. The pieces consistently arrive rotated by different amounts depending on which square they're placed on.

## Calibration Measurements

You perform the calibration procedure and measure the following rotation angles when placing pieces on each square:

### Raw Measurements

```
Square 1 (top-left):    piece rotates +12° clockwise
Square 2 (top-center):  piece rotates +8° clockwise
Square 3 (top-right):   piece rotates +15° clockwise

Square 4 (mid-left):    piece rotates -3° (3° counter-clockwise)
Square 5 (center):      piece rotates +0° (straight!)
Square 6 (mid-right):   piece rotates -8° (8° counter-clockwise)

Square 7 (bottom-left):  piece rotates +10° clockwise
Square 8 (bottom-center): piece rotates +5° clockwise
Square 9 (bottom-right):  piece rotates +13° clockwise
```

### Calculated Compensation

For each rotation angle, the compensation is the negative of that angle:

```
Compensation = -Rotation
```

| Square | Rotation | Compensation |
|--------|----------|--------------|
| 1      | +12°     | -12          |
| 2      | +8°      | -8           |
| 3      | +15°     | -15          |
| 4      | -3°      | +3           |
| 5      | +0°      | 0            |
| 6      | -8°      | +8           |
| 7      | +10°     | -10          |
| 8      | +5°      | -5           |
| 9      | +13°     | -13          |

## Code Update

### Find the Compensation Table

**File**: `src/main.cpp`
**Location**: Lines 70-81

### Before (Default/Uncompensated)

```cpp
const int wristCompensation[cellCount] = {
  0,    // Cell 0 (top-left):      square 1
  0,    // Cell 1 (top-center):    square 2
  0,    // Cell 2 (top-right):     square 3
  0,    // Cell 3 (middle-left):   square 4
  0,    // Cell 4 (center):        square 5
  0,    // Cell 5 (middle-right):  square 6
  0,    // Cell 6 (bottom-left):   square 7
  0,    // Cell 7 (bottom-center): square 8
  0     // Cell 8 (bottom-right):  square 9
};
```

### After (Calibrated)

```cpp
const int wristCompensation[cellCount] = {
  -12,  // Cell 0 (top-left):      square 1
  -8,   // Cell 1 (top-center):    square 2
  -15,  // Cell 2 (top-right):     square 3
  3,    // Cell 3 (middle-left):   square 4
  0,    // Cell 4 (center):        square 5
  8,    // Cell 5 (middle-right):  square 6
  -10,  // Cell 6 (bottom-left):   square 7
  -5,   // Cell 7 (bottom-center): square 8
  -13   // Cell 8 (bottom-right):  square 9
};
```

## Compilation

```bash
# Run these commands from the 44-tic-tac-toe-arm directory
# (the folder that contains platformio.ini).

# Clean previous build
platformio run --target clean

# Build for the target board
platformio run -e uno_r4_wifi

# Upload to device
platformio run -e uno_r4_wifi --target upload
```

## Verification

After uploading the new firmware:

1. **Place test X at pickup position** with reference mark

2. **Test each square** (1-9):
   ```
   Command: 1  (place on square 1)
   Result: Piece should arrive straight (±2-3° acceptable)
   
   Command: 2  (place on square 2)
   Result: Piece should arrive straight
   
   ... continue for all 9 squares
   ```

3. **Expected outcome**:
   - Most pieces arrive within ±2-3° of target
   - Significant improvement from ±12° rotations
   - Game play not affected by remaining small errors

## Fine-Tuning Example

If after calibration, you observe:

```
Residual rotations:
Square 1: -2° (still rotates slightly counter-clockwise)
Square 5: +0.5° (almost straight)
Square 9: +3° (rotates slightly clockwise)
```

This is typically acceptable for game play. However, if you want to fine-tune:

### Option A: Uniform Adjustment
If all pieces rotate in the same direction by ~2°, adjust all values equally:

```cpp
// Add +2 to all (shift entire array by 2°)
const int wristCompensation[cellCount] = {
  -10,  // was -12, now -12 + 2 = -10
  -6,   // was -8,  now -8 + 2 = -6
  -13,  // was -15, now -15 + 2 = -13
  5,    // was 3,   now 3 + 2 = 5
  2,    // was 0,   now 0 + 2 = 2
  10,   // was 8,   now 8 + 2 = 10
  -8,   // was -10, now -10 + 2 = -8
  -3,   // was -5,  now -5 + 2 = -3
  -11   // was -13, now -13 + 2 = -11
};
```

### Option B: Per-Square Adjustment
If specific squares still have residual rotation:

```cpp
// Adjust only problem squares
const int wristCompensation[cellCount] = {
  -14,  // was -12, Square 1 still -2°, so -12 - 2 = -14
  -8,   // Square 2 is good
  -15,  // Square 3 is good
  3,    // Square 4 is good
  0,    // Square 5 is good (was +0.5°, acceptable)
  8,    // Square 6 is good
  -10,  // Square 7 is good
  -5,   // Square 8 is good
  -10   // was -13, Square 9 still +3°, so -13 + 3 = -10
};
```

## Pattern Recognition

If all pieces rotate in the same direction, it suggests:
- **All clockwise**: Pickup position needs adjustment, or gripper rotation offset
- **All counter-clockwise**: Same potential issue
- **Uniform pattern**: May indicate systematic error in arm calibration

Consider rechecking:
1. Pickup position is exactly 150mm right of board center
2. Piece is grasped at consistent center-of-mass
3. Board is level
4. No mechanical slack in arm

## When to Stop Calibrating

✓ Stop when:
- Pieces arrive within ±2-3° of straight
- Acceptable for game play and user satisfaction
- Further tweaking produces minimal improvement
- Compensation values are stable (don't keep changing)

Examples of acceptable end states:
- All 9 squares within ±2°
- 8 squares straight, 1 square ±3°
- Systematic ±1° error across all squares

## Troubleshooting During Calibration

### Issue: Compensation values exceed servo range

**Problem**:
```
Home wrist angle = 73°
Compensation = -30°
Result = 73 - 30 = 43° ✓ (OK, in 0-180° range)

Home wrist angle = 73°
Compensation = -80°
Result = 73 - 80 = -7° ✗ (Invalid, negative)
```

**Solution**:
```cpp
// The code constrains automatically:
wristAtClearance = constrain(
  wristAtClearance + wristCompensation[cellIndex],
  minAngle(wristMotor),      // 0°
  maxAngle(wristMotor)       // 180°
);

// So -7° becomes 0° (clamped to minimum)
// This may not give desired compensation

// Instead, adjust home wrist angle in board poses:
// Edit poses[0..8][wristMotor] to start from different angle
// Then recalibrate compensation
```

### Issue: Measurements inconsistent

**Problem**: Placing on same square gives different rotations each time

**Causes**:
- Gripper not grasping piece consistently
- Pickup position varies
- Board surface not level or piece slides
- Servo mechanical play

**Solution**:
- Use reference mark on piece edge
- Mark pickup position with tape
- Verify board is level with bubble level
- Check for servo play by jiggling arm
- Take multiple measurements and average

### Issue: Compensation makes it worse

**Problem**: After applying compensation, pieces rotate more

**Likely cause**: Compensation values have wrong sign

**Solution**:
```
If pieces were rotating +10° clockwise,
and compensation is +10 (not -10),
then: +10° + 10° = +20° (worse!)

Fix: Change sign of all values
Old: {-12, -8, -15, ...}
New: {12, 8, 15, ...}  (inverted)
```

## Summary: Before vs. After

### Before Calibration
```
Placing X on different squares results in:
└─ Pieces arrive rotated ±5-15°
└─ Rotation varies significantly per square
└─ Game still playable but pieces visually misaligned
└─ Difficult to read board state
```

### After Calibration
```
Placing X on all squares results in:
└─ Pieces arrive straight ±2-3°
└─ Consistent alignment across board
└─ Game fully playable
└─ Clear board state for camera readback
└─ Professional appearance
```

## Example: Full Board After Calibration

With the calibrated compensation values:

```
    1 | 2 | 3
   ---+---+---
    4 | X | 6      ← All X pieces perfectly straight
   ---+---+---
    7 | 8 | 9

    All pieces aligned, ready for game play! ✓
```
