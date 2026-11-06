# Wrist Compensation Calibration Guide

## Quick Start

1. **Identify the problem**: When placing X pieces on the board, they arrive rotated
2. **Measure the rotation**: For each square, determine rotation angle relative to pickup orientation
3. **Calculate compensation**: `compensation = -rotation_angle`
4. **Update the table**: Modify `wristCompensation[]` in `src/main.cpp` (lines 70-81)
5. **Recompile and test**: Flash new firmware and verify improvement

## Detailed Calibration Steps

### Step 1: Prepare the Board and Arm

```
1. Start the arm: Send "ARM" command
2. Move to home: Confirm gripper opens at home position
3. Place X piece at pickup position:
   - 150mm to the right of board center
   - Aligned with top row (same Y as squares 1, 2, 3)
   - Piece should be straight (mark reference edge with marker)
4. Note piece orientation: Remember which edge points where
```

### Step 2: Test Placement on Each Square

For each square 1-9:

```
1. Command robot move: Send square number
2. Wait for completion: Arm returns to home
3. Measure rotation: 
   - Compare piece edge to reference marks
   - Positive rotation: piece rotates clockwise
   - Negative rotation: piece rotates counter-clockwise
   - Record angle in degrees (±0.5° precision if possible)
4. Remove piece: Clear board for next test
```

### Step 3: Create Compensation Map

Create a table with measured rotations:

| Square | Rotation | Compensation |
|--------|----------|--------------|
| 1      | +5°      | -5           |
| 2      | +3°      | -3           |
| 3      | +8°      | -8           |
| 4      | -2°      | +2           |
| 5      | 0°       | 0            |
| 6      | -4°      | -4           |
| 7      | +6°      | -6           |
| 8      | +4°      | -4           |
| 9      | +7°      | -7           |

### Step 4: Update Source Code

Edit `src/main.cpp`, find the wrist compensation table (lines 70-81):

```cpp
const int wristCompensation[cellCount] = {
  -5,   // Cell 0 (top-left):      square 1
  -3,   // Cell 1 (top-center):    square 2
  -8,   // Cell 2 (top-right):     square 3
  2,    // Cell 3 (middle-left):   square 4
  0,    // Cell 4 (center):        square 5
  -4,   // Cell 5 (middle-right):  square 6
  -6,   // Cell 6 (bottom-left):   square 7
  -4,   // Cell 7 (bottom-center): square 8
  -7    // Cell 8 (bottom-right):  square 9
};
```

### Step 5: Compile and Flash

```bash
# Build for the arm
platformio run -e uno_r4_wifi

# Flash to device (uses USB serial)
platformio run -e uno_r4_wifi --target upload
```

### Step 6: Verify Improvement

Repeat Step 2 for all squares:

```
- Most pieces should now be straight (±2-3° acceptable)
- Note any remaining rotation
- If systematic error remains, adjust all values equally
- If per-square error varies, refine individual values
```

### Step 7: Fine-Tune (if needed)

If residual rotation remains:

```
1. Identify pattern:
   - All clockwise? Add positive compensation to all
   - All counter-clockwise? Add negative compensation to all
   - Varies by location? Adjust individual squares

2. Make small adjustments:
   - Change by ±2-5° at a time
   - Recompile and test
   - Verify improvement before next adjustment

3. Stop when:
   - Pieces arrive within ±3° rotation
   - Acceptable for game play
   - Further improvement has diminishing returns
```

## Expected Compensation Ranges

Based on typical arm kinematics:

- **Typical range**: -20 to +20 degrees
- **Small adjustments**: ±5-10 degrees most common
- **Large adjustments**: ±15-20 degrees indicates mechanical issue

If compensation needed is outside ±20°:
1. Verify pickup position is exactly 150mm right of center
2. Check if gripper rotation angle is consistent
3. Inspect for servo play or mechanical slack
4. Recalibrate board-to-camera mapping

## Compensation Application During Movement

When placing piece on a destination square, the wrist motion is:

```
Initial Position
    ↓
[Lift to clearance with wrist at home angle]
    ↓
[Travel horizontally above board]
    ↓
[Apply compensation rotation at clearance height] ← Key step
    ↓
[Adjust wrist to final board angle during descent]
    ↓
[Lower to board and drop piece] ← Piece arrives straight
```

The compensation rotation happens **high in the air** at clearance height (M3 ≈ 90°), ensuring:
- No interference with board
- Wrist has full range to adjust
- Momentum doesn't affect placement accuracy

## Troubleshooting Calibration

### Problem: Servo doesn't move to compensation angle

**Causes:**
- Compensation value pushes wrist beyond 0-180° limits
- Servo slew rate too slow

**Solution:**
- Check: `constrained_angle = home_angle + compensation`
- Must result in 0-180° range
- If needed: Adjust home wrist angle in board poses

### Problem: Pieces still rotate but inconsistently

**Causes:**
- Pickup position varies
- Gripper not releasing pieces consistently
- Board surface not level

**Solution:**
- Verify pickup position with calipers
- Test gripper grip force
- Check board with level tool
- Use mark/reference edge on pieces consistently

### Problem: Compensation worked, then stopped working

**Causes:**
- Servo drifted or aged
- Mechanical play developed
- Compiled with old code accidentally

**Solution:**
- Verify source code has latest compensation values
- Check servo for mechanical play
- Recalibrate if servo drift detected
- Review build log to confirm new code was compiled

## Alternative Approaches

If wrist compensation alone insufficient:

### 1. Gripper Angle Adjustment
Adjust gripper (M0) angle when picking up to align piece differently:
```cpp
// In pickup poses
{adjustedGripperAngle, ...}
```

### 2. Pickup Position Adjustment
Move pickup position slightly (if possible):
- Adjust X position: varies side-to-side rotation
- Adjust Y position: varies top-to-bottom rotation
- Re-measure compensation after adjustment

### 3. Extend Clearance Time
Give wrist more time to settle at compensation angle:
```cpp
const int compensationMs_extra = 200;  // milliseconds
```

### 4. Two-Point Compensation
If per-square variation is high, could implement:
```cpp
// Compensation depends on both:
// - Destination square
// - Pickup position (if variable)
int wristCompensation[pickupCount][cellCount];
```

## Validation

After calibration, verify:

- [ ] All 9 squares have compensation values
- [ ] Values constrain to -180 to +180 range
- [ ] Pieces arrive within ±3° of target orientation
- [ ] Game play is not affected by residual rotation
- [ ] Compensation values saved in source code
- [ ] Build system includes latest main.cpp
- [ ] Device firmware is up-to-date

## Notes for Future Improvements

The current implementation is **per-destination square**. Could be extended:

1. **Pickup-position dependent**: Track where X was picked up
2. **Piece-center-of-mass dependent**: Handle different piece shapes
3. **Gripper rotation dependent**: Account for gripper orientation at pickup
4. **Board surface dependent**: Adapt to different board materials
5. **Servo aging compensation**: Auto-adjust as servo stiffness changes

Current design prioritizes **simplicity and predictability** for a stationary pickup and destination squares.
