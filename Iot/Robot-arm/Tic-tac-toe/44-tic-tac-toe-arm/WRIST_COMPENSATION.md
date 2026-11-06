# Wrist Rotation Compensation System

## Overview

The tic-tac-toe arm experiences piece rotation during placement due to kinematic changes as the arm moves from the pickup position to different squares on the board. This system provides per-square wrist compensation to keep pieces aligned as they are placed.

## Problem Description

When the arm picks up a piece from the pickup position (150mm to the right of board center, aligned with top row):
- The piece starts in a specific wrist orientation
- As the arm moves to a destination square and rises to clearance height, the kinematic configuration changes
- The wrist motor angle required to keep the piece straight changes based on the destination square
- Without compensation, pieces arrive rotated relative to their intended orientation

## Solution: Per-Square Wrist Compensation

The compensation system applies different wrist angles at clearance height for each destination square (0-8):

1. **Lift Phase**: Arm rises to `clearanceAngle` (90°) with wrist at home position
2. **Travel Phase**: Arm moves horizontally above the board
3. **Compensation Phase**: At clearance height, wrist rotates to compensation angle for the destination square
4. **Descent Phase**: Arm lowers while wrist adjusts from compensation angle to final board square angle
5. **Drop Phase**: Piece is released with proper alignment

## Calibration

### Initial Setup

The compensation table in `src/main.cpp` (lines 70-81) contains per-square offsets:

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

### Calibration Process

1. **Pick up an X piece** and place it at the pickup position (150mm right of center, top row)

2. **For each square (1-9)**:
   - Send robot move command to that square
   - Observe the piece orientation when placed
   - Note the rotation angle (positive = clockwise, negative = counter-clockwise)
   - Calculate compensation as: `-rotation_angle`
   
   Example: If piece rotates 10° clockwise when placing on square 1, set `wristCompensation[0] = -10`

3. **Update the table** in `src/main.cpp` with measured compensation values

4. **Recompile and test** by placing pieces on each square

### Fine-Tuning

- **Positive compensation**: Rotates wrist clockwise at clearance to compensate for counter-clockwise drift
- **Negative compensation**: Rotates wrist counter-clockwise at clearance to compensate for clockwise drift
- Start with ±5° adjustments and refine based on results
- Each square typically needs ±10-20° compensation

## Motion Sequence for ToCell (Placement)

The complete motion sequence when placing a piece on a destination square:

```
Time 0 → liftMs:               Lift to clearance (M3: position → clearanceAngle)
         travelStart → +travelMs:  Travel horizontally to square above board
         compensationStart → +compensationMs: Apply wrist compensation at clearance
         wristStart → +wristMs:    Rotate wrist from compensation to final board angle
         lowerStart → +lowerMs:    Lower arm to board level (M3: clearance → target)
```

Where:
- `liftMs`: Duration to reach clearance height
- `travelMs`: Duration to move above destination square
- `compensationMs`: Duration to rotate to compensation angle
- `wristMs`: Duration to rotate from compensation to final angle
- `lowerMs`: Duration to lower piece to board

## Technical Details

### Code Structure

- **Compensation table**: `wristCompensation[cellCount]` (line 70)
- **Clearance height**: `clearanceAngle = 90` degrees (lift motor, M3)
- **Wrist motor**: Motor 1 (M1)
- **Motion trigger**: When `phase == Motion::ToCell` and `pendingSquare > 0`

### Compensation Calculation

```cpp
int wristAtClearance = constrain(poses[homeTarget][wristMotor], ...);
if (orientAboveBoard && pendingSquare > 0 && pendingSquare <= cellCount) {
  int cellIndex = pendingSquare - 1;
  wristAtClearance = constrain(
    wristAtClearance + wristCompensation[cellIndex],
    minAngle(wristMotor),
    maxAngle(wristMotor)
  );
}
```

The compensation is applied as an offset to the home wrist angle, constrained to valid servo range (0-180°).

### Compensation Application

Two wrist segments are created during placement:
1. **Clearance segment**: Move from current wrist position to `wristAtClearance`
2. **Final segment**: Move from `wristAtClearance` to board square target angle

This two-phase approach ensures:
- Compensation is applied while high above board (minimizes board interference)
- Wrist can adjust to final angle during descent without obstruction
- Smooth piece arrival with proper alignment

## Testing

### Manual Verification

1. Place piece at pickup position
2. Command robot to place on each square (1-9)
3. Observe piece orientation visually
4. Check camera readback (if available) for actual board state
5. Adjust compensation values and repeat

### Automated Testing

The calibration web interface allows:
- Jogging wrist motor (M1) to test angles
- Preview movement to any square
- Test grip and drop sequences
- Adjust poses while monitoring

## Troubleshooting

### Piece still rotates after compensation

- **Measurement error**: Re-measure rotation angle more carefully
- **Compensation range exceeded**: Check if compensation + home_wrist_angle stays in 0-180° range
- **Kinematic drift varies by square**: Some drift may be position-dependent; adjust per-square compensation
- **Servo resolution**: If adjustments < 2°, servo resolution may limit precision

### Pieces rotate inconsistently

- **Loose servo**: Check servo mechanical play and tightness
- **Inconsistent pickup**: Ensure piece always gripped in same position relative to gripper
- **Board surface variation**: Uneven board can cause pieces to rotate during placement
- **Timing issues**: Verify motion phases complete without interruption

### Compensation causes collisions

- **Excessive angle**: Reduce compensation magnitude
- **Wrist limited**: Ensure compensation doesn't exceed servo range 0-180°
- **Adjust clearance**: If needed, increase `clearanceAngle` for more wrist freedom

## Advanced Configuration

### Modifying Motion Timing

If pieces need more time for wrist adjustment, increase `stepMs` or adjust overlap factors:
- `liftOverlap = 0.6f`: When travel starts relative to lift completion
- `lowerOverlap = 0.7f`: When lowering starts relative to travel completion

### Per-Pickup Compensation

If pickup position varies, add position-dependent compensation:
```cpp
// Could extend to: wristCompensation[pickupPosition][destinationSquare]
```

### Pose-Dependent Compensation

Currently compensation is fixed per destination square. Could be extended to account for:
- Different pickup positions
- Gripper rotation angles
- Piece center-of-mass variations

## References

- **Main source file**: `src/main.cpp` lines 70-81, 465-520
- **Motion timing**: `src/main.cpp` lines 32-40
- **Pose definitions**: `src/main.cpp` lines 48-67
- **Motion states**: `src/main.cpp` lines 101-103
