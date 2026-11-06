# Double-Check Wrist Compensation Logic

## Current Values Breakdown

```
Pickup location: 150mm RIGHT, TOP ROW

BOARD GRID:
Square 1  2  3          Compensation: +15  +8   0
Square 4  5  6          Compensation: +22   0  -12
Square 7  8  9          Compensation: +30 +15 -25

KEY OBSERVATIONS:
```

## Potential Issues Found

1. **Square 8 (bottom-center) = +15°**
   - Located directly below pickup
   - Should have minimal drift (like square 2, 5)?
   - But has +15° (same as square 1, top-left)
   - ⚠️ INCONSISTENT - why does vertical descent need same correction as horizontal left?

2. **Square 5 (center) = 0°**
   - Center of board
   - Balanced position from pickup
   - ✓ Makes sense: no compensation needed

3. **Left side progression: 15 → 22 → 30**
   - Increases as we go down and left ✓
   - Magnitude is reasonable ✓

4. **Right side progression: 0 → -12 → -25**
   - Increases as we go down and right ✓
   - Magnitude is reasonable ✓

## Hypothesis

**Square 8 might be wrong.** 

- Square 8 is directly below pickup (vertical descent)
- Should be similar to square 5 (0°) with maybe slight leftward correction
- Current value (+15°) seems too high

**Suggestion:** Change square 8 from +15° to something closer to square 5:
- Option A: 0° (no compensation, like center)
- Option B: +5° to +8° (slight correction if there's minor drift)

## Test this theory:

1. Place piece on square 8 (bottom-center)
2. Observe if it rotates more than expected
3. If it does, reduce the +15° value
4. Try +5° or 0° and test again

## Direction Check (Still Needed)

To verify if signs are correct, place on:
- **Square 1 (top-left):** Should rotate the opposite direction of square 9
- **Square 9 (bottom-right):** Should rotate the opposite direction of square 1

If they rotate the same direction, signs are backwards.
