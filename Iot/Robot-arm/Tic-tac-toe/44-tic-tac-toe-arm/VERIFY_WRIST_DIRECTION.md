# Verify Wrist Compensation Direction

## Current Values

```
Square 1  2  3
  15  8  0     (top row - minimal to moderate compensation)

Square 4  5  6
  22  0 -12    (middle row)

Square 7  8  9
  30 15 -25    (bottom row - strongest compensation)
```

## How to Test

Place pieces on each square and observe:

1. **Place on Square 1 (top-left):**
   - Current value: +15° (clockwise)
   - Does piece rotate CW or CCW when placed?
   - Is +15° too much, too little, or correct?

2. **Place on Square 3 (top-right, near pickup):**
   - Current value: 0° (no rotation)
   - Does piece stay straight or rotate?

3. **Place on Square 7 (bottom-left, opposite corner):**
   - Current value: +30° (maximum clockwise)
   - Does piece rotate CW or CCW?
   - Is +30° correct magnitude?

4. **Place on Square 9 (bottom-right):**
   - Current value: -25° (maximum counterclockwise)
   - Does piece rotate CW or CCW?
   - Is -25° correct magnitude?

## Pattern Analysis

Current pattern:
- **Left side:** Positive (CW): 15, 22, 30
- **Right side:** Negative (CCW): 0, -12, -25
- **Center:** Mixed: 8, 0, 15

## Questions to Answer

1. When arm picks up at top-right and places at top-left (square 1):
   - Does piece naturally rotate CLOCKWISE or COUNTERCLOCKWISE?
   - Our compensation: +15° clockwise
   - Is this correct direction?

2. When arm picks up at top-right and places at bottom-right (square 9):
   - Does piece naturally rotate CLOCKWISE or COUNTERCLOCKWISE?
   - Our compensation: -25° counterclockwise
   - Is this correct direction?

3. When arm places at center (square 5):
   - Does piece rotate at all?
   - Our compensation: 0° (correct)

## If Pattern is Wrong

Signs might be reversed. If pieces still rotate wrong way after compensation, try:
- Multiply all values by -1 to reverse direction
- Then fine-tune individual values ±5°
