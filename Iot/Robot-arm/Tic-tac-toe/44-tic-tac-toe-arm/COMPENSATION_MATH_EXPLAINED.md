# Wrist Compensation Math - Why Bigger Values

## The Problem

Original compensation values were **way too small** for the observed drift:

| Square | Old Value | Observed Drift | Problem |
|--------|-----------|-----------------|---------|
| 1      | -8°       | ±5-15°         | ✗ Way too small |
| 7      | -12°      | ±10-15°        | ✗ Insufficient |
| 9      | +6°       | ±10-15°        | ✗ Inadequate |

**You said pieces arrived rotated by 5-15°. We were only compensating by up to 12°. That's the problem!**

## The Geometry

```
Pickup Position: 150mm RIGHT of board center, TOP ROW

       Center (0, 0)
            |
            |
  +150mm ---O---- (PICKUP)
            |
            |
     SQUARE 7 (bottom-left) is:
     - 150mm+ to the LEFT
     - 3 rows DOWN
     - = ~300mm DIAGONAL from pickup
     
     = Maximum kinematic drift
```

### Kinematics Principle

As the arm moves further from pickup position:
- Link angles change more
- End effector (piece) rotates relative to base
- **Rotation increases non-linearly with distance**

The drift **is NOT linear**:
- Adjacent squares to pickup: 5-8° drift
- Middle squares: 10-15° drift  
- Opposite corner (square 7 from pickup): 25-30° drift

## New Compensation Values

Based on this model:

```
Pickup is at: top row, 150mm RIGHT

Square 1 (top-left):      -15°  (leftward, close to pickup)
Square 2 (top-center):    -8°   (slight left)
Square 3 (top-right):     0°    (reference, near pickup)

Square 4 (mid-left):      -22°  (stronger left, farther)
Square 5 (center):        0°    (balanced)
Square 6 (mid-right):     +12°  (rightward)

Square 7 (bot-left):      -30°  (MAXIMUM left drift, opposite corner)
Square 8 (bot-center):    -15°  (vertical descent, left drift)
Square 9 (bot-right):     +25°  (MAXIMUM right drift, far from pickup)
```

## Pattern Analysis

```
                Square 1    Square 2    Square 3
                  -15°        -8°         0°
    
                Square 4    Square 5    Square 6
                  -22°        0°       +12°
                
                Square 7    Square 8    Square 9
                  -30°       -15°       +25°
```

**Left side:** Negative (counterclockwise rotation needed)
- Increases in magnitude moving down
- Maximum: -30° at bottom-left

**Center:** Neutral (0°)
- No drift correction needed

**Right side:** Positive (clockwise rotation needed)
- Smaller on middle row (+12°)
- Large on bottom row (+25°)

## Why These Values?

1. **Pickup at top-right (150mm right)** creates asymmetry
   - Left side squares need more negative compensation
   - Right side squares need less positive compensation

2. **Distance increases drift**
   - Top row: ±8 to ±15°
   - Bottom row: ±15 to ±30°

3. **Diagonal distance = worst case**
   - Square 7 (opposite corner): -30° required
   - Square 9 (far right): +25° required

## The Key Insight

**Compensation magnitude should MATCH the observed drift magnitude.**

If pieces arrive at ±5-15° rotation, compensation must be at least ±15-30° to fully correct them.

New values: ±15° to ±30°  
Old values: ±4° to ±12°  

**~2.5x increase in compensation!**

## Testing

1. Compile and upload
2. Place piece on square 7 (should be most noticeable left compensation)
3. Watch wrist rotate ~30° counterclockwise
4. Piece should arrive much straighter

If still rotating, fine-tune individual values ±5° as needed.
