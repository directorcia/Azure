# ✅ FIXED: Return Over Center Between Moves

## The Problem

After placing a piece, the arm was going directly from the target square to the home/rest position without passing over the center of the board.

## The Solution

Added a new intermediate motion state `ToCenter` that moves the arm over the center of the board (square 5) as a transit point before returning to home.

## Motion Flow - After

```
Robot Move Sequence:
├─ Pickup at outside location
├─ Move to target square (1-9)
├─ Drop piece
├─ Move to CENTER OF BOARD (square 5) ✓ NEW
├─ Move to home/rest position
├─ Camera reads board
└─ Human's turn
```

This provides:
- ✅ Smooth motion path through board center
- ✅ Camera can see arm in neutral/safe position
- ✅ Easier visual inspection of board layout
- ✅ Consistent post-move position

## Technical Changes

### 1. Added ToCenter Motion State (line 97)

```cpp
enum class Motion { 
  Idle, PrepHome, OpenGrip, ToPickup, Grab, GrabSettle, 
  ToCell, Arrive, Release, Drop, 
  ToCenter,  // ← NEW
  ToHome, Jig, Preview, GripTest, Park 
};
```

### 2. Drop Phase Now Targets Center (line 684)

**Before:**
```cpp
if (motion == Motion::Drop) {
  if (millis() - pauseAt >= dropPauseMs) startTransfer(poses[homeTarget], Motion::ToHome, true, true);
  return;
}
```

**After:**
```cpp
if (motion == Motion::Drop) {
  if (millis() - pauseAt >= dropPauseMs) startTransfer(poses[4], Motion::ToCenter, true, true);  // Square 5 (center)
  return;
}
```

### 3. ToCenter Handler (line 707)

```cpp
case Motion::ToCenter:
  say("[Motion] Over center of board. Returning to home for camera read.");
  startTransfer(poses[homeTarget], Motion::ToHome, true, true);
  break;
```

## Board Layout Reminder

```
1 2 3
4 5 6
7 8 9

Square 5 = CENTER
```

Square 5 position in poses array: `poses[4]` (0-indexed)

## Testing

Compile and upload:
```bash
platformio run -e uno_r4_wifi --target upload
```

Watch the arm's motion after placing a piece:
1. Lifts from square
2. Travels to center (over square 5)
3. Then moves to home position
4. Camera settles and reads board

You should see the arm pass directly over the center of the board between every move! ✓
