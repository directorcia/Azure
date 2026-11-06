# 🔴 CRITICAL BUG FOUND AND FIXED

## The Problem

**No wrist adjustments were happening because the compensation check was using a variable that had already been zeroed out!**

### The Bug Flow

In `updateGameMotion()` at line 654-657:

```cpp
int square = pendingSquare;
pendingSquare = 0;  // ← ZEROED HERE
say("[Motion] Carrying piece directly to square " + String(square) + "; orienting M1 at clearance before lowering.");
startTransfer(poses[square - 1], Motion::ToCell, true, true);  // ← But compensation check uses pendingSquare
```

Then in `startTransfer()` at line 481:

```cpp
if (orientAboveBoard && pendingSquare > 0 && pendingSquare <= cellCount) {  // ← Always FALSE!
  // Apply compensation...
}
```

**Result:** The condition `pendingSquare > 0` was always FALSE because `pendingSquare` had been set to 0 before the function was called.

## The Fix

Added a `destSquare` parameter to `startTransfer()`:

### Change 1: Function Signature (line 475)

**Before:**
```cpp
void startTransfer(const int pose[motorCount], Motion phase, bool holdGrip, bool liftFirst = false)
```

**After:**
```cpp
void startTransfer(const int pose[motorCount], Motion phase, bool holdGrip, bool liftFirst = false, int destSquare = 0)
```

### Change 2: Compensation Check (line 481)

**Before:**
```cpp
if (orientAboveBoard && pendingSquare > 0 && pendingSquare <= cellCount) {
    int cellIndex = pendingSquare - 1;
```

**After:**
```cpp
if (orientAboveBoard && destSquare > 0 && destSquare <= cellCount) {
    int cellIndex = destSquare - 1;
```

### Change 3: Function Call (line 659)

**Before:**
```cpp
startTransfer(poses[square - 1], Motion::ToCell, true, true);
```

**After:**
```cpp
startTransfer(poses[square - 1], Motion::ToCell, true, true, square);  // ← Pass the square!
```

## Why This Matters

Now when placing a piece on the board:

1. ✅ `square` variable holds the correct destination (1-9)
2. ✅ `destSquare` parameter is passed to `startTransfer()`
3. ✅ Compensation check uses `destSquare` which is NEVER zeroed
4. ✅ Correct compensation angle is applied at clearance height
5. ✅ Wrist motor actually rotates at clearance height
6. ✅ Pieces arrive straight on the board

## Deploy Now

Compile and upload:

```bash
platformio run -e uno_r4_wifi
platformio run -e uno_r4_wifi --target upload
```

Then test:
- Open Serial Monitor at 115200 baud
- Place X pieces - you should now see:
  ```
  [Wrist] ToCell square N compensation=X current=73 clearance=Y
  [Wrist] compMs=... wristMs=...
  ```
- Wrist motor should visibly rotate at clearance height
- Pieces should arrive straight! 🎉

## The Issue in One Line

**Compensation was never applied because `pendingSquare` was set to 0 before the function checked it.**
