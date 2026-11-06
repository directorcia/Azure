#pragma once

#include <stdint.h>

// Pure board comparison so the camera rule can be read on its own.
// known[i] is 0 (empty), 'X', or 'O' — the same layout as tic_tac_toe::Board.
// seen[i] is the camera character for cell i+1: 'X', 'O', or '.'.
// human / robot are the symbols this game already uses (human is O, arm is X).

enum class CameraDiffKind : uint8_t {
  OneHumanMove,   // exactly one empty cell now holds the human piece; nothing else changed
  NoChange,       // camera matches the known board
  RobotMissing,   // a robot piece is gone or replaced
  NotOneNewHuman  // two or more cells differ, or the one change is not a new human piece
};

struct CameraDiff {
  CameraDiffKind kind;
  int square;  // 1..9 when kind is OneHumanMove, otherwise 0
  int changes;
};

inline CameraDiff diffCameraBoard(const char known[9], const char seen[9], char human, char robot) {
  CameraDiff diff = {CameraDiffKind::NoChange, 0, 0};
  bool robotMissing = false;
  int newHuman = 0;

  for (int i = 0; i < 9; i++) {
    char expected = known[i] == 0 ? '.' : known[i];
    if (seen[i] == expected) continue;
    diff.changes++;
    if (known[i] == 0 && seen[i] == human) {
      newHuman++;
      diff.square = i + 1;
    } else if (known[i] == robot && seen[i] != robot) {
      robotMissing = true;
    }
  }

  if (robotMissing) {
    diff.kind = CameraDiffKind::RobotMissing;
    diff.square = 0;
    return diff;
  }
  if (diff.changes == 0) return diff;
  if (diff.changes == 1 && newHuman == 1) {
    diff.kind = CameraDiffKind::OneHumanMove;
    return diff;
  }
  diff.kind = CameraDiffKind::NotOneNewHuman;
  diff.square = 0;
  return diff;
}
