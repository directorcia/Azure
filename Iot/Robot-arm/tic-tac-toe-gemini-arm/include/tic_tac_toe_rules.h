#pragma once

// Tic-tac-toe rules and a perfect-play search for the robot arm.
//
// The sketch calls this before it will move. selfTest() must pass at startup
// (rulesPassed); bestMovesForX() then names every equally optimal X square.
// Gemini is only allowed to pick from that set (see robotMove() in main.cpp).
//
// Squares are numbered 1..9, row-major, matching the web board and the pose
// table. Internally they are stored at index square - 1. A cell value of 0
// means empty; 'X' and 'O' are the only legal marks. Scores are always from
// X's point of view, because the arm is X.

#include <stdint.h>
#include <initializer_list>

namespace tic_tac_toe {
// Nine cells. place() is 1-based and rejects a full cell, a bad index, or a
// mark other than X or O. winner() / full() do not modify the board.
struct Board {
  char cells[9] = {};

  // Returns false and leaves the board unchanged when the move is illegal.
  bool place(int square, char player) {
    if (square < 1 || square > 9 || cells[square - 1] != 0 ||
        (player != 'X' && player != 'O')) return false;
    cells[square - 1] = player;
    return true;
  }

  // First matching line wins. Order does not matter: a legal board has at
  // most one three-in-a-row. Returns 0 when nobody has won yet.
  char winner() const {
    // Rows, then columns, then the two diagonals. Indexes are 0-based.
    const uint8_t lines[8][3] = {
      {0, 1, 2}, {3, 4, 5}, {6, 7, 8},
      {0, 3, 6}, {1, 4, 7}, {2, 5, 8},
      {0, 4, 8}, {2, 4, 6}
    };
    for (const auto &line : lines) {
      char player = cells[line[0]];
      if (player != 0 && player == cells[line[1]] && player == cells[line[2]]) return player;
    }
    return 0;
  }

  // True only when every cell is occupied. A won board can also be full;
  // callers check winner() first when the distinction matters.
  bool full() const {
    for (char cell : cells) if (cell == 0) return false;
    return true;
  }
};

// Every X move that shares the best minimax score.
// `squares` is a bitmask: bit (square - 1) set means that 1-based square is
// in the set. score == 0 with an empty mask means "no move" (game already
// over), which is distinct from a drawn evaluation of a live position.
struct BestMoves {
  uint16_t squares = 0;
  int score = 0;

  bool contains(int square) const {
    return square >= 1 && square <= 9 && (squares & (uint16_t(1) << (square - 1))) != 0;
  }
};

// Alpha-beta minimax. `board` is played in place and restored before return,
// so the caller's position is unchanged. `player` is whose turn it is.
// `depth` counts plies already played in this search (the root X move is
// depth 1, because bestMovesForX places it before calling in).
//
// Scores, from X's side, prefer a faster win and a slower loss:
//   X has won:  10 - depth
//   O has won:  depth - 10
//   draw:       0
// X maximises, O minimises. alpha is the best score X can already force,
// beta the best (lowest) score O can already force. A branch is cut once
// alpha >= beta, because the opponent would never walk into it.
inline int minimax(Board &board, char player, int depth, int alpha, int beta) {
  char winner = board.winner();
  if (winner == 'X') return 10 - depth;
  if (winner == 'O') return depth - 10;
  if (board.full()) return 0;

  // Sentinels sit outside the real score range (about -9..10), so the first
  // legal child always replaces them.
  int best = player == 'X' ? -100 : 100;
  for (int square = 0; square < 9; square++) {
    if (board.cells[square] != 0) continue;
    board.cells[square] = player;
    int score = minimax(board, player == 'X' ? 'O' : 'X', depth + 1, alpha, beta);
    board.cells[square] = 0;  // undo, so the next sibling sees the same node
    if (player == 'X') {
      if (score > best) best = score;
      if (best > alpha) alpha = best;
    } else {
      if (score < best) best = score;
      if (best < beta) beta = best;
    }
    // The opponent already has a better line elsewhere. Stop exploring.
    if (alpha >= beta) break;
  }
  return best;
}

// Pass-by-value so the search can scribble on a copy. Returns every empty
// square whose X reply ties the maximum score. An already won or full board
// yields an empty mask and does not search.
inline BestMoves bestMovesForX(Board board) {
  BestMoves result;
  if (board.winner() != 0 || board.full()) return result;
  result.score = -100;
  for (int square = 0; square < 9; square++) {
    if (board.cells[square] != 0) continue;
    board.cells[square] = 'X';
    // Depth 1: this X mark is the first ply. An immediate win therefore
    // scores 10 - 1 = 9, which is what selfTest expects for a forced win.
    int score = minimax(board, 'O', 1, -100, 100);
    board.cells[square] = 0;
    if (score > result.score) {
      result.score = score;
      result.squares = 0;  // a strictly better score drops the previous ties
    }
    if (score == result.score) result.squares |= uint16_t(1) << square;
  }
  return result;
}

// Startup gate for the arm. Covers illegal moves, every winning line for both
// marks, a draw, the empty-board opening, a one-move win, a forced block, the
// classic corner-fork defense, and the centre-opening reply. Also checks that
// the search does not mutate the caller's board. Any failure returns false
// and main.cpp refuses to move.
inline bool selfTest() {
  Board invalid;
  if (invalid.place(0, 'X') || invalid.place(10, 'O') || invalid.place(1, '?')) return false;
  if (!invalid.place(1, 'X') || invalid.place(1, 'O')) return false;
  // Same eight lines as winner(), but 1-based, so place() can fill them.
  const uint8_t lines[8][3] = {
    {1, 2, 3}, {4, 5, 6}, {7, 8, 9},
    {1, 4, 7}, {2, 5, 8}, {3, 6, 9}, {1, 5, 9}, {3, 5, 7}
  };
  for (char player : {'X', 'O'}) {
    for (const auto &line : lines) {
      Board winning;
      for (uint8_t square : line) winning.place(square, player);
      if (winning.winner() != player) return false;
    }
  }
  // X O X
  // X O O
  // O X X  — full, no three-in-a-row, so the search must refuse to move.
  Board draw;
  const char drawnCells[9] = {'X', 'O', 'X', 'X', 'O', 'O', 'O', 'X', 'X'};
  for (int square = 1; square <= 9; square++) draw.place(square, drawnCells[square - 1]);
  if (!draw.full() || draw.winner() != 0 || bestMovesForX(draw).squares != 0) return false;

  // Perfect play from empty draws, and every square is an equally best first
  // move. 0x1FF is bits 0..8, i.e. squares 1..9.
  Board empty;
  BestMoves opening = bestMovesForX(empty);
  if (opening.score != 0 || opening.squares != 0x1FF) return false;

  // X X . / O O . / . . .  — X plays square 3 and wins at depth 1 (score 9).
  // Bit 2 is square 3.
  Board winning;
  winning.place(1, 'X'); winning.place(2, 'X');
  winning.place(4, 'O'); winning.place(5, 'O');
  BestMoves win = bestMovesForX(winning);
  if (win.score != 9 || win.squares != (1 << 2)) return false;

  // O O . / . X . / . . .  — X must block square 3 or O wins next turn.
  Board block;
  block.place(1, 'O'); block.place(2, 'O'); block.place(5, 'X');
  if (bestMovesForX(block).squares != (1 << 2)) return false;

  // O on opposite corners, X in the centre. The drawing replies are the four
  // edges (squares 2, 4, 6, 8 — bits 1, 3, 5, 7), not another corner.
  Board fork;
  fork.place(1, 'O'); fork.place(9, 'O'); fork.place(5, 'X');
  BestMoves forkDefense = bestMovesForX(fork);
  if (forkDefense.score != 0 || forkDefense.squares != ((1 << 1) | (1 << 3) | (1 << 5) | (1 << 7))) return false;

  // O took the centre. X's optimal replies are the four corners
  // (squares 1, 3, 7, 9 — bits 0, 2, 6, 8).
  Board centre;
  centre.place(5, 'O');
  BestMoves corners = bestMovesForX(centre);
  if (corners.score != 0 || corners.squares != ((1 << 0) | (1 << 2) | (1 << 6) | (1 << 8))) return false;
  for (int square = 0; square < 9; square++) {
    if (centre.cells[square] != (square == 4 ? 'O' : 0)) return false;
  }
  return true;
}
}
