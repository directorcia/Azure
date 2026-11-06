#pragma once

#include <stdint.h>
#include <initializer_list>

namespace tic_tac_toe {
struct Board {
  char cells[9] = {};

  bool place(int square, char player) {
    if (square < 1 || square > 9 || cells[square - 1] != 0 ||
        (player != 'X' && player != 'O')) return false;
    cells[square - 1] = player;
    return true;
  }

  char winner() const {
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

  bool full() const {
    for (char cell : cells) if (cell == 0) return false;
    return true;
  }
};

struct BestMoves {
  uint16_t squares = 0;
  int score = 0;

  bool contains(int square) const {
    return square >= 1 && square <= 9 && (squares & (uint16_t(1) << (square - 1))) != 0;
  }
};

inline int minimax(Board &board, char player, int depth, int alpha, int beta) {
  char winner = board.winner();
  if (winner == 'X') return 10 - depth;
  if (winner == 'O') return depth - 10;
  if (board.full()) return 0;

  int best = player == 'X' ? -100 : 100;
  for (int square = 0; square < 9; square++) {
    if (board.cells[square] != 0) continue;
    board.cells[square] = player;
    int score = minimax(board, player == 'X' ? 'O' : 'X', depth + 1, alpha, beta);
    board.cells[square] = 0;
    if (player == 'X') {
      if (score > best) best = score;
      if (best > alpha) alpha = best;
    } else {
      if (score < best) best = score;
      if (best < beta) beta = best;
    }
    if (alpha >= beta) break;
  }
  return best;
}

inline BestMoves bestMovesForX(Board board) {
  BestMoves result;
  if (board.winner() != 0 || board.full()) return result;
  result.score = -100;
  for (int square = 0; square < 9; square++) {
    if (board.cells[square] != 0) continue;
    board.cells[square] = 'X';
    int score = minimax(board, 'O', 1, -100, 100);
    board.cells[square] = 0;
    if (score > result.score) {
      result.score = score;
      result.squares = 0;
    }
    if (score == result.score) result.squares |= uint16_t(1) << square;
  }
  return result;
}

inline bool selfTest() {
  Board invalid;
  if (invalid.place(0, 'X') || invalid.place(10, 'O') || invalid.place(1, '?')) return false;
  if (!invalid.place(1, 'X') || invalid.place(1, 'O')) return false;
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
  Board draw;
  const char drawnCells[9] = {'X', 'O', 'X', 'X', 'O', 'O', 'O', 'X', 'X'};
  for (int square = 1; square <= 9; square++) draw.place(square, drawnCells[square - 1]);
  if (!draw.full() || draw.winner() != 0 || bestMovesForX(draw).squares != 0) return false;

  Board empty;
  BestMoves opening = bestMovesForX(empty);
  if (opening.score != 0 || opening.squares != 0x1FF) return false;

  Board winning;
  winning.place(1, 'X'); winning.place(2, 'X');
  winning.place(4, 'O'); winning.place(5, 'O');
  BestMoves win = bestMovesForX(winning);
  if (win.score != 9 || win.squares != (1 << 2)) return false;

  Board block;
  block.place(1, 'O'); block.place(2, 'O'); block.place(5, 'X');
  if (bestMovesForX(block).squares != (1 << 2)) return false;

  Board fork;
  fork.place(1, 'O'); fork.place(9, 'O'); fork.place(5, 'X');
  BestMoves forkDefense = bestMovesForX(fork);
  if (forkDefense.score != 0 || forkDefense.squares != ((1 << 1) | (1 << 3) | (1 << 5) | (1 << 7))) return false;

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