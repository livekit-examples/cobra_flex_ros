# Copyright 2026 LiveKit
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Animated "LiveKit" screensaver frames for a character-cell display.

Pure component (no ROS, no I/O): callers pass a monotonic timestamp to
:meth:`LKGraphicProducer.render` and get back the full frame as a list of
fixed-width strings, one per display line. The caller decides how frames get
to the screen (the Cobra Flex driver diffs lines and sends ``{"T":3}`` OLED
commands).

Animation, per word instance:

- a new instance spawns every ``spawn_period`` (50 ms) at a random spot,
  starting with one random letter of the word shown at its in-word position;
  each spawn lands on the opposite half of the display from the previous one,
  horizontally or vertically, so consecutive words bounce across the screen;
- every ``reveal_period`` (25 ms) another random remaining letter appears;
- once complete the word holds for ``hold_time`` (50 ms), then letters are
  removed every ``erase_period`` (10 ms), newest first;
- at most ``max_instances`` live at once, and spawn spots are chosen so
  instances overlap at most ``max_overlap_cells`` cells (light overlap only);
  a negative value ``-N`` instead demands an ``N``-cell empty margin
  (Chebyshev distance) around every word.

Letter reveal/erase times are precomputed at spawn, so a frame is a pure
function of ``now`` — rendering never depends on how often it is called.
"""

import random
from typing import List, Optional, Sequence, Set, Tuple


class _WordInstance:
    """One in-flight copy of the word: fixed spot, precomputed letter schedule."""

    def __init__(self, row: int, col: int, order: Sequence[int], t0: float,
                 reveal_period: float, hold_time: float, erase_period: float) -> None:
        self.row = row
        self.col = col
        self.order = list(order)  # letter indices, in reveal order
        self._t0 = t0
        self._reveal_period = reveal_period
        self._erase_period = erase_period
        n = len(self.order)
        # First letter shows at t0, the last at t0 + (n-1) * reveal_period.
        self._erase_start = t0 + (n - 1) * reveal_period + hold_time
        self.end_time = self._erase_start + n * erase_period

    def visible_letters(self, now: float) -> Sequence[int]:
        """Letter indices visible at ``now``, or empty when spent.

        Erasing newest-first means the visible set is always a prefix of the
        reveal order: ``revealed`` grows from the front, ``erased`` trims from
        the back.
        """
        n = len(self.order)
        revealed = min(n, 1 + int((now - self._t0) / self._reveal_period))
        erased = min(n, max(0, int((now - self._erase_start) / self._erase_period)))
        return self.order[:revealed - erased]

    def cells(self) -> Set[Tuple[int, int]]:
        """Every cell this word can touch (its full footprint, not just visible)."""
        return {(self.row, self.col + k) for k in range(len(self.order))}


class LKGraphicProducer:
    """Produces frames of overlapping type-on/type-off "LiveKit" words."""

    def __init__(self,
                 width: int = 21,
                 height: int = 4,
                 word: str = 'LiveKit',
                 spawn_period: float = 0.150,
                 reveal_period: float = 0.075,
                 hold_time: float = 0.250,
                 erase_period: float = 0.100,
                 max_instances: int = 7,
                 max_overlap_cells: int = 0,
                 spawn_attempts: int = 8,
                 rng: Optional[random.Random] = None) -> None:
        if not word:
            raise ValueError('word must be non-empty')
        if width < len(word) or height < 1:
            raise ValueError(
                f'display {width}x{height} cannot fit {word!r} ({len(word)} cells)')
        if min(spawn_period, reveal_period, erase_period) <= 0.0 or hold_time < 0.0:
            raise ValueError('animation periods must be > 0 (hold_time >= 0)')
        self._width = width
        self._height = height
        self._word = word
        self._spawn_period = spawn_period
        self._reveal_period = reveal_period
        self._hold_time = hold_time
        self._erase_period = erase_period
        self._max_instances = max_instances
        self._max_overlap_cells = max_overlap_cells
        self._spawn_attempts = spawn_attempts
        self._rng = rng if rng is not None else random.Random()
        self._instances: List[_WordInstance] = []
        self._next_spawn: Optional[float] = None
        # Opposite-half spawn rule state. An axis only participates if the
        # display is big enough for a word to sit on either of its halves.
        self._last_spawn: Optional[Tuple[int, int]] = None
        self._h_flippable = self._col_side(0) != self._col_side(width - len(word))
        self._v_flippable = self._row_side(0) != self._row_side(height - 1)

    def render(self, now: float) -> List[str]:
        """Advance to ``now`` and return the frame as ``height`` fixed-width lines."""
        self._instances = [i for i in self._instances if now < i.end_time]

        if self._next_spawn is None or now - self._next_spawn > 4 * self._spawn_period:
            self._next_spawn = now  # first frame, or resync after a stall
        while now >= self._next_spawn:
            self._try_spawn(self._next_spawn)
            self._next_spawn += self._spawn_period

        grid = [[' '] * self._width for _ in range(self._height)]
        for inst in self._instances:  # oldest first; newer words draw on top
            for k in inst.visible_letters(now):
                grid[inst.row][inst.col + k] = self._word[k]
        return [''.join(line) for line in grid]

    def _col_side(self, col: int) -> bool:
        """Which horizontal half a word at ``col`` sits on (by its center cell)."""
        return 2 * col + (len(self._word) - 1) >= self._width - 1

    def _row_side(self, row: int) -> bool:
        """Which vertical half a word at ``row`` sits on."""
        return 2 * row >= self._height - 1

    def _flips_side(self, row: int, col: int) -> bool:
        """Opposite-half rule: a new word must land on the other half of the
        display than the previous spawn, horizontally OR vertically. Axes the
        display cannot split (``*_flippable`` False) are ignored; the first
        spawn goes anywhere.
        """
        if self._last_spawn is None or not (self._h_flippable or self._v_flippable):
            return True
        last_row, last_col = self._last_spawn
        return ((self._h_flippable and self._col_side(col) != self._col_side(last_col))
                or (self._v_flippable and self._row_side(row) != self._row_side(last_row)))

    def _try_spawn(self, t0: float) -> None:
        """Spawn one word at ``t0`` if an acceptable spot exists; else skip.

        ``max_overlap_cells >= 0`` allows that many cells of overlap with the
        live words; ``-N`` inflates the candidate footprint by ``N`` cells in
        every direction and allows no contact at all, guaranteeing an ``N``-cell
        gap between words.
        """
        if len(self._instances) >= self._max_instances:
            return
        occupied: Set[Tuple[int, int]] = set()
        for inst in self._instances:
            occupied |= inst.cells()
        margin = max(0, -self._max_overlap_cells)
        allowed = max(0, self._max_overlap_cells)
        max_col = self._width - len(self._word)
        for _ in range(self._spawn_attempts):
            row = self._rng.randrange(self._height)
            col = self._rng.randrange(max_col + 1)
            if not self._flips_side(row, col):
                continue
            candidate = {
                (r, c)
                for r in range(row - margin, row + margin + 1)
                for c in range(col - margin, col + len(self._word) + margin)
            }
            if len(candidate & occupied) <= allowed:
                order = list(range(len(self._word)))
                self._rng.shuffle(order)
                self._instances.append(_WordInstance(
                    row, col, order, t0,
                    self._reveal_period, self._hold_time, self._erase_period))
                self._last_spawn = (row, col)
                return
