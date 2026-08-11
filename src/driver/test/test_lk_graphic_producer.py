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

"""Offline tests for the animated OLED graphic (no hardware needed)."""

import random

import pytest

from cobra_flex_driver.lk_graphic_producer import LKGraphicProducer

WORD = 'LiveKit'


def make(seed=1, **kwargs):
    # Pin the timings the lifecycle tests probe against, so retuning the
    # producer's defaults doesn't silently shift the test schedule.
    kwargs.setdefault('spawn_period', 0.050)
    kwargs.setdefault('reveal_period', 0.025)
    kwargs.setdefault('hold_time', 0.050)
    kwargs.setdefault('erase_period', 0.010)
    return LKGraphicProducer(rng=random.Random(seed), **kwargs)


def visible_chars(lines):
    return [c for line in lines for c in line if c != ' ']


def test_frame_shape_is_fixed():
    producer = make()
    for t in (0.0, 0.013, 0.5, 3.0):
        lines = producer.render(t)
        assert len(lines) == 4
        assert all(len(line) == 21 for line in lines)


def test_single_instance_lifecycle():
    # One spawn only: next spawns land where the first word already sits.
    producer = make(width=7, height=1, max_instances=15, max_overlap_cells=0)
    assert len(visible_chars(producer.render(0.0))) == 1  # first random letter

    # A new letter every 25 ms until all 7 are up at t = 150 ms.
    for k in range(1, 7):
        assert len(visible_chars(producer.render(k * 0.025 + 0.001))) == k + 1

    # Fully printed: the line reads exactly "LiveKit" through the 50 ms hold.
    assert producer.render(0.155)[0] == WORD
    assert producer.render(0.205)[0] == WORD

    # Then one letter drops every 10 ms, and revealed positions still show
    # their correct in-word character while erasing.
    for k in range(1, 7):
        lines = producer.render(0.200 + k * 0.010 + 0.001)
        chars = visible_chars(lines)
        assert len(chars) == 7 - k
        for row, line in enumerate(lines):
            for col, c in enumerate(line):
                assert c == ' ' or c == WORD[col]
    assert visible_chars(producer.render(0.271)) == []


def test_erase_removes_newest_first():
    producer = make(width=7, height=1, max_overlap_cells=0)
    producer.render(0.0)  # anchor the (single) spawn at t = 0
    reveal_sequence = []
    for k in range(7):
        cells = {i for i, c in enumerate(producer.render(k * 0.025 + 0.001)[0]) if c != ' '}
        reveal_sequence.append(cells)
    erase_sequence = []
    for k in range(1, 8):
        cells = {i for i, c in enumerate(producer.render(0.200 + k * 0.010 + 0.001)[0])
                 if c != ' '}
        erase_sequence.append(cells)
    # Erasing newest-to-oldest walks back through the reveal snapshots.
    assert erase_sequence == reveal_sequence[-2::-1] + [set()]


def test_instance_cap():
    # A huge hold keeps every instance alive and unrestricted overlap lets
    # every spawn succeed; the population must still stop at the cap.
    producer = make(hold_time=1000.0, max_instances=15, max_overlap_cells=21 * 4)
    for step in range(100):  # one spawn period per step
        producer.render(step * 0.050)
    assert len(producer._instances) == 15


def test_overlap_stays_light():
    # Each word is placed overlapping the union of the words alive at its
    # spawn by <= 2 cells; live instances are kept in spawn order, so the
    # invariant is checkable against every earlier live footprint.
    producer = make(max_overlap_cells=2)
    for step in range(400):
        producer.render(step * 0.010)
        live = producer._instances
        for i, inst in enumerate(live):
            earlier = set().union(*(o.cells() for o in live[:i])) if i else set()
            assert len(inst.cells() & earlier) <= 2


def test_negative_overlap_enforces_margin():
    # max_overlap_cells = -1 -> at least one empty cell (Chebyshev) between
    # any two live footprints, i.e. never adjacent, not even diagonally.
    producer = make(max_overlap_cells=-1)
    for step in range(400):
        producer.render(step * 0.010)
        live = producer._instances
        for i, a in enumerate(live):
            for b in live[i + 1:]:
                gap = min(
                    max(abs(ra - rb), abs(ca - cb))
                    for ra, ca in a.cells() for rb, cb in b.cells())
                assert gap >= 2


def test_consecutive_spawns_flip_display_half():
    # Every spawn must sit on the opposite half of the 21x4 display from the
    # previous one, horizontally (word center vs column 10) or vertically
    # (rows 0-1 vs 2-3).
    producer = make(max_overlap_cells=len(WORD))  # permissive overlap: spawns rarely skip
    right = lambda col: col + (len(WORD) - 1) / 2 >= 10  # noqa: E731
    bottom = lambda row: row >= 2  # noqa: E731
    seen = set()  # holds real references: reused object ids can't alias spawns
    spawns = []
    for step in range(600):
        producer.render(step * 0.010)
        for inst in producer._instances:
            if inst not in seen:
                seen.add(inst)
                spawns.append((inst.row, inst.col))
    assert len(spawns) > 20
    for (r1, c1), (r2, c2) in zip(spawns, spawns[1:]):
        assert right(c1) != right(c2) or bottom(r1) != bottom(r2)


def test_letters_render_at_word_positions():
    producer = make()
    for step in range(0, 300):
        for row, line in enumerate(producer.render(step * 0.010)):
            for col, c in enumerate(line):
                if c != ' ':
                    # Every visible char belongs to "LiveKit" at a consistent
                    # in-word offset for some instance on that row.
                    assert any(
                        inst.row == row
                        and 0 <= col - inst.col < len(WORD)
                        and WORD[col - inst.col] == c
                        for inst in producer._instances)


def test_rejects_display_too_small_for_word():
    with pytest.raises(ValueError):
        LKGraphicProducer(width=6, height=1)
