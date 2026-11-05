#!/usr/bin/env python3

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

"""Zero a Feetech SMS/STS servo and, optionally, set its EEPROM angle limits.

Bench utility, not an automated test -- it is deliberately not registered with
CMake, so ``colcon test`` never runs it. Hold or leave the axis at the pose you
want to be angle 0, then run this once. Add ``--limits`` to continue into a
guided walkthrough that captures the axis's physical travel and writes it to
the servo's own min/max angle limits.

WHY THE CENTRE TICK AND NOT A CONFIG OFFSET
-------------------------------------------
The driver has two ways to place zero, and they are not interchangeable:

``pan_home_ticks`` (software, in cobra_flex.yaml) is the right knob for small
mounting variance -- it lives in config and is visible in review. But travel is
clamped so it cannot cross the 0/4095 encoder wrap, so the widest symmetric
range around a home tick H is ``min(H, 4095 - H)``. That is maximised only at
H = 2048. A head that must pan nearly a full turn therefore *has* to have its
physical zero on the centre tick, and only the servo's own offset register can
put it there.

So: use ``pan_home_ticks`` for a head with modest travel, and this script for a
head that needs the full rotation. The cost is that the offset lives in servo
EEPROM -- invisible from config, and it travels with the servo if it is swapped.

WHAT IT WRITES
--------------
Sends the SMS/STS midpoint-calibration command (value 128 to the
TORQUE_ENABLE register, 40), bracketed by an EEPROM unlock/lock so the
resulting offset survives a power cycle. The firmware computes the OFS
register (31) itself. No motion is ever commanded: the servo keeps its
physical pose and only its idea of what to call that pose changes.

With ``--limits`` (or ``--limits-only``) it also writes the min/max angle
limit registers (9 and 11), again under an EEPROM unlock/lock. The servo
enforces these itself: position commands outside the window are silently
clamped, so a move that stalls exactly at a limit is the EEPROM, not an
obstruction. The window is a pair of absolute ticks and cannot cross the
0/4095 encoder wrap -- zero the axis first (mid-travel at 2048) so it never
has to.

THE LIMITS WALKTHROUGH
----------------------
Torque is released so the axis can be moved by hand (support a tilt head
before agreeing -- it goes limp). Move the axis to one end of its travel and
press Enter, then the same for the other end; the lower and higher captures
become the min and max limit. The window is confirmed before anything is
written, and the matching ``*_min/max_angle_deg`` driver config is printed
afterwards, rounded inward so the software limits stay inside the servo's
own. To write known values instead of capturing them by hand (e.g. opening a
full-rotation pan head to [0, 4095]), pass ``--min-ticks``/``--max-ticks``.

The driver does not read the servo's limits; its own ``*_min/max_angle_deg``
must be kept inside them by hand. So after a successful limits write the
script offers to update those two lines in the driver params file
(``--config``, default: the bringup cobra_flex.yaml next to this package).
The axis prefix is taken from ``--axis`` or inferred by matching ``--id``
against the file's ``pan_motor_id``/``tilt_motor_id``; the degrees are
measured from that axis's ``*_home_ticks`` in the file. Only those two lines
change; everything else in the file, comments included, is left as is.

USAGE
-----
Find the bus with ``ros2 run cobra_flex_bringup identify_serial_ports.py``,
stop any running driver (a second reader corrupts the bus), then::

    python3 zeroing.py --port /dev/ttyACM0            # confirm, then write
    python3 zeroing.py --port /dev/ttyACM0 --read-only  # inspect, write nothing
    python3 zeroing.py --port /dev/ttyACM0 --limits     # zero, then set limits
    python3 zeroing.py --port /dev/ttyACM0 --limits-only  # limits, no re-zero
    python3 zeroing.py --port /dev/ttyACM0 --limits-only --min-ticks 0 --max-ticks 4095
    python3 zeroing.py --port /dev/ttyACM0 --id 2 --limits --axis tilt \
        --config /cobra_flex_ros/src/bringup/config/cobra_flex.yaml

Exits 0 on success, 1 on any failure.
"""

import argparse
import math
import os
import re
import select
import sys
import time

import serial

# SMS/STS control table addresses (see scservo/include/scservo/SMS_STS.h).
ADDR_ID = 5
ADDR_MIN_ANGLE_LIMIT = 9
ADDR_MAX_ANGLE_LIMIT = 11
ADDR_OFS = 31
ADDR_MODE = 33
ADDR_TORQUE_ENABLE = 40
ADDR_LOCK = 55
ADDR_PRESENT_POSITION = 56

INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03

CALIBRATION_CMD = 128           # written to ADDR_TORQUE_ENABLE
MODE_SERVO = 0
TICKS_PER_REVOLUTION = 4096
CENTRE_TICK = TICKS_PER_REVOLUTION // 2
# The firmware lands within a tick or two of centre; anything wider means the
# command did not take (or the axis moved mid-write).
CENTRE_TOLERANCE_TICKS = 4
# Two limit captures closer than this are one pose, not a travel range.
MIN_LIMIT_SPAN_TICKS = 16
EEPROM_SETTLE_S = 0.25          # EEPROM writes are not instant
LIMIT_POLL_PERIOD_S = 0.1       # position refresh rate during limit capture
# Driver params file the limits step offers to update; the bringup package
# sits beside this one in the workspace.
DEFAULT_CONFIG = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '..', '..', 'bringup', 'config', 'cobra_flex.yaml'))
AXES = ('pan', 'tilt')


class ServoError(Exception):
    """A servo transaction failed or returned a malformed packet."""


def _checksum(payload):
    """Feetech packet checksum: bitwise NOT of the byte sum, low 8 bits."""
    return (~sum(payload)) & 0xFF


def _send(ser, servo_id, instruction, params):
    """Write one instruction packet to the bus."""
    body = [servo_id, len(params) + 2, instruction] + list(params)
    ser.reset_input_buffer()
    ser.write(bytes([0xFF, 0xFF] + body + [_checksum(body)]))


def _read_status(ser, servo_id, param_len):
    """Read and validate a status packet, returning its parameter bytes."""
    packet = ser.read(6 + param_len)
    if len(packet) != 6 + param_len:
        raise ServoError(
            f'no reply from ID {servo_id} (got {len(packet)} of '
            f'{6 + param_len} bytes) -- wrong port, wrong baud, unpowered '
            f'bus, or another process is holding the port')
    if packet[0] != 0xFF or packet[1] != 0xFF:
        raise ServoError(f'bad packet header from ID {servo_id}: {packet.hex()}')
    if packet[2] != servo_id:
        raise ServoError(
            f'reply came from ID {packet[2]}, not {servo_id}: {packet.hex()}')
    if _checksum(packet[2:-1]) != packet[-1]:
        raise ServoError(f'checksum mismatch from ID {servo_id}: {packet.hex()}')
    return list(packet[5:5 + param_len])


def ping(ser, servo_id):
    """Return True if the servo answers a PING."""
    try:
        _send(ser, servo_id, INST_PING, [])
        _read_status(ser, servo_id, 0)
    except ServoError:
        return False
    return True


def read_word(ser, servo_id, address):
    """Read a little-endian 16-bit register."""
    _send(ser, servo_id, INST_READ, [address, 2])
    low, high = _read_status(ser, servo_id, 2)
    return low | (high << 8)


def read_byte(ser, servo_id, address):
    """Read an 8-bit register."""
    _send(ser, servo_id, INST_READ, [address, 1])
    return _read_status(ser, servo_id, 1)[0]


def write_byte(ser, servo_id, address, value):
    """Write an 8-bit register and consume its acknowledgement."""
    _send(ser, servo_id, INST_WRITE, [address, value])
    _read_status(ser, servo_id, 0)


def write_word(ser, servo_id, address, value):
    """Write a little-endian 16-bit register and consume its acknowledgement."""
    _send(ser, servo_id, INST_WRITE,
          [address, value & 0xFF, (value >> 8) & 0xFF])
    _read_status(ser, servo_id, 0)


def signed_offset(raw):
    """Decode the OFS register, whose bit 11 is a sign bit rather than a value."""
    magnitude = raw & 0x7FF
    return -magnitude if raw & 0x800 else magnitude


def ticks_to_degrees(ticks):
    """Convert a tick count to degrees."""
    return ticks * 360.0 / TICKS_PER_REVOLUTION


def snapshot(ser, servo_id):
    """Collect the registers that describe where the servo thinks zero is."""
    return {
        'id': read_byte(ser, servo_id, ADDR_ID),
        'position': read_word(ser, servo_id, ADDR_PRESENT_POSITION),
        'offset': signed_offset(read_word(ser, servo_id, ADDR_OFS)),
        'min_limit': read_word(ser, servo_id, ADDR_MIN_ANGLE_LIMIT),
        'max_limit': read_word(ser, servo_id, ADDR_MAX_ANGLE_LIMIT),
        'mode': read_byte(ser, servo_id, ADDR_MODE),
    }


def print_snapshot(label, state):
    """Print one register snapshot as an indented block."""
    offset_from_centre = state['position'] - CENTRE_TICK
    print(f'  {label}')
    print(f'    ID .................. {state["id"]}')
    print(f'    present position .... {state["position"]} ticks '
          f'({offset_from_centre:+d} from centre, '
          f'{ticks_to_degrees(offset_from_centre):+.1f} deg)')
    print(f'    offset (OFS) ........ {state["offset"]}')
    print(f'    angle limits ........ [{state["min_limit"]}, {state["max_limit"]}]')
    print(f'    mode ................ {state["mode"]} (0 = servo)')


def set_servo_mode(ser, servo_id):
    """Put the servo into position (servo) mode, as the SDK's InitMotor does."""
    write_byte(ser, servo_id, ADDR_LOCK, 0)
    time.sleep(EEPROM_SETTLE_S)
    write_byte(ser, servo_id, ADDR_MODE, MODE_SERVO)
    time.sleep(EEPROM_SETTLE_S)
    write_byte(ser, servo_id, ADDR_LOCK, 1)
    time.sleep(EEPROM_SETTLE_S)


def calibrate(ser, servo_id):
    """Unlock EEPROM, send the midpoint-calibration command, lock it again."""
    write_byte(ser, servo_id, ADDR_LOCK, 0)
    time.sleep(EEPROM_SETTLE_S)
    write_byte(ser, servo_id, ADDR_TORQUE_ENABLE, CALIBRATION_CMD)
    time.sleep(EEPROM_SETTLE_S)
    write_byte(ser, servo_id, ADDR_LOCK, 1)
    time.sleep(EEPROM_SETTLE_S)


def torque_off(ser, servo_id):
    """Release the servo so the axis can be moved by hand (RAM write, no lock)."""
    write_byte(ser, servo_id, ADDR_TORQUE_ENABLE, 0)


def write_limits(ser, servo_id, min_ticks, max_ticks):
    """Unlock EEPROM, write the min/max angle limit registers, lock it again."""
    write_byte(ser, servo_id, ADDR_LOCK, 0)
    time.sleep(EEPROM_SETTLE_S)
    write_word(ser, servo_id, ADDR_MIN_ANGLE_LIMIT, min_ticks)
    time.sleep(EEPROM_SETTLE_S)
    write_word(ser, servo_id, ADDR_MAX_ANGLE_LIMIT, max_ticks)
    time.sleep(EEPROM_SETTLE_S)
    write_byte(ser, servo_id, ADDR_LOCK, 1)
    time.sleep(EEPROM_SETTLE_S)


def parse_args(argv):
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description=__doc__.split('\n')[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        '--port', default='/dev/ttyACM0',
        help='servo bus serial device (default: %(default)s). Identify it with '
             'ros2 run cobra_flex_bringup identify_serial_ports.py')
    parser.add_argument('--id', type=int, default=1,
                        help='servo bus ID to zero (default: %(default)s)')
    parser.add_argument('--baud', type=int, default=1000000,
                        help='servo bus baud rate (default: %(default)s)')
    parser.add_argument('--read-only', action='store_true',
                        help='report the current registers and exit without writing')
    parser.add_argument('--set-servo-mode', action='store_true',
                        help='if the servo is left in wheel mode (the driver does '
                             'this on shutdown), switch it back to servo mode '
                             'before calibrating')
    parser.add_argument('--limits', action='store_true',
                        help='after zeroing, walk through capturing the axis '
                             'travel by hand and write it to the servo\'s EEPROM '
                             'angle limits')
    parser.add_argument('--limits-only', action='store_true',
                        help='only the angle-limit walkthrough; skip the '
                             'midpoint calibration')
    parser.add_argument('--min-ticks', type=int, default=None, metavar='TICKS',
                        help='with --limits/--limits-only: write this min limit '
                             'instead of capturing it by hand')
    parser.add_argument('--max-ticks', type=int, default=None, metavar='TICKS',
                        help='with --limits/--limits-only: write this max limit '
                             'instead of capturing it by hand')
    parser.add_argument('--config', default=DEFAULT_CONFIG, metavar='YAML',
                        help='driver params file whose *_min/max_angle_deg the '
                             'limits step offers to update (default: '
                             '%(default)s)')
    parser.add_argument('--axis', choices=AXES, default=None,
                        help='which axis prefix to update in --config; by '
                             'default inferred by matching --id against the '
                             'file\'s pan_motor_id/tilt_motor_id')
    parser.add_argument('--no-config', action='store_true',
                        help='never offer to update --config')
    parser.add_argument('-y', '--yes', action='store_true',
                        help='skip the confirmation prompts (the config update '
                             'is still asked about)')
    args = parser.parse_args(argv)
    if (args.min_ticks is None) != (args.max_ticks is None):
        parser.error('--min-ticks and --max-ticks must be given together')
    if args.min_ticks is not None:
        if not (args.limits or args.limits_only):
            parser.error('--min-ticks/--max-ticks require --limits or '
                         '--limits-only')
        if not 0 <= args.min_ticks < args.max_ticks <= TICKS_PER_REVOLUTION - 1:
            parser.error(f'angle limits must satisfy 0 <= min < max <= '
                         f'{TICKS_PER_REVOLUTION - 1}')
    return args


def confirm(servo_id):
    """Ask before writing EEPROM. Returns True to proceed."""
    print('\n  This writes the servo EEPROM: the pose above becomes the new')
    print('  centre tick (2048), and that survives power cycles. Make sure the')
    print('  axis is physically where you want angle 0 before continuing.\n')
    try:
        reply = input(f'  Zero servo ID {servo_id} at its current pose? [y/N] ')
    except EOFError:
        return False
    return reply.strip().lower() in ('y', 'yes')


def capture_extreme(ser, servo_id, step_label):
    """Live-print the position while the user moves the axis; Enter captures.

    Returns the last position sampled before Enter was pressed, so letting go
    of the axis to reach the keyboard cannot corrupt the capture.
    """
    print(f'\n  {step_label}')
    if not sys.stdin.isatty():
        input('    press Enter to capture the position: ')
        return read_word(ser, servo_id, ADDR_PRESENT_POSITION)
    print('    press Enter to capture the position')
    last = None
    while True:
        ready, _, _ = select.select([sys.stdin], [], [], LIMIT_POLL_PERIOD_S)
        if ready:
            sys.stdin.readline()
            break
        try:
            last = read_word(ser, servo_id, ADDR_PRESENT_POSITION)
        except ServoError:
            continue    # one lost packet should not abort the walkthrough
        offset = last - CENTRE_TICK
        print(f'\r    position: {last:4d} ticks ({offset:+5d} from centre, '
              f'{ticks_to_degrees(offset):+7.1f} deg) ', end='', flush=True)
    print()
    if last is None:
        last = read_word(ser, servo_id, ADDR_PRESENT_POSITION)
    return last


def _config_param(text, key):
    """Return the scalar value of `key:` in a ROS params YAML, or None.

    A regex rather than a YAML parser so the later rewrite can preserve the
    file byte-for-byte apart from the two lines it changes.
    """
    match = re.search(rf'^(\s*){re.escape(key)}\s*:\s*([^\s#]+)', text, re.M)
    return match.group(2) if match else None


def _infer_axis(text, servo_id):
    """Pick the axis whose *_motor_id in the config equals servo_id."""
    hits = [axis for axis in AXES
            if _config_param(text, f'{axis}_motor_id') == str(servo_id)]
    return hits[0] if len(hits) == 1 else None


def _replace_param(text, key, value):
    """Rewrite the value of `key:` in place, keeping indent and any comment."""
    pattern = rf'^(\s*{re.escape(key)}\s*:\s*)[^\s#]+'
    new, count = re.subn(pattern, rf'\g<1>{value}', text, count=1, flags=re.M)
    if count != 1:
        raise KeyError(key)
    return new


def offer_config_update(args, low, high):
    """Prompt to write the driver's software limits for this axis into --config.

    The driver measures angles from its own *_home_ticks, not necessarily
    2048, so the degrees are recomputed from the file's value. Returns True
    if the file was updated; a refusal or a problem with the file is
    reported and returns False -- the EEPROM write has already succeeded.
    """
    if args.no_config:
        return False
    path = args.config
    try:
        with open(path) as f:
            text = f.read()
    except OSError as exc:
        print(f'\n  (not updating driver config: {exc})')
        return False

    axis = args.axis or _infer_axis(text, args.id)
    if axis is None:
        print(f'\n  (not updating {path}: cannot tell which axis servo ID '
              f'{args.id} is -- pass --axis pan|tilt)')
        return False

    home_raw = _config_param(text, f'{axis}_home_ticks')
    home = int(home_raw) if home_raw is not None else CENTRE_TICK
    if not low <= home <= high:
        print(f'\n  (not updating {path}: {axis}_home_ticks {home} lies '
              f'outside the new window [{low}, {high}]; fix home first)')
        return False
    # Round inward so the driver never commands a tick the servo would clamp.
    min_deg = math.ceil(ticks_to_degrees(low - home) * 10) / 10
    max_deg = math.floor(ticks_to_degrees(high - home) * 10) / 10
    min_key, max_key = f'{axis}_min_angle_deg', f'{axis}_max_angle_deg'
    old_min, old_max = _config_param(text, min_key), _config_param(text, max_key)
    if old_min is None or old_max is None:
        print(f'\n  (not updating {path}: {min_key}/{max_key} not found)')
        return False

    print(f'\n  Driver config {path}')
    print(f'    {axis}_home_ticks ... {home} '
          f'({"from file" if home_raw is not None else "default"})')
    print(f'    {min_key} ... {old_min} -> {min_deg:.1f}')
    print(f'    {max_key} ... {old_max} -> {max_deg:.1f}')
    print('  The driver does not read the servo limits; these keep its own '
          'window inside them.')
    try:
        reply = input('  Update these two lines now? [y/N] ')
    except EOFError:
        reply = ''
    if reply.strip().lower() not in ('y', 'yes'):
        print('  Config left unchanged; copy the values above by hand.')
        return False

    text = _replace_param(text, min_key, f'{min_deg:.1f}')
    text = _replace_param(text, max_key, f'{max_deg:.1f}')
    with open(path, 'w') as f:
        f.write(text)
    print(f'  Updated {path}. Rebuild/relaunch the driver to pick it up.')
    return True


def run_limits_walkthrough(ser, args):
    """Capture (or take from flags) the axis travel and write the angle limits.

    Returns a process exit code; ServoError propagates to main's handler.
    """
    servo_id = args.id
    if args.min_ticks is not None:
        low, high = args.min_ticks, args.max_ticks
        print(f'\n  Writing angle limits [{low}, {high}] from '
              '--min/--max-ticks; nothing is captured.')
        print('  Torque is released for the EEPROM write -- a tilt head goes '
              'limp.')
    else:
        print('\n  Limits walkthrough: the travel you now demonstrate by hand '
              'becomes the')
        print('  window the servo itself enforces. Torque is about to be '
              'released so the')
        print('  axis can be moved -- a tilt head goes limp; support it '
              'first.')
        try:
            input('\n  Press Enter to release torque: ')
        except EOFError:
            print('\nAborted; no limits written.')
            return 1
        torque_off(ser, servo_id)
        first = capture_extreme(
            ser, servo_id,
            'Step 1 of 2: move the axis by hand to ONE end of its travel.')
        second = capture_extreme(
            ser, servo_id,
            'Step 2 of 2: move the axis to the OTHER end of its travel.')
        low, high = min(first, second), max(first, second)
        if high - low < MIN_LIMIT_SPAN_TICKS:
            print(f'\nFAILED: the two captures ({first} and {second}) are only '
                  f'{high - low} ticks apart -- that is one pose, not a '
                  'travel range.', file=sys.stderr)
            return 1

    low_deg = ticks_to_degrees(low - CENTRE_TICK)
    high_deg = ticks_to_degrees(high - CENTRE_TICK)
    print(f'\n  min limit ... {low} ticks ({low_deg:+.1f} deg from centre)')
    print(f'  max limit ... {high} ticks ({high_deg:+.1f} deg from centre)')
    if not low <= CENTRE_TICK <= high:
        print('\n  WARNING: the centre tick (2048, angle 0, the default home) '
              'is outside')
        print('  this window. Either the axis was not zeroed at mid-travel '
              'first, or its')
        print('  travel crosses the 0/4095 wrap, which these limits cannot '
              'express.')

    if not args.yes:
        print('\n  This writes the servo EEPROM: position commands outside '
              'the window are')
        print('  silently clamped by the servo, and that survives power '
              'cycles.\n')
        try:
            reply = input(f'  Write angle limits [{low}, {high}] to servo ID '
                          f'{servo_id}? [y/N] ')
        except EOFError:
            reply = ''
        if reply.strip().lower() not in ('y', 'yes'):
            print('\nAborted; no limits written.')
            return 1

    torque_off(ser, servo_id)
    write_limits(ser, servo_id, low, high)
    after = snapshot(ser, servo_id)
    print()
    print_snapshot('after limits write:', after)
    if (after['min_limit'], after['max_limit']) != (low, high):
        print(f'\nFAILED: limits read back as [{after["min_limit"]}, '
              f'{after["max_limit"]}], not the [{low}, {high}] just written.',
              file=sys.stderr)
        return 1

    # Round inward (min up, max down) so the software limits derived from
    # these can never command a tick the servo would clamp -- a clamped move
    # never reaches its target and the driver times out waiting.
    min_deg = math.ceil(low_deg * 10) / 10
    max_deg = math.floor(high_deg * 10) / 10
    print(f'\nOK: the servo now clamps position commands to ticks '
          f'[{low}, {high}].')
    print('Torque was left off; the driver enables it again on startup.')
    print('\nMatching driver config (pan_ or tilt_ prefix for this axis, '
          'home at 2048):')
    print(f'    *_min_angle_deg: {min_deg:.1f}')
    print(f'    *_max_angle_deg: {max_deg:.1f}')
    offer_config_update(args, low, high)
    return 0


def main(argv=None):
    """Zero one servo and verify the result. Returns a process exit code."""
    args = parse_args(argv)

    try:
        with serial.Serial(args.port, args.baud, timeout=0.3) as ser:
            time.sleep(0.05)
            if not ping(ser, args.id):
                print(f'No servo answered a PING at ID {args.id} on {args.port} '
                      f'@ {args.baud} baud.', file=sys.stderr)
                print('Check the port, and stop any running pan_tilt_driver -- a '
                      'second reader corrupts the bus.', file=sys.stderr)
                return 1

            print(f'\nServo ID {args.id} on {args.port} @ {args.baud} baud\n')
            before = snapshot(ser, args.id)
            print_snapshot('before:', before)

            if args.read_only:
                print('\n--read-only: nothing written.')
                return 0

            do_zero = not args.limits_only
            do_limits = args.limits or args.limits_only
            capture_needed = do_limits and args.min_ticks is None

            # Calibrating or capturing limits in wheel mode bakes in an error.
            # The position register is reported in a different frame there --
            # on this head it read 85 ticks (7.5 deg) high -- so the pose the
            # firmware records (or the extreme you capture) would not be the
            # pose you are looking at. Explicit --min/--max-ticks writes read
            # no positions, so they skip this gate.
            if (do_zero or capture_needed) and before['mode'] != MODE_SERVO:
                if not args.set_servo_mode:
                    print(f'\nServo is in mode {before["mode"]}, not servo mode '
                          f'({MODE_SERVO}).', file=sys.stderr)
                    print('Its position register reads in a different frame '
                          'there, so calibrating now would record the wrong '
                          'pose as centre.', file=sys.stderr)
                    print('The driver leaves the servo in wheel mode on '
                          'shutdown. Re-run with --set-servo-mode to switch it '
                          'back first.', file=sys.stderr)
                    return 1
                print('\n  servo is in wheel mode; switching to servo mode first')
                set_servo_mode(ser, args.id)
                before = snapshot(ser, args.id)
                print()
                print_snapshot('after mode switch:', before)
                if before['mode'] != MODE_SERVO:
                    print(f'\nFAILED: servo still reports mode '
                          f'{before["mode"]}.', file=sys.stderr)
                    return 1

            if do_zero:
                if not args.yes and not confirm(args.id):
                    print('\nAborted; nothing written.')
                    return 1

                calibrate(ser, args.id)
                after = snapshot(ser, args.id)
                print()
                print_snapshot('after zeroing:', after)

                drift = after['position'] - CENTRE_TICK
                if abs(drift) > CENTRE_TOLERANCE_TICKS:
                    print(f'\nFAILED: position reads {after["position"]}, '
                          f'which is {drift:+d} ticks from centre '
                          f'({CENTRE_TICK}).', file=sys.stderr)
                    print('The calibration command did not take. The servo '
                          'may not support it, or the axis moved mid-write.',
                          file=sys.stderr)
                    return 1

                moved = after['position'] - before['position']
                print(f'\nOK: this pose now reads {after["position"]} ticks '
                      f'(was {before["position"]}, a {moved:+d} tick relabel).')
                print('Nothing rotated -- only the servo\'s idea of zero '
                      'changed.')
                if not do_limits:
                    print('\nWith the physical zero on the centre tick, the '
                          'driver can use the full')
                    print('travel: pan_home_ticks: 2048 with '
                          'pan_min/max_angle_deg: -/+179.9.')

            if do_limits:
                return run_limits_walkthrough(ser, args)
    except serial.SerialException as exc:
        print(f'Serial error on {args.port}: {exc}', file=sys.stderr)
        return 1
    except ServoError as exc:
        print(f'Servo error: {exc}', file=sys.stderr)
        return 1

    return 0


if __name__ == '__main__':
    sys.exit(main())
