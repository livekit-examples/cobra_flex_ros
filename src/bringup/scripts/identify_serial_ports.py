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

"""Identify which /dev/ttyACM* port is the Cobra Flex chassis vs the pan/tilt bus.

The robot exposes three USB CDC-ACM devices whose /dev/ttyACM* numbering
shuffles across boots:

  * chassis   -- CH343 bridge to the ESP32; speaks JSON, answers ``{"T":130}``
                 with a feedback frame at 115200 baud
  * pan/tilt  -- CH343 bridge to the Feetech STS servo bus; servo IDs 1/2
                 answer a PING instruction at 1 Mbaud
  * ESP32 native USB (Espressif JTAG/serial debug unit) -- debug-only, skipped

Each candidate port is identified by probing with both protocols (read-only:
no motion commands are sent). A port already held open by another process is
not probed -- a second reader corrupts traffic for both -- and is classified
from its USB serial number instead.

Run standalone or via ``ros2 run cobra_flex_bringup identify_serial_ports.py``.
Exits 0 when both ports were found, 1 otherwise. ``--json`` prints a
machine-readable result; the default output includes the stable
``/dev/serial/by-id`` paths to use in ``cobra_flex.yaml``.

Note: opening the chassis port can reset the ESP32 (DTR auto-download
circuit), so the probe allows a few seconds for it to boot and reply.
"""

import argparse
import glob
import json
import os
import sys
import time

import serial
from serial.tools import list_ports

ESPRESSIF_VID = 0x303A          # ESP32 native USB (debug-only)
CHASSIS_BAUD = 115200
PAN_TILT_BAUD = 1000000
SERVO_IDS = (1, 2)              # pan, tilt
CHASSIS_PROBE_TIMEOUT = 6.0     # port open may reset the ESP32; allow a boot
# Last known serial numbers, used only when a port is busy and can't be probed.
KNOWN_SERIALS = {'5AE6059088': 'chassis', '5A7C121888': 'pan_tilt'}


def find_holders(device: str) -> list:
    """Best-effort list of (pid, comm) with ``device`` open (own-user procs,
    or all procs when root -- e.g. inside the cobra container)."""
    real = os.path.realpath(device)
    holders = []
    for fd_dir in glob.glob('/proc/[0-9]*/fd'):
        try:
            for fd in os.listdir(fd_dir):
                if os.path.realpath(os.path.join(fd_dir, fd)) == real:
                    pid = fd_dir.split('/')[2]
                    with open(f'/proc/{pid}/comm') as f:
                        holders.append((int(pid), f.read().strip()))
                    break
        except OSError:
            continue  # process exited, or not ours to inspect
    return holders


def by_id_path(device: str) -> str:
    """Stable /dev/serial/by-id path for ``device``, or ``device`` itself."""
    real = os.path.realpath(device)
    for link in glob.glob('/dev/serial/by-id/*'):
        if os.path.realpath(link) == real:
            return link
    return device


def probe_pan_tilt(device: str) -> bool:
    """True if a Feetech STS servo answers a PING (no motion commanded)."""
    with serial.Serial(device, PAN_TILT_BAUD, timeout=0.15) as ser:
        for sid in SERVO_IDS:
            for _ in range(2):
                ser.reset_input_buffer()
                ser.write(bytes([0xFF, 0xFF, sid, 0x02, 0x01,
                                 (~(sid + 0x02 + 0x01)) & 0xFF]))
                resp = ser.read(6)
                if len(resp) == 6 and resp[0] == 0xFF and resp[1] == 0xFF:
                    return True
    return False


def probe_chassis(device: str) -> bool:
    """True if the port answers a ``{"T":130}`` poll with a JSON frame."""
    deadline = time.monotonic() + CHASSIS_PROBE_TIMEOUT
    with serial.Serial(device, CHASSIS_BAUD, timeout=0.5) as ser:
        ser.reset_input_buffer()
        while time.monotonic() < deadline:
            ser.write(b'{"T":130}\n')
            line = ser.readline()
            try:
                if isinstance(json.loads(line.decode('ascii')), dict):
                    return True
            except (ValueError, UnicodeDecodeError):
                continue  # boot noise or partial line; keep polling
    return False


def classify(port) -> dict:
    """Probe one ACM port and return its identification record."""
    info = {
        'device': port.device,
        'by_id': by_id_path(port.device),
        'usb_serial': port.serial_number,
        'role': 'unknown',
        'how': '',
    }
    if port.vid == ESPRESSIF_VID:
        info.update(role='esp32_debug', how='Espressif native USB (debug-only)')
        return info

    holders = find_holders(port.device)
    if holders:
        who = ', '.join(f'{comm}[{pid}]' for pid, comm in holders)
        info['how'] = f'port busy ({who}); matched by USB serial number'
        info['role'] = KNOWN_SERIALS.get(port.serial_number, 'unknown')
        return info

    try:
        if probe_pan_tilt(port.device):
            info.update(role='pan_tilt',
                        how=f'STS servo ping answered @ {PAN_TILT_BAUD} baud')
        elif probe_chassis(port.device):
            info.update(role='chassis',
                        how=f'JSON feedback frame @ {CHASSIS_BAUD} baud')
        else:
            info['how'] = ('no response to servo ping or {"T":130} poll '
                           '(unpowered, or held by a process this user '
                           'cannot see -- try as root)')
    except serial.SerialException as exc:
        info['how'] = f'probe failed: {exc}'
    return info


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--json', action='store_true',
                        help='print machine-readable JSON instead of a report')
    args = parser.parse_args()

    acm_ports = sorted((p for p in list_ports.comports()
                        if p.device.startswith('/dev/ttyACM')),
                       key=lambda p: p.device)
    if not acm_ports:
        print('No /dev/ttyACM* devices found.', file=sys.stderr)
        return 1

    results = [classify(p) for p in acm_ports]
    found = {r['role']: r for r in results}

    if args.json:
        print(json.dumps(results, indent=2))
    else:
        for r in results:
            label = {'chassis': 'COBRA CHASSIS', 'pan_tilt': 'PAN/TILT BUS',
                     'esp32_debug': 'ESP32 debug', 'unknown': 'UNKNOWN'}[r['role']]
            serial_no = r['usb_serial'] or '-'
            print(f"{r['device']:14} {label:14} serial={serial_no:12} {r['how']}")
        print()
        print('Stable paths for cobra_flex.launch.py:')
        for role, arg in (('chassis', 'rover_port'),
                          ('pan_tilt', 'pan_tilt_port')):
            path = found[role]['by_id'] if role in found else '<NOT FOUND>'
            print(f'  {arg}:={path}')

    return 0 if 'chassis' in found and 'pan_tilt' in found else 1


if __name__ == '__main__':
    sys.exit(main())
