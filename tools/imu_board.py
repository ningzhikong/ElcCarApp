"""Flash/read the motor-disabled IMU axis diagnostic, with coherent SWD snapshots."""
import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / '.build/imu_test'
UID = '37FF71064E573436189D1843'
PACK = r'D:\Keil\ARM\Pack\Keil\STM32F1xx_DFP\2.2.0'


def decode(raw):
    if len(raw) != 76:
        raise ValueError('Wrong IMU diagnostic size')
    magic, sequence, state, error, tick, samples, period, *values = struct.unpack('<7I12f', raw)
    if magic != 0x494d5531 or sequence % 2 or state not in (1, 2, 3):
        raise ValueError('Invalid/in-progress IMU diagnostic')
    if not all(math.isfinite(v) for v in values) or (samples and not 20 <= period <= 50):
        raise ValueError('Invalid IMU numeric/timing data')
    return dict(sequence=sequence, state=state, error=error, tick_ms=tick, samples=samples,
                period_ms=period, gyro_dps=values[:3], angle_deg=values[3:6],
                bias_dps=values[6:9], accel_g=values[9:12])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--flash', action='store_true')
    parser.add_argument('--seconds', type=float, default=2.)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not .5 <= args.seconds <= 30:
        parser.error('Capture duration must be 0.5..30 seconds')
    sys.path.insert(0, str(ROOT / '.build/pyocd_deps'))
    from pyocd.core.helpers import ConnectHelper
    from elftools.elf.elffile import ELFFile
    from intelhex import IntelHex
    identity = json.loads((OUT / 'build_id.json').read_text())
    if identity['mode'] != 'imu_test':
        raise ValueError('Build --imu-test first')
    with (OUT / 'ElcCarApp.axf').open('rb') as stream:
        symbols = {s.name: (s['st_value'], s['st_size']) for s in ELFFile(stream).get_section_by_name('.symtab').iter_symbols()}
    address, size = symbols['imu_diagnostic']
    assert size == 76 and 0x20000000 <= address <= 0x20005000 - size
    expected = IntelHex(str(OUT / 'ElcCarApp.hex'))
    addresses = expected.addresses()
    assert addresses and all(0x08000000 <= a < 0x08010000 for a in addresses)
    opts = dict(target_override='stm32f103c8', pack=[PACK], connect_mode='attach',
                auto_unlock=False, resume_on_disconnect=False, frequency=1000000, no_config=True)
    def connect():
        session = ConnectHelper.session_with_chosen_probe(unique_id=UID, blocking=False, options=opts)
        if session is None:
            raise RuntimeError('ST-Link is not connected')
        return session
    folder = ROOT / '.build/hardware_checks' / datetime.now().strftime('imu_%Y%m%d_%H%M%S_%f')
    folder.mkdir()
    for name in ('ElcCarApp.axf', 'ElcCarApp.hex', 'build_id.json'):
        shutil.copy2(OUT / name, folder / name)
    result = dict(identity, motion_permitted=False, requested_capture_s=args.seconds, flashed=args.flash)
    snapshots = []
    try:
        if args.flash:
            with connect() as session:
                target = session.target
                target.halt()
                target.write32(0x4000043c, 0); target.write32(0x40000440, 0)
                assert target.read32(0x4000043c) == target.read32(0x40000440) == 0
                (folder / 'before_flash.bin').write_bytes(bytes(target.read_memory_block8(0x08000000, 0x10000)))
                (folder / 'before_ram.bin').write_bytes(bytes(target.read_memory_block8(0x20000000, 0x5000)))
            subprocess.run([sys.executable, str(ROOT / 'tools/pyocd_local.py'), 'load', '--no-config',
                            '--pack', PACK, '-t', 'stm32f103c8', '-u', UID, '-f', '1000000', '-M', 'halt',
                            '-O', 'auto_unlock=false', '-O', 'resume_on_disconnect=false', '--no-reset',
                            '-e', 'sector', str(OUT / 'ElcCarApp.hex')], check=True, cwd=ROOT)
        with connect() as session:
            target = session.target
            first, last = min(addresses), max(addresses)
            actual = bytes(target.read_memory_block8(first, last - first + 1))
            if not all(actual[a - first] == expected[a] for a in addresses):
                raise RuntimeError('Live flash differs from motor-disabled IMU image')
            result['verified_bytes'] = len(addresses)
            if args.flash:
                (folder / 'readback.bin').write_bytes(actual)
                target.reset_and_halt(); target.resume()
            def snapshot():
                if target.read32(0x4000043c) or target.read32(0x40000440):
                    target.halt(); target.write32(0x4000043c, 0); target.write32(0x40000440, 0)
                    raise RuntimeError('Nonzero PWM in locked diagnostic; CPU halted')
                for _ in range(8):
                    seq = target.read32(address + 4)
                    raw = bytes(target.read_memory_block8(address, size))
                    end = target.read32(address + 4)
                    if seq == end and not seq % 2 and struct.unpack_from('<I', raw, 4)[0] == seq:
                        return decode(raw)
                raise RuntimeError('Unable to obtain coherent IMU snapshot')
            deadline = time.monotonic() + 5
            while True:
                try:
                    initial = snapshot()
                except ValueError:
                    initial = None
                if initial and initial['state'] in (2, 3):
                    break
                if time.monotonic() >= deadline:
                    raise TimeoutError('IMU calibration did not complete')
                time.sleep(.05)
            result['initial'] = initial
            if initial['state'] != 2:
                raise RuntimeError(f'IMU initialization/calibration failed: {initial}')
            deadline = time.monotonic() + args.seconds
            while time.monotonic() < deadline:
                sample = snapshot(); snapshots.append(sample)
                if sample['state'] != 2:
                    raise RuntimeError(f'IMU sampling fault: {sample}')
                time.sleep(.05)
            result.update(final=snapshot(), pwm=[target.read32(0x4000043c), target.read32(0x40000440)],
                          control_timer_enabled=bool(target.read32(0x40012c00) & 1),
                          cpu=target.get_state().name, outcome='completed')
            if result['control_timer_enabled'] or result['pwm'] != [0, 0]:
                target.halt(); target.write32(0x4000043c, 0); target.write32(0x40000440, 0)
                raise RuntimeError('Motor lock was not confirmed; CPU halted')
            result['delta_angle_deg'] = [b - a for a, b in zip(initial['angle_deg'], result['final']['angle_deg'])]
    except BaseException as exc:
        result.update(outcome='failed', error=str(exc))
    finally:
        (folder / 'samples.json').write_text(json.dumps(snapshots, indent=2), encoding='utf-8')
        (folder / 'result.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps(dict(result, folder=str(folder)), indent=2))
    return 0 if result.get('outcome') == 'completed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
