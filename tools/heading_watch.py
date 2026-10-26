"""Observe a locally started raised-wheel heading check; never requests motion."""
from datetime import datetime
import json
from pathlib import Path
import struct
import sys
import time

ROOT = Path(__file__).resolve().parents[1]


def main():
    sys.path.insert(0, str(ROOT / '.build/pyocd_deps'))
    from elftools.elf.elffile import ELFFile
    from intelhex import IntelHex
    from pyocd.core.helpers import ConnectHelper
    output = ROOT / '.build/ground_test'
    identity = json.loads((output / 'build_id.json').read_text())
    with (output / 'ElcCarApp.axf').open('rb') as stream:
        symbols = {s.name: s['st_value'] for s in ELFFile(stream).get_section_by_name('.symtab').iter_symbols()}
    folder = ROOT / '.build/hardware_checks' / datetime.now().strftime('heading_watch_%Y%m%d_%H%M%S_%f')
    folder.mkdir()
    opts = dict(target_override='stm32f103c8', pack=[r'D:\Keil\ARM\Pack\Keil\STM32F1xx_DFP\2.2.0'],
                connect_mode='attach', auto_unlock=False, resume_on_disconnect=False, frequency=1000000, no_config=True)
    result = dict(identity, motion_requested=False)
    rows = []
    try:
        session = ConnectHelper.session_with_chosen_probe(unique_id='37FF71064E573436189D1843', blocking=False, options=opts)
        if session is None:
            raise RuntimeError('ST-Link disconnected')
        with session:
            target = session.target
            expected = IntelHex(str(output / 'ElcCarApp.hex'))
            first, last = min(expected.addresses()), max(expected.addresses())
            image = target.read_memory_block8(first, last-first+1)
            if not all(image[a-first] == expected[a] for a in expected.addresses()):
                raise RuntimeError('Live image mismatch')
            deadline = time.monotonic() + 50
            while time.monotonic() < deadline:
                for _ in range(6):
                    tick = target.read32(symbols['current'])
                    view = struct.unpack('<5I4i6f', bytes(target.read_memory_block8(symbols['current'], 60)))
                    if tick == view[0] == target.read32(symbols['current']):
                        break
                else:
                    raise RuntimeError('Unable to obtain coherent control snapshot')
                status = struct.unpack('<6I', bytes(target.read_memory_block8(symbols['ground_result'], 24)))
                heading = struct.unpack('<3I4f', bytes(target.read_memory_block8(symbols['ground_heading'], 28)))
                if status[0] != 0x47524434:
                    raise RuntimeError('Invalid ground state')
                row = dict(tick_ms=view[0], state=status[1], reason=status[2], elapsed_ms=status[4],
                    yaw_error_deg=(heading[3]-heading[6]+180)%360-180, rate_dps=heading[4],
                    target_rpm=[view[11]*6000/1080, view[12]*6000/1320],
                    pwm=[view[13], view[14]], imu_error=heading[2])
                rows.append(row)
                if status[1] >= 4:
                    result['final_state'] = status[1]
                    result['final_reason'] = status[2]
                    result['final_pwm'] = row['pwm']
                    break
                time.sleep(.1)
            running = [r for r in rows if r['state'] == 2]
            held_left = [r for r in running if r['yaw_error_deg'] >= 1 and abs(r['rate_dps']) < 2]
            correct = [r for r in held_left if r['target_rpm'][0] > r['target_rpm'][1] + .05]
            result.update(running_samples=len(running), held_left_samples=len(held_left),
                          corrective_target_samples=len(correct),
                          left_error_corrective_target_seen=bool(correct),
                          outcome='completed' if result.get('final_state') == 4 and result.get('final_reason') == 0 else 'incomplete_or_fault')
    except BaseException as exc:
        result.update(outcome='failed', error=str(exc))
    finally:
        (folder / 'samples.json').write_text(json.dumps(rows, indent=2), encoding='utf-8')
        (folder / 'result.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps(dict(result, folder=str(folder)), indent=2))
    return 0 if result.get('outcome') == 'completed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
