"""Flash the one-shot ground test, or read its RAM trace without resetting it."""
import argparse
import csv
from datetime import datetime
import hashlib
import json
from pathlib import Path
import struct
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / '.build/ground_test'
UID = '37FF71064E573436189D1843'
PACK = r'D:\Keil\ARM\Pack\Keil\STM32F1xx_DFP\2.2.0'
STATES = ['ready', 'countdown', 'running', 'settling', 'done', 'stopped', 'fault']


def decode_trace(raw, *, line=False):
    if len(raw) % 60:
        raise ValueError('Truncated ground trace')
    return list(struct.iter_unpack('<Iii4f4If2If' if line else '<Iii4f2IfIf2If', raw))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--flash', action='store_true', help='Back up, program, verify, reset to READY; never start motion')
    parser.add_argument('--line', action='store_true', help='Select ground-line image and IR trace')
    args = parser.parse_args()
    global OUT
    if args.line:
        OUT = ROOT / '.build/ground_line'
    sys.path.insert(0, str(ROOT / '.build/pyocd_deps'))
    from pyocd.core.helpers import ConnectHelper
    from elftools.elf.elffile import ELFFile
    from intelhex import IntelHex
    identity = json.loads((OUT / 'build_id.json').read_text())
    if identity.get('mode') != ('ground_line' if args.line else 'ground_test'):
        raise ValueError('Build the selected ground mode first')
    with (OUT / 'ElcCarApp.axf').open('rb') as stream:
        symbols = {s.name: (s['st_value'], s['st_size']) for s in ELFFile(stream).get_section_by_name('.symtab').iter_symbols()}
    required = {'ground_result', 'ground_trace', 'ground_target_rpm', 'ground_run_ms', 'ground_sample_ms',
                'ground_pwm_limit', 'ground_ramp_ms', 'ground_start_rpm',
                'current', 'init_pending', 'completed_frames', 'error_count'}
    if args.line:
        required.update(('ground_ir', 'ground_ir_direction', 'ground_max_rpm'))
    else:
        required.update(('ground_motion_stats', 'ground_heading', 'ground_heading_axis', 'ground_heading_sign',
                         'ground_imu_init_result', 'ground_imu_calibration_result', 'mpu_calibration_report',
                         'heading_kp', 'heading_kd', 'heading_max_turn_rpm', 'heading_slew_rpm_s', 'heading_max_error_deg'))
    missing = sorted(required.difference(symbols))
    if missing:
        raise RuntimeError('ELF lacks required readback symbols; refusing to flash: ' + ', '.join(missing))
    assert symbols['ground_result'][1] == 24 and symbols['ground_trace'][1] == 121 * 60
    h = IntelHex(str(OUT / 'ElcCarApp.hex'))
    addresses = h.addresses()
    assert addresses and all(0x08000000 <= a < 0x08010000 for a in addresses)
    opts = {'target_override': 'stm32f103c8', 'pack': [PACK], 'connect_mode': 'attach',
            'auto_unlock': False, 'resume_on_disconnect': False, 'frequency': 1000000, 'no_config': True}
    def connect():
        session = ConnectHelper.session_with_chosen_probe(unique_id=UID, blocking=False, options=opts)
        if session is None:
            raise RuntimeError('ST-Link is not available; check USB connection and device descriptors')
        return session
    def verify(t):
        first = min(addresses)
        actual = bytes(t.read_memory_block8(first, max(addresses) - first + 1))
        if not all(actual[a-first] == h[a] for a in addresses):
            raise RuntimeError('Live firmware differs from ground-test ELF; refusing symbol access')
        return actual
    folder = ROOT / '.build/hardware_checks' / ('ground_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    folder.mkdir(parents=True)
    if args.flash:
        with connect() as session:
            t = session.target
            t.halt()
            t.write32(0x4000043c, 0); t.write32(0x40000440, 0)
            assert t.read32(0x4000043c) == t.read32(0x40000440) == 0
            (folder / 'before_flash.bin').write_bytes(bytes(t.read_memory_block8(0x08000000, 0x10000)))
        subprocess.run([sys.executable, str(ROOT / 'tools/pyocd_local.py'), 'load', '--no-config',
                        '--pack', PACK, '-t', 'stm32f103c8', '-u', UID, '-f', '1000000', '-M', 'halt',
                        '-O', 'auto_unlock=false', '-O', 'resume_on_disconnect=false', '--no-reset',
                        '-e', 'sector', str(OUT / 'ElcCarApp.hex')], check=True, cwd=ROOT)
    with connect() as session:
        t = session.target
        actual = verify(t)
        result = {**identity, 'verified_bytes': len(addresses), 'sha256': hashlib.sha256(actual).hexdigest()}
        if args.flash:
            (folder / 'readback.bin').write_bytes(actual)
            t.reset_and_halt()
            t.resume()
            deadline = time.monotonic() + 6
            while True:
                if t.read32(0x4000043c) or t.read32(0x40000440):
                    t.halt(); t.write32(0x4000043c, 0); t.write32(0x40000440, 0)
                    raise RuntimeError('Unexpected motion while initializing READY')
                if t.read8(symbols['init_pending'][0]) == 0 and t.read32(symbols['completed_frames'][0]) > 0:
                    break
                if time.monotonic() >= deadline:
                    raise TimeoutError('Ground initialization/display did not complete')
                time.sleep(.05)
        raw = bytes(t.read_memory_block8(symbols['ground_result'][0], 24))
        magic, state, reason, count, elapsed, quiet = struct.unpack('<6I', raw)
        if magic != 0x47524434 or state >= len(STATES) or count > 121:
            raise RuntimeError('Invalid ground result')
        if args.flash and (state != 0 or count != 0 or t.read32(0x4000043c) or t.read32(0x40000440)):
            t.halt(); t.write32(0x4000043c, 0); t.write32(0x40000440, 0)
            raise RuntimeError('Expected stopped READY after flashing')
        if state in (1, 2, 3):
            raise RuntimeError('Test is active; wait for automatic stop before reading')
        trace = bytes(t.read_memory_block8(symbols['ground_trace'][0], count * 60)) if count else b''
        rows = decode_trace(trace, line=args.line)
        result.update(state=STATES[state], reason=reason, count=count, elapsed_ms=elapsed, quiet_ticks=quiet,
                      cpu=t.get_state().name, pwm=[t.read32(0x4000043c), t.read32(0x40000440)],
                      data_retention='RAM only; reconnect without power loss or reset',
                      target_rpm=struct.unpack('<f', bytes(t.read_memory_block8(symbols['ground_target_rpm'][0], 4)))[0],
                      run_ms=t.read32(symbols['ground_run_ms'][0]), sample_ms=t.read32(symbols['ground_sample_ms'][0]),
                      countdown_ms=3000, pwm_limit=t.read32(symbols['ground_pwm_limit'][0]),
                      ramp_ms=t.read32(symbols['ground_ramp_ms'][0]),
                      start_rpm=struct.unpack('<f', bytes(t.read_memory_block8(symbols['ground_start_rpm'][0], 4)))[0],
                      cpr_reference=[1080, 1320], test_executed=bool(count))
        if args.line:
            levels, black_count, ir_tick, valid, error, raw_levels, noisy = struct.unpack('<4If2I', bytes(t.read_memory_block8(symbols['ground_ir'][0], 28)))
            direction = struct.unpack('<i', bytes(t.read_memory_block8(symbols['ground_ir_direction'][0], 4)))[0]
            result.update(ir={ 'levels': ''.join('1' if levels & (1 << i) else '0' for i in range(8)),
                               'black_count': black_count, 'tick_ms': ir_tick, 'valid': valid, 'error': error,
                               'raw_levels': ''.join('1' if raw_levels & (1 << i) else '0' for i in range(8)), 'noisy': noisy },
                          ir_direction=direction, motion_permitted=direction in (-1, 1), steering_limit_rpm=4,
                          controller='shared filtered PD with bounded variable RPM; parameters in route_config.h',
                          max_base_rpm=struct.unpack('<f', bytes(t.read_memory_block8(symbols['ground_max_rpm'][0], 4)))[0],
                          ir_filter='3 consecutive frames per channel; 4 flips in 200ms => noisy; clear after 200ms stable',
                          line_loss_policy='raw all-white/all-black, filtered loss, noisy or stale => immediate stop; latch reason 104')
        control = bytes(t.read_memory_block8(symbols['current'][0], 60))
        result['control_fault'] = struct.unpack('<5I4i6f', control)[3]
        if not args.line:
            assert symbols['ground_heading'][1] == 28
            tick, ready, error, angle, rate, bias, reference = struct.unpack('<3I4f', bytes(t.read_memory_block8(symbols['ground_heading'][0], 28)))
            read_float = lambda name: struct.unpack('<f', bytes(t.read_memory_block8(symbols[name][0], 4)))[0]
            heading_error = (angle - reference + 180) % 360 - 180
            result['heading'] = dict(enabled=True, axis=t.read32(symbols['ground_heading_axis'][0]),
                sign=read_float('ground_heading_sign'), tick_ms=tick, ready=ready, error=error,
                angle_deg=angle, reference_deg=reference, yaw_error_deg=heading_error,
                rate_dps=rate, bias_dps=bias, kp=read_float('heading_kp'), kd=read_float('heading_kd'),
                turn_limit_rpm=read_float('heading_max_turn_rpm'), slew_rpm_s=read_float('heading_slew_rpm_s'),
                fault_error_deg=read_float('heading_max_error_deg'))
            report = struct.unpack('<6I8f', bytes(t.read_memory_block8(symbols['mpu_calibration_report'][0], 56)))
            result['imu_startup'] = dict(init_result=t.read32(symbols['ground_imu_init_result'][0]),
                calibration_result=t.read32(symbols['ground_imu_calibration_result'][0]),
                requested=report[0], samples=report[1], io_failures=report[2], gravity_failures=report[3],
                rate_failures=report[4], flags=report[5], gyro_min=report[6:9], gyro_max=report[9:12],
                gravity_squared_range=report[12:14])
        result['oled'] = {name: int.from_bytes(bytes(t.read_memory_block8(symbols[name][0], symbols[name][1])), 'little')
                          for name in ('init_pending', 'completed_frames', 'error_count')}
        if args.flash:
            assert result['cpu'] == 'RUNNING' and result['control_fault'] == 0
            assert result['oled']['init_pending'] == 0 and result['oled']['completed_frames'] > 0
        with (folder / 'trace.csv').open('w', newline='', encoding='utf-8') as stream:
            writer = csv.writer(stream)
            writer.writerow(['elapsed_ms', 'raw_left', 'raw_right', 'rpm_left', 'rpm_right',
                             'pwm_left', 'pwm_right', 'period_us', 'fault',
                             'ir_levels_mask' if args.line else 'yaw_error_deg',
                             'ir_age_ms' if args.line else 'imu_age_ms',
                             'turn_rpm', 'ir_raw_levels_mask', 'ir_noisy', 'target_base_rpm'])
            writer.writerows(rows)
        if rows:
            plateau = [r for r in rows if 2000 <= r[0] < result['run_ms']]
            if plateau:
                result['plateau'] = {'mean_rpm_left': sum(r[3] for r in plateau) / len(plateau),
                                     'mean_rpm_right': sum(r[4] for r in plateau) / len(plateau),
                                     'mean_absolute_wheel_difference_rpm': sum(abs(r[3]-r[4]) for r in plateau) / len(plateau)}
                result['target_mean_absolute_error_rpm'] = [sum(abs(r[3] - (r[14] - r[11])) for r in plateau) / len(plateau),
                                                            sum(abs(r[4] - (r[14] + r[11])) for r in plateau) / len(plateau)]
            result['peak_pwm'] = max(max(r[5], r[6]) for r in rows)
            result['period_us_range'] = [min(r[7] for r in rows), max(r[7] for r in rows)]
        if not args.line:
            assert symbols['ground_motion_stats'][1] == 32
            stats = struct.unpack('<Iii5I', bytes(t.read_memory_block8(symbols['ground_motion_stats'][0], 32)))
            ticks, counts_l, counts_r, period, pwm_l, pwm_r, cap_l, cap_r = stats
            result['full_tick_stats'] = dict(ticks=ticks, counts=[counts_l, counts_r], period_us=period,
                                           pwm_sums=[pwm_l, pwm_r], capped_ticks=[cap_l, cap_r])
            if ticks and period:
                if (not 5000 * ticks <= period <= 15000 * ticks or max(cap_l, cap_r) > ticks
                        or max(pwm_l, pwm_r) > result['pwm_limit'] * ticks):
                    raise RuntimeError('Invalid full-tick ground summary')
                result['full_tick_stats'].update(
                    mean_rpm=[counts_l * 60000000 / (1080 * period), counts_r * 60000000 / (1320 * period)],
                    mean_pwm=[pwm_l / ticks, pwm_r / ticks], cap_fraction=[cap_l / ticks, cap_r / ticks],
                    basis='every control tick after ramp + 1s, before output stop; not 100ms subsamples')
    (folder / 'result.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps({**result, 'folder': str(folder)}, indent=2))


if __name__ == '__main__':
    main()
