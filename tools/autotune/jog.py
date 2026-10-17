"""One bounded, single-wheel diagnostic pulse; never scores or tunes a motor."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import re
import time

import bench


def run(board, args):
    board.wait_stopped()
    board.send('HELLO')
    if board.read() != bench.HELLO:
        raise RuntimeError('Wrong firmware or boot mode')
    board.send('INFO')
    if board.read() != f'INFO ELC_INC_PI_V1 {args.build_id}':
        raise RuntimeError('Firmware build does not match the requested build')
    board.send('ENC')
    before = board.read()
    if len(before.split()) != 4 or not before.startswith('ENC '):
        raise RuntimeError('Missing cumulative encoder diagnostic')
    board.send('ARM 1 1')
    if board.read() != 'ARMED 1 1':
        raise RuntimeError('Arm was not confirmed')
    left, right = (args.pwm, 0) if args.side == 'left' else (0, args.pwm)
    board.send(f'JOG 1 {left} {right} {args.duration_ms}')
    if board.read() != f'JOGACK 1 {left} {right} {args.duration_ms}':
        raise RuntimeError('Jog was not acknowledged exactly')
    deadline = time.monotonic() + 2
    observations = []
    while time.monotonic() < deadline:
        board.send('PING')
        status = board.status()
        observations.append(status)
        if status[0] != 1 or status[2] != 0:
            raise RuntimeError(f'Jog fault or unexpected experiment: {status}')
        if status[1] == 2:
            if status[4] != args.duration_ms:
                raise RuntimeError(f'Unexpected jog duration: {status}')
            break
        if status[1] != 1:
            raise RuntimeError(f'Jog interrupted: {status}')
        time.sleep(.05)
    else:
        raise TimeoutError('Jog did not finish')
    board.send('ENC')
    after = board.read()
    return {'encoder_before': before, 'encoder_after': after, 'observations': observations}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--build-id', required=True)
    parser.add_argument('--side', choices=('left', 'right'), required=True)
    parser.add_argument('--pwm', type=int, default=100)
    parser.add_argument('--duration-ms', type=int, default=600)
    parser.add_argument('--output', type=Path, default=Path('.build/hardware_checks'))
    args = parser.parse_args()
    if not re.fullmatch('[0-9a-f]{16}', args.build_id):
        parser.error('build-id must be the expected 16-character firmware build ID')
    if not 1 <= abs(args.pwm) <= 150 or not 50 <= args.duration_ms <= 800 or args.duration_ms % 10:
        parser.error('PWM must be nonzero within +/-150; duration must be 50..800ms in 10ms steps')
    import serial
    folder = args.output / ('jog_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    folder.mkdir(parents=True)
    board = bench.Board.__new__(bench.Board)
    board.connection = serial.Serial(port=None, baudrate=115200, timeout=.1, write_timeout=.5)
    board.connection.dtr = board.connection.rts = False
    board.connection.port = args.port
    board.buffer = bytearray()
    board.audit = (folder / 'serial.jsonl').open('w', encoding='utf-8')
    result = {'source': 'diagnostic_jog', 'side': args.side, 'pwm': args.pwm,
              'duration_ms': args.duration_ms, 'build_id': args.build_id}
    try:
        board.connection.open()
        board.connection.reset_input_buffer()
        result.update(run(board, args))
        result['outcome'] = 'completed'
    except BaseException as exc:
        result.update(outcome='stopped_or_failed', error=str(exc))
    finally:
        if board.connection.is_open:
            try:
                board.wait_stopped()
                result['stopped_and_quiet'] = True
            except BaseException as exc:
                result['stop_error'] = str(exc)
        board.connection.close()
        board.audit.close()
        (folder / 'result.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps(result, indent=2))
    return 0 if result.get('outcome') == 'completed' and result.get('stopped_and_quiet') else 1


if __name__ == '__main__':
    raise SystemExit(main())
