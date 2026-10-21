"""Flash and verify the motor-disabled IR diagnostic via the known ST-Link."""
import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--verify-only', action='store_true',
                    help='Verify and start the already flashed IR diagnostic without programming')
args = parser.parse_args()
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / '.build/pyocd_deps'))
from pyocd.core.helpers import ConnectHelper
from intelhex import IntelHex
from elftools.elf.elffile import ELFFile

out = ROOT / '.build/ir_test'
identity = json.loads((out / 'build_id.json').read_text())
if identity.get('mode') != 'ir_test':
    raise ValueError('Rebuild with --ir-test before using this tool')
opts = {'target_override': 'stm32f103c8',
        'pack': [r'D:\Keil\ARM\Pack\Keil\STM32F1xx_DFP\2.2.0'],
        'connect_mode': 'attach', 'auto_unlock': False,
        'resume_on_disconnect': False, 'frequency': 1000000, 'no_config': True}
with (out / 'ElcCarApp.axf').open('rb') as f:
    symbols = {s.name: s['st_value'] for s in ELFFile(f).get_section_by_name('.symtab').iter_symbols()}
required = ('ir_test_raw', 'ir_test_levels', 'ir_test_frames', 'ir_test_select_errors')
if not all(n in symbols for n in required):
    raise ValueError('Not an IR diagnostic ELF')
h = IntelHex(str(out / 'ElcCarApp.hex'))
if not h.addresses() or not all(0x08000000 <= a < 0x08010000 for a in h.addresses()):
    raise ValueError('Image exceeds Flash bounds')
if not args.verify_only:
    # Stop and back up in a separate session; release the probe before CLI load.
    with ConnectHelper.session_with_chosen_probe(unique_id='37FF71064E573436189D1843', blocking=False, options=opts) as before:
        t = before.target
        t.halt()
        t.write32(0x4000043c, 0); t.write32(0x40000440, 0)
        if t.read32(0x4000043c) or t.read32(0x40000440):
            raise RuntimeError('PWM did not stop')
        backup = out / 'before_flash.bin'
        if not backup.exists():
            backup.write_bytes(bytes(t.read_memory_block8(0x08000000, 0x10000)))
    subprocess.run([sys.executable, str(ROOT / '.build/pyocd_local.py'), 'load', '--no-config',
        '--pack', r'D:\Keil\ARM\Pack\Keil\STM32F1xx_DFP\2.2.0', '-t', 'stm32f103c8',
        '-u', '37FF71064E573436189D1843', '-f', '1000000', '-M', 'halt',
        '-O', 'auto_unlock=false', '-O', 'resume_on_disconnect=false',
        '--no-reset', '-e', 'sector', str(out / 'ElcCarApp.hex')], check=True, cwd=ROOT)
with ConnectHelper.session_with_chosen_probe(unique_id='37FF71064E573436189D1843', blocking=False, options=opts) as session:
    t = session.target
    t.halt()
    try:
        t.write32(0x4000043c, 0)
        t.write32(0x40000440, 0)
        h = IntelHex(str(out / 'ElcCarApp.hex'))
        addresses = h.addresses()
        actual = t.read_memory_block8(min(addresses), max(addresses) - min(addresses) + 1)
        assert all(actual[a-min(addresses)] == h[a] for a in addresses), 'Flash mismatch'
        t.reset_and_halt()
        t.resume()
        time.sleep(1)
        samples = []
        for _ in range(2):
            samples.append({'raw': list(t.read_memory_block8(symbols['ir_test_raw'], 8)), 'levels': list(t.read_memory_block8(symbols['ir_test_levels'], 8)),
                            'frames': t.read32(symbols['ir_test_frames']),
                            'changed': t.read8(symbols['ir_test_changed']),
                            'select_errors': t.read32(symbols['ir_test_select_errors']),
                            'pwm': [t.read32(0x4000043c), t.read32(0x40000440)]})
            time.sleep(1.0)
        assert samples[1]['frames'] > samples[0]['frames'], 'Test not scanning'
        assert all(s['pwm'] == [0, 0] and s['select_errors'] == 0 for s in samples)
        result = {**identity, 'verified_bytes': len(addresses), 'samples': samples, 'state': t.get_state().name}
        (out / 'verification.json').write_text(json.dumps(result, indent=2))
        print(json.dumps(result))
    except BaseException:
        t.halt()
        t.write32(0x4000043c, 0)
        t.write32(0x40000440, 0)
        raise
