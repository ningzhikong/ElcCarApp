"""Build the Keil project into .build without touching the board or IDE outputs."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def build(keil, ir_test=False, ground_test=False, ground_line=False, imu_test=False):
    tools = keil / 'ARM/ARMCC/bin'
    if ir_test and (ground_test or ground_line):
        raise ValueError('Select only one diagnostic mode')
    if imu_test and (ir_test or ground_test or ground_line):
        raise ValueError('Select only one diagnostic mode')
    ground_test = ground_test or ground_line
    mode = 'imu_test' if imu_test else 'ground_line' if ground_line else 'ground_test' if ground_test else 'ir_test' if ir_test else 'normal'
    output = ROOT / '.build' / ('firmware' if mode == 'normal' else mode)
    output.mkdir(parents=True, exist_ok=True)
    project = ROOT / 'MDK-ARM/ElcCarApp.uvprojx'
    target = ET.parse(project).find('./Targets/Target')
    digest = hashlib.sha256(b'armcc5-cortexm3-microlib-O1-v1')
    digest.update(mode.encode('ascii'))
    compiler_version = subprocess.check_output([tools / 'armcc.exe', '--vsn'])
    digest.update(compiler_version.replace(b'\r\n', b'\n'))
    inputs = sorted([*ROOT.glob('Core/**/*.c'), *ROOT.glob('Core/**/*.h'),
                     *ROOT.glob('Drivers/**/*.c'), *ROOT.glob('Drivers/**/*.h'),
                     ROOT / 'ElcCarApp.ioc', project,
                     ROOT / 'MDK-ARM/startup_stm32f103xb.s', Path(__file__).resolve()])
    for item in inputs:
        digest.update(item.relative_to(ROOT).as_posix().encode('utf-8') + b'\0')
        digest.update(item.read_bytes().replace(b'\r\n', b'\n') + b'\0')
    build_id = digest.hexdigest()[:16]
    (output / 'build_id.json').write_text(json.dumps({
        'build_id': build_id, 'mode': mode}), encoding='utf-8')
    includes = [ROOT / p for p in (
        'Core/Inc', 'Drivers/STM32F1xx_HAL_Driver/Inc',
        'Drivers/STM32F1xx_HAL_Driver/Inc/Legacy',
        'Drivers/CMSIS/Device/ST/STM32F1xx/Include', 'Drivers/CMSIS/Include')]
    commands, objects = [], []
    log = output / 'build.log'

    def run(args):
        commands.append([str(a) for a in args])
        result = subprocess.run(args, cwd=ROOT, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True, errors='replace')
        with log.open('a', encoding='utf-8') as stream:
            stream.write(subprocess.list2cmdline([str(a) for a in args]) + '\n' + result.stdout)
        if result.stdout:
            print(result.stdout, end='')
        if result.returncode:
            raise RuntimeError(f'Build failed ({result.returncode}); see {log}')

    log.write_text('', encoding='utf-8')
    files = target.findall('./Groups/Group/Files/File')
    if ground_test:
        item = ET.Element('File')
        ET.SubElement(item, 'FileType').text = '1'
        ET.SubElement(item, 'FilePath').text = '../Core/Src/ground_test.c'
        files.append(item)
        if not ground_line:
            item = ET.Element('File')
            ET.SubElement(item, 'FileType').text = '1'
            ET.SubElement(item, 'FilePath').text = '../Core/Src/heading_control.c'
            files.append(item)
    if imu_test:
        item = ET.Element('File')
        ET.SubElement(item, 'FileType').text = '1'
        ET.SubElement(item, 'FilePath').text = '../Core/Src/imu_diagnostic.c'
        files.append(item)
    for item in files:
        kind = item.findtext('FileType')
        if kind not in ('1', '2'):
            continue
        source = (project.parent / item.findtext('FilePath').replace('\\', '/')).resolve()
        obj = output / (source.stem + '.o')
        objects.append(obj)
        if kind == '1':
            run([tools / 'armcc.exe', '--cpu', 'Cortex-M3', '--c99', '-O1', '-g',
                 '--split_sections', '--library_type=microlib', '-D__MICROLIB',
                 '-DSTM32F103xB', '-DUSE_HAL_DRIVER',
                 *(['-DCAR_IR_TEST'] if ir_test else []),
                 *(['-DCAR_IMU_TEST'] if imu_test else []),
                 *(['-DCAR_GROUND_TEST'] if ground_test else []),
                 *(['-DCAR_GROUND_LINE'] if ground_line else []),
                 f'-DCAR_BUILD_ID="{build_id}"',
                 *[f'-I{p}' for p in includes], '-c', source, '-o', obj])
        else:
            run([tools / 'armasm.exe', '--cpu', 'Cortex-M3', '--pd', '__MICROLIB SETA 1',
                 '-g', source, '-o', obj])
    scatter = output / 'firmware.sct'
    scatter.write_text('''LR_IROM1 0x08000000 0x10000 {
 ER_IROM1 0x08000000 0x10000 {
  *.o (RESET, +First)
  *(InRoot$$Sections)
  .ANY (+RO)
 }
 RW_IRAM1 0x20000000 0x5000 { .ANY (+RW +ZI) }
}
''', encoding='ascii')
    image = output / 'ElcCarApp.axf'
    run([tools / 'armlink.exe', '--cpu', 'Cortex-M3', '--library_type=microlib',
         '--scatter', scatter, '--map', '--info=sizes,stack', '--callgraph',
         '--list', output / 'firmware.map', '-o', image, *objects])
    run([tools / 'fromelf.exe', '--i32combined', '--output', output / 'ElcCarApp.hex', image])
    print(f'Built {image}; no flash/download command was run.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--keil', type=Path, default=Path(r'D:\Keil'))
    parser.add_argument('--ir-test', action='store_true', help='Build motor-disabled infrared diagnostic')
    parser.add_argument('--imu-test', action='store_true', help='Build motor-disabled IMU axis/sign diagnostic')
    parser.add_argument('--ground-test', action='store_true', help='Build standalone local-button ground test')
    parser.add_argument('--ground-line', action='store_true', help='Build bounded 10 RPM line-following test')
    args = parser.parse_args()
    build(args.keil, args.ir_test, args.ground_test, args.ground_line, args.imu_test)
