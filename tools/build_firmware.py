"""Build the Keil project into .build without touching the board or IDE outputs."""
import argparse
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def build(keil):
    tools = keil / 'ARM/ARMCC/bin'
    output = ROOT / '.build/firmware'
    output.mkdir(parents=True, exist_ok=True)
    project = ROOT / 'MDK-ARM/ElcCarApp.uvprojx'
    target = ET.parse(project).find('./Targets/Target')
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
    for item in target.findall('./Groups/Group/Files/File'):
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
    build(parser.parse_args().keil)
