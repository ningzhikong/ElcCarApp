"""Verify archived handover data; optionally restore it. Does not access hardware."""
import argparse
import hashlib
import io
import json
from pathlib import Path, PurePosixPath
import zipfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--extract', action='store_true')
    parser.add_argument('--overwrite', action='store_true')
    args = parser.parse_args()
    archive_dir = Path(__file__).resolve().parent
    root = archive_dir.parent
    manifest = json.loads((archive_dir / 'manifest.json').read_text(encoding='utf-8'))
    count = 0
    for archive in manifest['archives']:
        data = bytearray()
        for part in archive['parts']:
            name = part['name']
            assert Path(name).name == name, 'Invalid archive part path'
            chunk = (archive_dir / name).read_bytes()
            assert len(chunk) == part['bytes'], name + ': length mismatch'
            assert hashlib.sha256(chunk).hexdigest() == part['sha256'], name + ': SHA256 mismatch'
            data.extend(chunk)
        assert hashlib.sha256(data).hexdigest() == archive['sha256'], archive['name'] + ': SHA256 mismatch'
        entries = {entry['path']: entry for entry in manifest['files'] if entry['archive'] == archive['name']}
        with zipfile.ZipFile(io.BytesIO(data)) as bundle:
            assert len(bundle.namelist()) == len(entries) == archive['files'], 'Archive entry count mismatch'
            assert set(bundle.namelist()) == set(entries), 'Archive entry list mismatch'
            for name, entry in entries.items():
                path = PurePosixPath(name)
                assert not path.is_absolute() and '..' not in path.parts and '\\' not in name, 'Unsafe archive path'
                target = (root / name).resolve()
                assert target.is_relative_to(root), 'Archive path escapes workspace'
                content = bundle.read(name)
                assert len(content) == entry['bytes'], name + ': length mismatch'
                assert hashlib.sha256(content).hexdigest() == entry['sha256'], name + ': SHA256 mismatch'
                if args.extract:
                    if target.exists() and target.read_bytes() != content and not args.overwrite:
                        raise RuntimeError('Refusing to overwrite: ' + name)
                    if not target.exists() or target.read_bytes() != content:
                        target.parent.mkdir(parents=True, exist_ok=True)
                        target.write_bytes(content)
                count += 1
        print('Verified:', archive['name'], archive['files'], 'files')
    print(('Restored' if args.extract else 'Verified'), count, 'files; no hardware operations performed.')


if __name__ == '__main__':
    main()
