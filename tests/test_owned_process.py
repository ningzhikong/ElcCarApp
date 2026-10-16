import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools/autotune'))
from owned_process import run_logged


class OwnedProcessTests(unittest.TestCase):
    def test_output_survives_process_exit(self):
        with tempfile.TemporaryDirectory(dir=ROOT / '.build/tests') as directory:
            folder = Path(directory)
            prompt = folder / 'input.txt'
            prompt.write_text('test input', encoding='utf-8')
            code = run_logged([sys.executable, '-c', 'import sys; print(sys.stdin.read())'], prompt, folder / 'run', timeout=3)
            self.assertEqual(code, 0)
            self.assertIn('test input', (folder / 'run/stdout.jsonl').read_text())

    def test_timeout_kills_only_owned_tree(self):
        with tempfile.TemporaryDirectory(dir=ROOT / '.build/tests') as directory:
            folder = Path(directory)
            prompt = folder / 'input.txt'
            prompt.write_text('', encoding='utf-8')
            child = folder / 'child.py'
            child.write_text('import pathlib,sys,time\ntime.sleep(.8)\npathlib.Path(sys.argv[1]).write_text("alive")\n')
            owned_mark, unrelated_mark = folder / 'owned', folder / 'unrelated'
            unrelated = subprocess.Popen([sys.executable, str(child), str(unrelated_mark)])
            parent = folder / 'parent.py'
            parent.write_text('import subprocess,sys,time\nsubprocess.Popen(sys.argv[1:])\ntime.sleep(30)\n')
            try:
                with self.assertRaises(TimeoutError):
                    run_logged([sys.executable, str(parent), sys.executable, str(child), str(owned_mark)],
                               prompt, folder / 'run', timeout=.3)
                unrelated.wait(timeout=3)
                time.sleep(.8)
                self.assertTrue(unrelated_mark.exists())
                self.assertFalse(owned_mark.exists())
                self.assertTrue(json.loads((folder / 'run/process.json').read_text())['timed_out'])
            finally:
                if unrelated.poll() is None:
                    unrelated.terminate()
                    unrelated.wait()
