"""Run one owned process tree with on-disk output and bounded lifetime."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time


def _windows_run(command, cwd, streams, timeout, state):
    # Create suspended, assign to a private kill-on-close Job, then resume.
    # This prevents a .exe from spawning unowned descendants before assignment.
    import _winapi
    import ctypes as c
    from ctypes import wintypes as w
    import msvcrt

    class Basic(c.Structure):
        _fields_ = [('process_time', c.c_longlong), ('job_time', c.c_longlong),
                    ('flags', w.DWORD), ('min_ws', c.c_size_t), ('max_ws', c.c_size_t),
                    ('active', w.DWORD), ('affinity', c.c_size_t),
                    ('priority', w.DWORD), ('scheduling', w.DWORD)]

    class IO(c.Structure):
        _fields_ = [(name, c.c_ulonglong) for name in ('ro', 'wo', 'oo', 'rt', 'wt', 'ot')]

    class Extended(c.Structure):
        _fields_ = [('basic', Basic), ('io', IO), ('process_mem', c.c_size_t),
                    ('job_mem', c.c_size_t), ('peak_process', c.c_size_t), ('peak_job', c.c_size_t)]

    kernel = c.WinDLL('kernel32', use_last_error=True)
    kernel.CreateJobObjectW.argtypes = [w.LPVOID, w.LPCWSTR]
    kernel.CreateJobObjectW.restype = w.HANDLE
    kernel.SetInformationJobObject.argtypes = [w.HANDLE, c.c_int, w.LPVOID, w.DWORD]
    kernel.SetInformationJobObject.restype = w.BOOL
    kernel.AssignProcessToJobObject.argtypes = [w.HANDLE, w.HANDLE]
    kernel.AssignProcessToJobObject.restype = w.BOOL
    kernel.TerminateJobObject.argtypes = [w.HANDLE, w.UINT]
    kernel.TerminateJobObject.restype = w.BOOL
    kernel.ResumeThread.argtypes = [w.HANDLE]
    kernel.ResumeThread.restype = w.DWORD
    kernel.CloseHandle.argtypes = [w.HANDLE]
    kernel.CloseHandle.restype = w.BOOL

    job = kernel.CreateJobObjectW(None, None)
    if not job:
        raise c.WinError(c.get_last_error())
    process = thread = None
    try:
        limits = Extended()
        limits.basic.flags = 0x2000  # JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE
        if not kernel.SetInformationJobObject(job, 9, c.byref(limits), c.sizeof(limits)):
            raise c.WinError(c.get_last_error())
        info = subprocess.STARTUPINFO()
        info.dwFlags = subprocess.STARTF_USESTDHANDLES | subprocess.STARTF_USESHOWWINDOW
        info.wShowWindow = subprocess.SW_HIDE
        handles = [msvcrt.get_osfhandle(stream.fileno()) for stream in streams]
        info.hStdInput, info.hStdOutput, info.hStdError = handles
        info.lpAttributeList = {'handle_list': handles}
        previous = [os.get_handle_inheritable(handle) for handle in handles]
        try:
            for handle in handles:
                os.set_handle_inheritable(handle, True)
            process, thread, pid, _ = _winapi.CreateProcess(
                command[0], subprocess.list2cmdline(command), None, None, True,
                subprocess.CREATE_NO_WINDOW | 0x4,  # CREATE_SUSPENDED
                None, str(cwd), info)
        finally:
            for handle, inheritable in zip(handles, previous):
                os.set_handle_inheritable(handle, inheritable)
        state['pid'] = pid
        if not kernel.AssignProcessToJobObject(job, process):
            raise c.WinError(c.get_last_error())
        if kernel.ResumeThread(thread) == 0xFFFFFFFF:
            raise c.WinError(c.get_last_error())
        state['ownership'] = 'windows_job_kill_on_close'
        result = _winapi.WaitForSingleObject(process, math.ceil(timeout*1000))
        if result == _winapi.WAIT_TIMEOUT:
            state['timed_out'] = True
            kernel.TerminateJobObject(job, 1)
            _winapi.WaitForSingleObject(process, 5000)
            raise TimeoutError(f'Owned CLI process exceeded {timeout:g}s; see {cwd}')
        if result != _winapi.WAIT_OBJECT_0:
            raise OSError(f'Unexpected process wait result: {result}')
        return _winapi.GetExitCodeProcess(process)
    finally:
        # Closing the job also cleans up descendants after a successful parent exit.
        kernel.CloseHandle(job)
        if process is not None:
            if _winapi.GetExitCodeProcess(process) == 259:
                _winapi.TerminateProcess(process, 1)  # e.g. assignment failed while suspended
                _winapi.WaitForSingleObject(process, 5000)
            _winapi.CloseHandle(process)
        if thread is not None:
            _winapi.CloseHandle(thread)


def run_logged(command, input_path, folder, timeout=180):
    """No shell; logs survive failures. Cleanup affects this call's tree only."""
    folder = Path(folder).resolve()
    folder.mkdir(parents=True, exist_ok=True)
    command = [str(item) for item in command]
    state = {'command': command, 'timeout_s': timeout, 'timed_out': False}
    started = time.monotonic()
    metadata = folder / 'process.json'

    def persist():
        metadata.write_text(json.dumps(state, indent=2, ensure_ascii=False), encoding='utf-8')

    persist()
    try:
        with open(input_path, 'rb') as stdin, (folder / 'stdout.jsonl').open('wb') as stdout, \
                (folder / 'stderr.log').open('wb') as stderr:
            if os.name == 'nt':
                code = _windows_run(command, folder, (stdin, stdout, stderr), timeout, state)
            else:
                process = subprocess.Popen(command, stdin=stdin, stdout=stdout, stderr=stderr,
                                           cwd=folder, start_new_session=True)
                state.update(pid=process.pid, ownership='posix_process_group')
                try:
                    code = process.wait(timeout=timeout)
                except subprocess.TimeoutExpired as exc:
                    state['timed_out'] = True
                    raise TimeoutError(f'Owned CLI process exceeded {timeout:g}s') from exc
                finally:
                    try:
                        os.killpg(process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    process.wait()
        state['returncode'] = code
        return code
    except BaseException as exc:
        state['error'] = str(exc)
        raise
    finally:
        state['elapsed_s'] = time.monotonic() - started
        persist()
