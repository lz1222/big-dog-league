"""Supervise a zero-velocity SDK FrontJump software flow.

This module deliberately has no ROS imports.  The gait node supplies
publishers, feedback, state callbacks, and parameter values; unit tests can
therefore use a fake clock and a fake process without contacting a robot.
"""

from collections import deque
from dataclasses import dataclass, field
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import socket
import stat
import subprocess
import tempfile
import threading
import time
import uuid


_INTERFACE_PATTERN = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.:-]{0,14}$')
_OUTPUT_CAPTURE_LIMIT_BYTES = 65536
_TEST_ONLY_SMOKE_HELPER_MARKER = (
    b'RK_NON_ARM_TEST_ONLY_FAKE_SDK_HELPER_V1'
)
_FEEDBACK_PROGRESS = {
    'acquire_gait_lock': 0.05,
    'publish_locomotion_zero': 0.10,
    'wait_final_cmd_zero': 0.20,
    'sdk_front_jump': 0.45,
    'post_settle': 0.70,
    'stability_unavailable': 0.85,
    'supervised_flow_done': 1.00,
}


class FrontJumpConfigurationError(ValueError):
    """Raised when a FrontJump parameter or runtime dependency is invalid."""


def is_test_only_smoke_helper(path):
    """只认可带固定标识的 ELF，防止 smoke 误调用真实 Unitree helper。

    该检查不是通用的二进制信任机制；它仅把 software smoke 的最小边界
    固化为随仓库测试源码编译的无网络 ELF。任何读取失败、脚本或无标识文件
    都按不可信处理，由调用方 fail-closed。
    """
    try:
        candidate = Path(str(path))
        with candidate.open('rb') as stream:
            contents = stream.read(_OUTPUT_CAPTURE_LIMIT_BYTES)
    except (OSError, TypeError, ValueError):
        return False
    return contents.startswith(b'\x7fELF') and (
        _TEST_ONLY_SMOKE_HELPER_MARKER in contents
    )


class CleanupGuardError(RuntimeError):
    """Raised when persistent cleanup evidence cannot be trusted."""


class ProcessIdentityError(RuntimeError):
    """Raised before a signal could target an unrelated process group."""


class ProcessStartError(RuntimeError):
    """Report whether Popen ran and whether emergency cleanup was verified."""

    def __init__(
        self,
        message,
        *,
        process_started,
        cleanup_completed,
        identity_unverified=False,
        diagnostics=None,
    ):
        super().__init__(message)
        self.process_started = bool(process_started)
        self.cleanup_completed = bool(cleanup_completed)
        self.identity_unverified = bool(identity_unverified)
        self.diagnostics = dict(diagnostics or {})


class _FlowExit(Exception):
    def __init__(self, terminal_state, stage, reason):
        super().__init__(reason)
        self.terminal_state = terminal_state
        self.stage = stage
        self.reason = reason


def _finite_number(value, name, *, positive=False, nonnegative=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise FrontJumpConfigurationError(
            '{} must be a finite number'.format(name)
        )
    number = float(value)
    if not math.isfinite(number):
        raise FrontJumpConfigurationError(
            '{} must be a finite number'.format(name)
        )
    if positive and number <= 0.0:
        raise FrontJumpConfigurationError(
            '{} must be greater than 0'.format(name)
        )
    if nonnegative and number < 0.0:
        raise FrontJumpConfigurationError(
            '{} must be greater than or equal to 0'.format(name)
        )
    return number


def _nonempty_string(value, name):
    if not isinstance(value, str) or not value.strip():
        raise FrontJumpConfigurationError(
            '{} must be a non-empty string'.format(name)
        )
    return value.strip()


def _read_boot_id():
    try:
        return Path('/proc/sys/kernel/random/boot_id').read_text(
            encoding='ascii'
        ).strip()
    except (OSError, UnicodeError) as error:
        raise CleanupGuardError(
            'linux boot_id is unavailable: {}'.format(error)
        ) from error


def _read_process_stat(pid):
    path = Path('/proc') / str(int(pid)) / 'stat'
    raw = path.read_text(encoding='ascii')
    close_index = raw.rfind(')')
    if close_index < 0:
        raise ProcessIdentityError('malformed /proc process stat')
    fields = raw[close_index + 2:].split()
    if len(fields) < 20:
        raise ProcessIdentityError('short /proc process stat')
    return {
        'state': fields[0],
        'parent_pid': int(fields[1]),
        'pgid': int(fields[2]),
        'session_id': int(fields[3]),
        'start_ticks': int(fields[19]),
    }


def _read_process_executable(pid):
    return os.path.realpath('/proc/{}/exe'.format(int(pid)))


def _process_group_members(pgid, session_id):
    members = []
    try:
        proc_entries = list(Path('/proc').iterdir())
    except OSError as error:
        raise ProcessIdentityError(
            'cannot inspect /proc: {}'.format(error)
        ) from error
    for entry in proc_entries:
        if not entry.name.isdigit():
            continue
        try:
            process_stat = _read_process_stat(int(entry.name))
        except (FileNotFoundError, ProcessLookupError):
            continue
        except PermissionError as error:
            raise ProcessIdentityError(
                'permission denied while inspecting /proc'
            ) from error
        except (OSError, ValueError, ProcessIdentityError) as error:
            raise ProcessIdentityError(
                'cannot verify a /proc process entry'
            ) from error
        if (
            process_stat['pgid'] == int(pgid)
            and process_stat['session_id'] == int(session_id)
        ):
            members.append(
                {
                    'pid': int(entry.name),
                    'state': process_stat['state'],
                    'start_ticks': process_stat['start_ticks'],
                }
            )
    return members


class PersistentCleanupGuard:
    """Atomic fail-closed journal for one FrontJump helper lifecycle."""

    SCHEMA_VERSION = 1
    VALID_STATES = {'DIRTY', 'CLEAN'}

    def __init__(self, path, *, boot_id_reader=None, wall_clock_ns=None):
        expanded = Path(str(path)).expanduser()
        self.path = Path(os.path.abspath(str(expanded)))
        self._boot_id_reader = boot_id_reader or _read_boot_id
        self._wall_clock_ns = wall_clock_ns or time.time_ns
        self._lock = threading.RLock()
        self._record = None
        self._invalid_reason = ''

    @property
    def current_record(self):
        with self._lock:
            if self._record is None:
                return None
            return json.loads(json.dumps(self._record))

    @property
    def invalid_reason(self):
        with self._lock:
            return self._invalid_reason

    @property
    def current_boot_id(self):
        return self._boot_id_reader()

    def load(self):
        with self._lock:
            if not self.path.exists() and not self.path.is_symlink():
                try:
                    self._validate_parent(create=True)
                except (OSError, CleanupGuardError) as error:
                    self._record = None
                    self._invalid_reason = '{}: {}'.format(
                        type(error).__name__,
                        str(error),
                    )
                    raise CleanupGuardError(
                        self._invalid_reason
                    ) from error
                self._record = None
                self._invalid_reason = ''
                return None
            try:
                self._validate_parent(create=False)
                flags = os.O_RDONLY
                if hasattr(os, 'O_NOFOLLOW'):
                    flags |= os.O_NOFOLLOW
                fd = os.open(str(self.path), flags)
                try:
                    file_stat = os.fstat(fd)
                    if not stat.S_ISREG(file_stat.st_mode):
                        raise CleanupGuardError(
                            'cleanup guard is not a regular file'
                        )
                    if file_stat.st_uid != os.geteuid():
                        raise CleanupGuardError(
                            'cleanup guard owner must be the effective user'
                        )
                    if stat.S_IMODE(file_stat.st_mode) != 0o600:
                        raise CleanupGuardError(
                            'cleanup guard permissions must be 0600'
                        )
                    with os.fdopen(fd, 'r', encoding='utf-8') as stream:
                        fd = -1
                        record = json.load(stream)
                finally:
                    if fd >= 0:
                        os.close(fd)
                self._validate_record(record)
            except (
                OSError,
                ValueError,
                TypeError,
                json.JSONDecodeError,
                CleanupGuardError,
            ) as error:
                self._record = None
                self._invalid_reason = '{}: {}'.format(
                    type(error).__name__,
                    str(error),
                )
                raise CleanupGuardError(self._invalid_reason) from error
            self._record = record
            self._invalid_reason = ''
            return self.current_record

    def begin_dirty(self, operation):
        with self._lock:
            if (
                self._record is not None
                or self.path.exists()
                or self.path.is_symlink()
            ):
                raise CleanupGuardError(
                    'cleanup guard is already armed'
                )
            now = int(self._wall_clock_ns())
            record = {
                'schema_version': self.SCHEMA_VERSION,
                'state': 'DIRTY',
                'cleanup_fault_id': uuid.uuid4().hex,
                'boot_id': self._boot_id_reader(),
                'created_at_unix_ns': now,
                'updated_at_unix_ns': now,
                'operation': {
                    'reservation_token': str(
                        operation.get('reservation_token', '')
                    ),
                    'entry_type': str(operation.get('entry_type', '')),
                    'motion_name': str(operation.get('motion_name', '')),
                    'goal_uuid': str(operation.get('goal_uuid', '')),
                    'command_identity': str(
                        operation.get('command_identity', '')
                    ),
                },
                'helper': {
                    'resolved_executable': '',
                    'pid': None,
                    'pgid': None,
                    'session_id': None,
                    'start_ticks': None,
                    'started': False,
                },
                'lock': {
                    'lock_acquire_command_published': False,
                    'lock_release_command_published': False,
                    'generation': 0,
                },
                'cleanup': {
                    'terminate_group_sent': False,
                    'kill_group_sent': False,
                    'leader_reaped': False,
                    'group_empty': False,
                    'cleanup_completed': False,
                },
                'faults': [],
            }
            self._atomic_write(record)
            self._record = record
            return json.loads(json.dumps(record))

    def update(self, mutator):
        with self._lock:
            if self._record is None:
                raise CleanupGuardError('cleanup guard is not armed')
            record = json.loads(json.dumps(self._record))
            mutator(record)
            record['updated_at_unix_ns'] = int(self._wall_clock_ns())
            self._validate_record(record)
            self._atomic_write(record)
            self._record = record
            return self.current_record

    def record_fault(self, fault_type, reason, *, operation=None):
        with self._lock:
            if self._record is None:
                self.begin_dirty(operation or {})

            def mutate(record):
                record['faults'].append(
                    {
                        'fault_id': uuid.uuid4().hex,
                        'fault_type': str(fault_type),
                        'reason': str(reason),
                    }
                )

            return self.update(mutate)

    def mark_clean_and_clear(self, expected_fault_id):
        with self._lock:
            if self._record is None:
                raise CleanupGuardError('cleanup guard is not loaded')
            if (
                self._record['cleanup_fault_id']
                != str(expected_fault_id)
            ):
                raise CleanupGuardError('cleanup fault ID changed')
            if not self._record['cleanup']['cleanup_completed']:
                raise CleanupGuardError(
                    'cleanup guard cannot clear before cleanup completes'
                )
            self.clear(expected_fault_id)

    def clear(self, expected_fault_id):
        with self._lock:
            if self._record is None:
                raise CleanupGuardError('cleanup guard is not loaded')
            if (
                self._record['cleanup_fault_id']
                != str(expected_fault_id)
            ):
                raise CleanupGuardError('cleanup fault ID mismatch')
            self._validate_parent(create=False)
            if self.path.is_symlink():
                raise CleanupGuardError(
                    'cleanup guard symbolic link is rejected'
                )
            on_disk_record = self.load()
            if (
                on_disk_record is None
                or on_disk_record['cleanup_fault_id']
                != str(expected_fault_id)
            ):
                raise CleanupGuardError(
                    'cleanup guard changed before clear'
                )
            directory_fd = -1
            try:
                # ``load()`` validates the parsed evidence. Re-open and
                # revalidate the pinned directory entry immediately before
                # removal so a chmod/type/symlink replacement cannot turn a
                # clean request into an unchecked destructive operation.
                directory_fd = self._open_parent_directory(create=False)
                if not self._validate_existing_guard_at(directory_fd):
                    raise CleanupGuardError(
                        'cleanup guard disappeared before clear'
                    )
                os.unlink(self.path.name, dir_fd=directory_fd)
                os.fsync(directory_fd)
            finally:
                if directory_fd >= 0:
                    os.close(directory_fd)
            self._record = None
            self._invalid_reason = ''

    def _validate_parent(self, *, create):
        parent = self.path.parent
        current = Path(parent.anchor)
        for part in parent.parts[1:]:
            current = current / part
            try:
                current_stat = os.lstat(str(current))
            except FileNotFoundError:
                if not create:
                    raise CleanupGuardError(
                        'cleanup guard parent does not exist'
                    )
                os.mkdir(str(current), mode=0o700)
                current_stat = os.lstat(str(current))
            if stat.S_ISLNK(current_stat.st_mode):
                raise CleanupGuardError(
                    'cleanup guard directory symbolic link is rejected'
                )
            if not stat.S_ISDIR(current_stat.st_mode):
                raise CleanupGuardError(
                    'cleanup guard parent is not a directory'
                )
        parent_stat = os.lstat(str(parent))
        if parent_stat.st_uid != os.geteuid():
            raise CleanupGuardError(
                'cleanup guard directory owner must be the effective user'
            )
        if stat.S_IMODE(parent_stat.st_mode) != 0o700:
            raise CleanupGuardError(
                'cleanup guard directory permissions must be 0700'
            )

    @staticmethod
    def _validate_guard_stat(file_stat):
        if stat.S_ISLNK(file_stat.st_mode):
            raise CleanupGuardError('cleanup guard symbolic link is rejected')
        if not stat.S_ISREG(file_stat.st_mode):
            raise CleanupGuardError('cleanup guard is not a regular file')
        if file_stat.st_uid != os.geteuid():
            raise CleanupGuardError(
                'cleanup guard owner must be the effective user'
            )
        if stat.S_IMODE(file_stat.st_mode) != 0o600:
            raise CleanupGuardError('cleanup guard permissions must be 0600')

    def _open_parent_directory(self, *, create):
        self._validate_parent(create=create)
        flags = os.O_RDONLY
        if hasattr(os, 'O_DIRECTORY'):
            flags |= os.O_DIRECTORY
        if hasattr(os, 'O_NOFOLLOW'):
            flags |= os.O_NOFOLLOW
        directory_fd = os.open(str(self.path.parent), flags)
        try:
            parent_stat = os.fstat(directory_fd)
            if (
                not stat.S_ISDIR(parent_stat.st_mode)
                or parent_stat.st_uid != os.geteuid()
                or stat.S_IMODE(parent_stat.st_mode) != 0o700
            ):
                raise CleanupGuardError(
                    'cleanup guard parent changed during validation'
                )
            return directory_fd
        except Exception:
            os.close(directory_fd)
            raise

    def _validate_existing_guard_at(self, directory_fd):
        try:
            file_stat = os.stat(
                self.path.name,
                dir_fd=directory_fd,
                follow_symlinks=False,
            )
        except FileNotFoundError:
            return False
        self._validate_guard_stat(file_stat)
        return True

    def _atomic_write(self, record):
        self._validate_record(record)
        payload = (
            json.dumps(
                record,
                sort_keys=True,
                separators=(',', ':'),
            )
            + '\n'
        ).encode('utf-8')
        temporary_name = '.{}.{}.tmp'.format(
            self.path.name,
            uuid.uuid4().hex,
        )
        flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
        if hasattr(os, 'O_NOFOLLOW'):
            flags |= os.O_NOFOLLOW
        fd = -1
        directory_fd = -1
        try:
            directory_fd = self._open_parent_directory(create=True)
            self._validate_existing_guard_at(directory_fd)
            fd = os.open(
                temporary_name,
                flags,
                0o600,
                dir_fd=directory_fd,
            )
            os.fchmod(fd, 0o600)
            with os.fdopen(fd, 'wb') as stream:
                fd = -1
                stream.write(payload)
                stream.flush()
                os.fsync(stream.fileno())
            # The directory fd pins the already-validated parent. Re-check
            # the target immediately before replacement; an unsafe chmod,
            # type swap, or symlink must fail closed rather than be masked.
            self._validate_existing_guard_at(directory_fd)
            os.replace(
                temporary_name,
                self.path.name,
                src_dir_fd=directory_fd,
                dst_dir_fd=directory_fd,
            )
            os.fsync(directory_fd)
        finally:
            if fd >= 0:
                os.close(fd)
            try:
                if directory_fd >= 0:
                    os.unlink(temporary_name, dir_fd=directory_fd)
            except FileNotFoundError:
                pass
            finally:
                if directory_fd >= 0:
                    os.close(directory_fd)

    @classmethod
    def _validate_record(cls, record):
        def require_string(container, key, *, nonempty=False):
            value = container.get(key)
            if not isinstance(value, str):
                raise CleanupGuardError(
                    '{} must be a string'.format(key)
                )
            if nonempty and not value:
                raise CleanupGuardError(
                    '{} must not be empty'.format(key)
                )
            return value

        def require_bool(container, key):
            if not isinstance(container.get(key), bool):
                raise CleanupGuardError(
                    '{} must be a boolean'.format(key)
                )

        def require_nonnegative_int(container, key):
            value = container.get(key)
            if (
                isinstance(value, bool)
                or not isinstance(value, int)
                or value < 0
            ):
                raise CleanupGuardError(
                    '{} must be a non-negative integer'.format(key)
                )

        def require_optional_positive_int(container, key):
            value = container.get(key)
            if value is None:
                return
            if (
                isinstance(value, bool)
                or not isinstance(value, int)
                or value <= 0
            ):
                raise CleanupGuardError(
                    '{} must be null or a positive integer'.format(key)
                )

        if not isinstance(record, dict):
            raise CleanupGuardError('cleanup guard root must be an object')
        if record.get('schema_version') != cls.SCHEMA_VERSION:
            raise CleanupGuardError('unknown cleanup guard schema')
        if record.get('state') not in cls.VALID_STATES:
            raise CleanupGuardError('invalid cleanup guard state')
        for key in (
            'cleanup_fault_id',
            'boot_id',
            'operation',
            'helper',
            'lock',
            'cleanup',
            'faults',
        ):
            if key not in record:
                raise CleanupGuardError(
                    'cleanup guard missing {}'.format(key)
                )
        require_string(
            record,
            'cleanup_fault_id',
            nonempty=True,
        )
        try:
            parsed_fault_id = uuid.UUID(
                hex=record['cleanup_fault_id']
            )
        except (AttributeError, TypeError, ValueError) as error:
            raise CleanupGuardError(
                'cleanup fault ID must be UUID4 hex'
            ) from error
        if (
            parsed_fault_id.version != 4
            or parsed_fault_id.hex != record['cleanup_fault_id']
        ):
            raise CleanupGuardError(
                'cleanup fault ID must be UUID4 hex'
            )
        require_string(record, 'boot_id', nonempty=True)
        require_nonnegative_int(record, 'created_at_unix_ns')
        require_nonnegative_int(record, 'updated_at_unix_ns')
        if (
            record['updated_at_unix_ns']
            < record['created_at_unix_ns']
        ):
            raise CleanupGuardError(
                'cleanup guard update time precedes create time'
            )
        if not isinstance(record['operation'], dict):
            raise CleanupGuardError('operation must be an object')
        if not isinstance(record['helper'], dict):
            raise CleanupGuardError('helper must be an object')
        if not isinstance(record['lock'], dict):
            raise CleanupGuardError('lock must be an object')
        if not isinstance(record['cleanup'], dict):
            raise CleanupGuardError('cleanup must be an object')
        if not isinstance(record['faults'], list):
            raise CleanupGuardError('faults must be an array')
        for key in (
            'reservation_token',
            'entry_type',
            'motion_name',
            'goal_uuid',
            'command_identity',
        ):
            require_string(record['operation'], key)
        if record['operation']['entry_type'] not in ('', 'action', 'json'):
            raise CleanupGuardError(
                'operation entry_type must be action or json'
            )
        for key in (
            'resolved_executable',
            'pid',
            'pgid',
            'session_id',
            'start_ticks',
            'started',
        ):
            if key not in record['helper']:
                raise CleanupGuardError(
                    'helper missing {}'.format(key)
                )
        require_string(record['helper'], 'resolved_executable')
        for key in ('pid', 'pgid', 'session_id', 'start_ticks'):
            require_optional_positive_int(record['helper'], key)
        require_bool(record['helper'], 'started')
        if record['helper']['started']:
            resolved_executable = record['helper'][
                'resolved_executable'
            ]
            if (
                not os.path.isabs(resolved_executable)
                or os.path.realpath(resolved_executable)
                != resolved_executable
            ):
                raise CleanupGuardError(
                    'started helper executable must be absolute '
                    'and normalized'
                )
            for key in ('pid', 'pgid', 'session_id', 'start_ticks'):
                if record['helper'][key] is None:
                    raise CleanupGuardError(
                        'started helper identity is incomplete'
                    )
            if (
                record['helper']['pgid'] != record['helper']['pid']
                or record['helper']['session_id']
                != record['helper']['pid']
            ):
                raise CleanupGuardError(
                    'started helper process group/session identity '
                    'is invalid'
                )
        for key in (
            'lock_acquire_command_published',
            'lock_release_command_published',
            'generation',
        ):
            if key not in record['lock']:
                raise CleanupGuardError(
                    'lock missing {}'.format(key)
                )
        require_bool(record['lock'], 'lock_acquire_command_published')
        require_bool(record['lock'], 'lock_release_command_published')
        require_nonnegative_int(record['lock'], 'generation')
        for key in (
            'terminate_group_sent',
            'kill_group_sent',
            'leader_reaped',
            'group_empty',
            'cleanup_completed',
        ):
            if key not in record['cleanup']:
                raise CleanupGuardError(
                    'cleanup missing {}'.format(key)
                )
            require_bool(record['cleanup'], key)
        for fault in record['faults']:
            if not isinstance(fault, dict):
                raise CleanupGuardError(
                    'cleanup guard fault must be an object'
                )
            for key in ('fault_id', 'fault_type', 'reason'):
                require_string(fault, key, nonempty=True)
            try:
                parsed_fault_id = uuid.UUID(hex=fault['fault_id'])
            except (AttributeError, TypeError, ValueError) as error:
                raise CleanupGuardError(
                    'fault ID must be UUID4 hex'
                ) from error
            if (
                parsed_fault_id.version != 4
                or parsed_fault_id.hex != fault['fault_id']
            ):
                raise CleanupGuardError(
                    'fault ID must be UUID4 hex'
                )


@dataclass(frozen=True)
class FrontJumpProfile:
    """Independent, immutable start or finish supervision parameters."""

    name: str
    pre_stop_duration: float
    final_zero_epsilon: float
    final_zero_confirm_samples: int
    final_zero_timeout: float
    sdk_timeout: float
    post_settle_duration: float

    def __post_init__(self):
        object.__setattr__(
            self, 'name', _nonempty_string(self.name, 'profile.name')
        )
        object.__setattr__(
            self,
            'pre_stop_duration',
            _finite_number(
                self.pre_stop_duration,
                '{}.pre_stop_duration'.format(self.name),
                nonnegative=True,
            ),
        )
        object.__setattr__(
            self,
            'final_zero_epsilon',
            _finite_number(
                self.final_zero_epsilon,
                '{}.final_zero_epsilon'.format(self.name),
                positive=True,
            ),
        )
        if (
            isinstance(self.final_zero_confirm_samples, bool)
            or not isinstance(self.final_zero_confirm_samples, int)
            or self.final_zero_confirm_samples <= 0
        ):
            raise FrontJumpConfigurationError(
                '{}.final_zero_confirm_samples must be a positive integer'
                .format(self.name)
            )
        object.__setattr__(
            self,
            'final_zero_timeout',
            _finite_number(
                self.final_zero_timeout,
                '{}.final_zero_timeout'.format(self.name),
                positive=True,
            ),
        )
        object.__setattr__(
            self,
            'sdk_timeout',
            _finite_number(
                self.sdk_timeout,
                '{}.sdk_timeout'.format(self.name),
                positive=True,
            ),
        )
        object.__setattr__(
            self,
            'post_settle_duration',
            _finite_number(
                self.post_settle_duration,
                '{}.post_settle_duration'.format(self.name),
                nonnegative=True,
            ),
        )

    @property
    def worst_case_duration_sec(self):
        """返回软件监管各等待窗口的保守总时长。

        此值供上游 Action 超时配置校验使用；它不是实体跳跃完成证明，
        只覆盖 pre-stop、最终零速确认、SDK 调用和动作后静置窗口。
        """
        return (
            self.pre_stop_duration
            + self.final_zero_timeout
            + self.sdk_timeout
            + self.post_settle_duration
        )


@dataclass(frozen=True)
class FrontJumpConfig:
    """Common immutable FrontJump runtime configuration."""

    sdk_action_executable: str
    sdk_network_interface: str
    zero_publish_rate_hz: float
    final_cmd_stale_timeout: float
    estop_state_stale_timeout: float
    # smoke 只允许仓库测试 ELF，绝不允许调用 Unitree SDK helper。
    software_smoke_mode: bool = False

    def __post_init__(self):
        executable = _nonempty_string(
            self.sdk_action_executable, 'sdk_action_executable'
        )
        interface = _nonempty_string(
            self.sdk_network_interface, 'sdk_network_interface'
        )
        if _INTERFACE_PATTERN.fullmatch(interface) is None:
            raise FrontJumpConfigurationError(
                'sdk_network_interface is not a safe interface name'
            )
        object.__setattr__(self, 'sdk_action_executable', executable)
        object.__setattr__(self, 'sdk_network_interface', interface)
        object.__setattr__(
            self,
            'zero_publish_rate_hz',
            _finite_number(
                self.zero_publish_rate_hz,
                'zero_publish_rate_hz',
                positive=True,
            ),
        )
        object.__setattr__(
            self,
            'final_cmd_stale_timeout',
            _finite_number(
                self.final_cmd_stale_timeout,
                'final_cmd_stale_timeout',
                positive=True,
            ),
        )
        object.__setattr__(
            self,
            'estop_state_stale_timeout',
            _finite_number(
                self.estop_state_stale_timeout,
                'estop_state_stale_timeout',
                positive=True,
            ),
        )
        if not isinstance(self.software_smoke_mode, bool):
            raise FrontJumpConfigurationError(
                'software_smoke_mode must be a boolean'
            )


@dataclass(frozen=True)
class FinalCommandSample:
    """One newly received final command sample."""

    sequence: int
    receive_time: float
    linear_x: float
    linear_y: float
    angular_z: float


@dataclass(frozen=True)
class EstopSample:
    """Latest typed estop heartbeat sample."""

    sequence: int
    receive_time: float
    active: bool


@dataclass(frozen=True)
class ProcessResult:
    """Captured helper process result."""

    return_code: int
    stdout: str
    stderr: str


@dataclass(frozen=True)
class FrontJumpOutcome:
    """Internal structured result; it is not an Action schema."""

    success: bool
    terminal_state: str
    stage: str
    reason: str
    helper_started: bool
    sdk_request_may_have_been_sent: bool
    cleanup_completed: bool
    sdk_command_accepted: bool
    post_settle_completed: bool
    physical_crossing_unverified: bool = True
    lock_acquire_command_published: bool = False
    lock_release_command_published: bool = False
    helper_group_empty: bool = True

    def message(self):
        fields = []
        if self.success:
            fields.extend(
                [
                    'supervised_front_jump_flow_completed',
                    'sdk_command_accepted=true',
                    'post_settle_completed=true',
                    'physical_crossing_unverified=true',
                ]
            )
        fields.extend(
            [
                'stage={}'.format(self.stage),
                'reason={}'.format(self.reason),
                'helper_started={}'.format(
                    str(self.helper_started).lower()
                ),
                'sdk_request_may_have_been_sent={}'.format(
                    str(self.sdk_request_may_have_been_sent).lower()
                ),
                'cleanup_completed={}'.format(
                    str(self.cleanup_completed).lower()
                ),
            ]
        )
        return ';'.join(fields)


@dataclass
class _GoalContext:
    motion_name: str
    profile: FrontJumpProfile
    goal_id: str
    cancel_requested: threading.Event
    gait_stop_requested: threading.Event
    created_at: float
    stage: str = 'created'
    feedback_enabled: bool = True
    lock_owned: bool = False
    lock_acquire_command_published: bool = False
    lock_release_command_published: bool = False
    lock_generation: int = 0
    zero_allowed: bool = False
    first_zero_time: object = None
    last_zero_time: object = None
    zero_publish_count: int = 0
    helper_started: bool = False
    helper_path: str = ''
    sdk_request_may_have_been_sent: bool = False
    sdk_command_accepted: bool = False
    post_settle_completed: bool = False
    helper_process: object = None
    helper_reaped: bool = False
    helper_return_code: object = None
    helper_stdout: str = ''
    helper_stderr: str = ''
    helper_terminate_sent: bool = False
    helper_kill_sent: bool = False
    helper_group_empty: bool = True
    ros_publish_faults: list = field(default_factory=list)
    local_cleanup_events: list = field(default_factory=list)
    stop_reason: str = ''
    guard_fault_id: str = ''
    guard_armed: bool = False
    guard_update_failed: bool = False
    completion_event: threading.Event = field(
        default_factory=threading.Event
    )
    final_window_start: object = None
    final_last_processed_sequence: int = 0
    final_last_sample_time: object = None
    final_zero_streak: int = 0
    final_zero_required: int = 0
    last_final_sample: object = None
    cleanup_completed: bool = False
    event_history: list = field(default_factory=list)


class _SubprocessHandle:
    """Popen wrapper that captures output without pipe-buffer deadlock."""

    def __init__(
        self,
        process,
        stdout_file,
        stderr_file,
        expected_executable,
    ):
        self._process = process
        self._stdout_file = stdout_file
        self._stderr_file = stderr_file
        self._result = None
        self._expected_executable = os.path.realpath(
            str(expected_executable)
        )
        identity = self._capture_identity(
            process,
            self._expected_executable,
            deadline=time.monotonic() + 0.20,
        )
        self._pgid = identity['pgid']
        self._session_id = identity['session_id']
        self._start_ticks = identity['start_ticks']
        self._observed_executable = identity['executable']

    @staticmethod
    def _capture_identity(process, expected_executable, *, deadline):
        """Collect one complete, bounded identity before group signalling."""

        last_error = None
        while True:
            if process.poll() is not None:
                raise ProcessIdentityError(
                    'helper exited before identity collection completed'
                )
            try:
                process_stat = _read_process_stat(process.pid)
                pgid = os.getpgid(process.pid)
                session_id = os.getsid(process.pid)
                executable = _read_process_executable(process.pid)
                if pgid != process.pid:
                    raise ProcessIdentityError(
                        'helper process group ID must equal its PID'
                    )
                if session_id != process.pid:
                    raise ProcessIdentityError(
                        'helper session ID must equal its PID'
                    )
                if executable != expected_executable:
                    raise ProcessIdentityError(
                        'helper executable identity does not match argv[0]'
                    )
                return {
                    'pid': int(process.pid),
                    'pgid': int(pgid),
                    'session_id': int(session_id),
                    'start_ticks': int(process_stat['start_ticks']),
                    'executable': executable,
                }
            except (OSError, ValueError, ProcessIdentityError) as error:
                last_error = error
            if time.monotonic() >= deadline:
                raise ProcessIdentityError(
                    'helper identity collection failed: {}'.format(
                        last_error
                    )
                ) from last_error
            time.sleep(0.01)

    @property
    def pid(self):
        return self._process.pid

    @property
    def pgid(self):
        return self._pgid

    @property
    def session_id(self):
        return self._session_id

    @property
    def start_ticks(self):
        return self._start_ticks

    @property
    def executable(self):
        return self._observed_executable

    def poll(self):
        return self._process.poll()

    def terminate(self):
        self._signal_group(signal.SIGTERM)

    def kill(self):
        self._signal_group(signal.SIGKILL)

    def group_members(self):
        return _process_group_members(self._pgid, self._session_id)

    def group_empty(self):
        return not self.group_members()

    def _signal_group(self, signal_number):
        self._verify_signal_identity()
        try:
            os.killpg(self._pgid, signal_number)
        except ProcessLookupError:
            return

    def _verify_signal_identity(self):
        if self._process.poll() is None:
            process_stat = _read_process_stat(self.pid)
            if process_stat['start_ticks'] != self._start_ticks:
                raise ProcessIdentityError(
                    'helper PID start ticks changed'
                )
            if process_stat['pgid'] != self._pgid:
                raise ProcessIdentityError('helper PGID changed')
            if process_stat['session_id'] != self._session_id:
                raise ProcessIdentityError('helper session ID changed')
            if _read_process_executable(self.pid) != self._observed_executable:
                raise ProcessIdentityError(
                    'helper executable identity changed'
                )
            return

        for member in self.group_members():
            process_stat = _read_process_stat(member['pid'])
            if (
                process_stat['pgid'] != self._pgid
                or process_stat['session_id'] != self._session_id
            ):
                raise ProcessIdentityError(
                    'helper process group identity changed'
                )

    def reap(self, timeout):
        if self._result is not None:
            return self._result
        return_code = self._process.wait(timeout=timeout)
        stdout = self._read_capped_output(self._stdout_file)
        stderr = self._read_capped_output(self._stderr_file)
        self._stdout_file.close()
        self._stderr_file.close()
        self._result = ProcessResult(return_code, stdout, stderr)
        return self._result

    @staticmethod
    def _read_capped_output(stream):
        stream.seek(0, os.SEEK_END)
        size = stream.tell()
        truncated = size > _OUTPUT_CAPTURE_LIMIT_BYTES
        if truncated:
            stream.seek(
                size - _OUTPUT_CAPTURE_LIMIT_BYTES,
                os.SEEK_SET,
            )
        else:
            stream.seek(0)
        payload = stream.read(_OUTPUT_CAPTURE_LIMIT_BYTES)
        text = payload.decode('utf-8', errors='replace')
        if truncated:
            return '[truncated {} bytes]\\n{}'.format(
                size - len(payload),
                text,
            )
        return text


class SubprocessRunner:
    """Start the real SDK helper as a fixed argv process."""

    def start(self, argv):
        stdout_file = tempfile.TemporaryFile(mode='w+b')
        stderr_file = tempfile.TemporaryFile(mode='w+b')
        process = None
        try:
            process = subprocess.Popen(
                list(argv),
                shell=False,
                stdout=stdout_file,
                stderr=stderr_file,
                start_new_session=True,
            )
            return _SubprocessHandle(
                process,
                stdout_file,
                stderr_file,
                argv[0],
            )
        except Exception as error:
            cleanup_completed = process is None
            identity_unverified = process is not None
            diagnostics = {
                'failure_stage': 'popen'
                if process is None
                else 'post_popen_identity_collection',
            }
            if process is not None:
                diagnostics['spawn_pid'] = int(process.pid)
                diagnostics['leader_terminate_sent'] = False
                diagnostics['leader_kill_sent'] = False
                diagnostics['leader_reaped'] = False
                try:
                    diagnostics['observed_pgid'] = os.getpgid(process.pid)
                except (OSError, ProcessLookupError):
                    diagnostics['observed_pgid'] = None
                try:
                    diagnostics['observed_session_id'] = os.getsid(
                        process.pid
                    )
                except (OSError, ProcessLookupError):
                    diagnostics['observed_session_id'] = None

                # Identity collection failed. Never signal an unverified
                # PGID: only the Popen leader object may receive a bounded
                # best-effort signal, and group cleanup remains unverified.
                try:
                    if process.poll() is None:
                        process.terminate()
                        diagnostics['leader_terminate_sent'] = True
                except Exception:
                    pass
                try:
                    process.wait(timeout=0.20)
                except Exception:
                    pass
                try:
                    if process.poll() is None:
                        process.kill()
                        diagnostics['leader_kill_sent'] = True
                except Exception:
                    pass
                try:
                    process.wait(timeout=0.50)
                except Exception:
                    pass
                diagnostics['leader_reaped'] = bool(
                    process.poll() is not None
                )
                # A failed identity collection cannot prove a whole process
                # group is absent, even if the leader was reaped.
                cleanup_completed = False
            stdout_file.close()
            stderr_file.close()
            raise ProcessStartError(
                '{}: {}'.format(type(error).__name__, str(error)),
                process_started=process is not None,
                cleanup_completed=cleanup_completed,
                identity_unverified=identity_unverified,
                diagnostics=diagnostics,
            ) from error


def resolve_sdk_executable(configured, *, environment=None, which=None):
    """Resolve the configured helper without using the current directory."""

    configured = _nonempty_string(configured, 'sdk_action_executable')
    environment = os.environ if environment is None else environment
    which = shutil.which if which is None else which
    path = Path(configured)

    if path.is_absolute():
        if not path.exists():
            raise FrontJumpConfigurationError(
                'absolute SDK helper does not exist'
            )
        if not path.is_file():
            raise FrontJumpConfigurationError(
                'absolute SDK helper is not a regular file'
            )
        if not os.access(str(path), os.X_OK):
            raise FrontJumpConfigurationError(
                'absolute SDK helper is not executable'
            )
        return str(path)

    if '/' in configured or '\\' in configured:
        raise FrontJumpConfigurationError(
            'relative SDK helper paths with separators are rejected'
        )

    path_value = environment.get('PATH')
    resolved = which(configured, path=path_value)
    if resolved:
        resolved_path = Path(resolved)
        if resolved_path.is_file() and os.access(str(resolved_path), os.X_OK):
            return str(resolved_path.resolve())

    prefixes = environment.get('AMENT_PREFIX_PATH', '')
    for prefix in prefixes.split(os.pathsep):
        if not prefix:
            continue
        candidate = (
            Path(prefix)
            / 'lib'
            / 'rk_go2_sdk_bridge'
            / configured
        )
        if candidate.is_file() and os.access(str(candidate), os.X_OK):
            return str(candidate.resolve())

    raise FrontJumpConfigurationError('SDK helper executable was not found')


class FrontJumpSupervisor:
    """Own a FrontJump goal's gait lock and zero-command lifecycle."""

    def __init__(
        self,
        *,
        profiles,
        config,
        publish_lock,
        publish_zero,
        process_runner=None,
        clock=None,
        waiter=None,
        feedback_callback=None,
        event_logger=None,
        executable_resolver=None,
        interface_index_resolver=None,
        cleanup_guard=None,
        ros_cleanup_allowed=None,
    ):
        if not isinstance(config, FrontJumpConfig):
            raise FrontJumpConfigurationError(
                'config must be a FrontJumpConfig'
            )
        if set(profiles) != {'start', 'finish'}:
            raise FrontJumpConfigurationError(
                'profiles must contain independent start and finish entries'
            )
        start_profile = profiles['start']
        finish_profile = profiles['finish']
        if not isinstance(start_profile, FrontJumpProfile):
            raise FrontJumpConfigurationError(
                'start profile must be a FrontJumpProfile'
            )
        if not isinstance(finish_profile, FrontJumpProfile):
            raise FrontJumpConfigurationError(
                'finish profile must be a FrontJumpProfile'
            )
        if start_profile is finish_profile:
            raise FrontJumpConfigurationError(
                'start and finish profiles must be independent objects'
            )
        if start_profile.name != 'start' or finish_profile.name != 'finish':
            raise FrontJumpConfigurationError(
                'profile names must be start and finish'
            )
        if not callable(publish_lock) or not callable(publish_zero):
            raise FrontJumpConfigurationError(
                'publish_lock and publish_zero must be callable'
            )

        self.profiles = {
            'start': start_profile,
            'finish': finish_profile,
        }
        self.config = config
        self._publish_lock_callback = publish_lock
        self._publish_zero_callback = publish_zero
        self._process_runner = process_runner or SubprocessRunner()
        self._clock = clock or time.monotonic
        self._waiter = waiter
        self._default_feedback_callback = feedback_callback
        self._event_logger = event_logger
        self._resolve_executable = (
            executable_resolver or resolve_sdk_executable
        )
        self._interface_index = (
            interface_index_resolver or socket.if_nametoindex
        )
        self.cleanup_guard = cleanup_guard
        self._ros_cleanup_allowed_callback = (
            ros_cleanup_allowed or (lambda: True)
        )

        self._condition = threading.Condition(threading.RLock())
        self._final_samples = deque(maxlen=2048)
        self._final_sequence = 0
        self._estop_sample = None
        self._estop_sequence = 0
        self._mux_status = None
        self._active_context = None
        self._feedback_callback = None
        self.completed_contexts = []
        self._last_cleanup_context = None

    @property
    def active_context(self):
        with self._condition:
            return self._active_context

    @property
    def active_process(self):
        with self._condition:
            context = self._active_context or self._last_cleanup_context
            if context is None:
                return None
            process = context.helper_process
            if process is None or context.helper_group_empty:
                return None
            return process

    @property
    def cleanup_pending(self):
        with self._condition:
            context = self._last_cleanup_context
            return bool(
                context is not None and not context.cleanup_completed
            )

    @property
    def completion_event(self):
        with self._condition:
            context = self._active_context
            if context is None:
                event = threading.Event()
                event.set()
                return event
            return context.completion_event

    def select_profile(self, motion_name):
        normalized = str(motion_name or '').strip().lower()
        if normalized in ('start_jump', 'jump_start_obstacle'):
            return self.profiles['start']
        if normalized in (
            'finish_jump',
            'end_jump',
            'jump_end_obstacle',
        ):
            return self.profiles['finish']
        raise FrontJumpConfigurationError(
            'unsupported FrontJump motion name'
        )

    def update_final_command(
        self,
        linear_x,
        linear_y,
        angular_z,
        *,
        receive_time=None,
    ):
        now = self._clock() if receive_time is None else float(receive_time)
        with self._condition:
            self._final_sequence += 1
            sample = FinalCommandSample(
                sequence=self._final_sequence,
                receive_time=now,
                linear_x=float(linear_x),
                linear_y=float(linear_y),
                angular_z=float(angular_z),
            )
            self._final_samples.append(sample)
            self._condition.notify_all()
            return sample

    def update_estop(self, active, *, receive_time=None):
        now = self._clock() if receive_time is None else float(receive_time)
        with self._condition:
            self._estop_sequence += 1
            self._estop_sample = EstopSample(
                sequence=self._estop_sequence,
                receive_time=now,
                active=bool(active),
            )
            self._condition.notify_all()
            return self._estop_sample

    def update_mux_status(self, raw_status, *, receive_time=None):
        now = self._clock() if receive_time is None else float(receive_time)
        raw_text = str(raw_status)
        try:
            parsed = json.loads(raw_text)
        except (TypeError, ValueError):
            parsed = None
        with self._condition:
            self._mux_status = {
                'receive_time': now,
                'raw': raw_text,
                'parsed': parsed,
            }
            self._condition.notify_all()

    def wake(self):
        with self._condition:
            self._condition.notify_all()

    def request_cancel(self):
        with self._condition:
            context = self._active_context
            if context is None:
                return False
            context.cancel_requested.set()
            if not context.stop_reason:
                context.stop_reason = 'cancel_requested'
            self._condition.notify_all()
            return True

    def request_gait_stop(self):
        return self.request_stop('gait_stop_requested')

    def request_stop(self, reason='node_shutdown'):
        with self._condition:
            context = self._active_context
            if context is None:
                return False
            context.gait_stop_requested.set()
            if not context.stop_reason:
                context.stop_reason = str(reason)
            self._condition.notify_all()
            return True

    def begin_recovery_window(self):
        with self._condition:
            return self._final_sequence

    def wait_for_update(self, timeout):
        with self._condition:
            self._condition.wait(timeout=max(0.0, float(timeout)))

    def recovery_evidence_ready(
        self,
        baseline_sequence,
        *,
        confirm_samples,
        epsilon,
    ):
        now = self._clock()
        with self._condition:
            estop = self._estop_sample
            samples = [
                sample
                for sample in self._final_samples
                if sample.sequence > int(baseline_sequence)
            ]
        if (
            estop is None
            or estop.active
            or now - estop.receive_time
            > self.config.estop_state_stale_timeout
        ):
            return False, 'typed estop is missing, active, or stale'

        streak = 0
        for sample in samples:
            if (
                now - sample.receive_time
                > self.config.final_cmd_stale_timeout
                or not self._is_zero_sample(sample, float(epsilon))
            ):
                streak = 0
                continue
            streak += 1
        if streak < int(confirm_samples):
            return False, 'fresh final-cmd zero evidence is insufficient'
        if not samples:
            return False, 'no new final-cmd sample was received'
        if (
            now - samples[-1].receive_time
            > self.config.final_cmd_stale_timeout
        ):
            return False, 'latest final-cmd sample is stale'
        return True, ''

    @staticmethod
    def guard_process_absent(record):
        helper = record.get('helper', {})
        if not helper.get('started'):
            return True, ''
        required = (
            'pid',
            'pgid',
            'session_id',
            'start_ticks',
            'resolved_executable',
        )
        if any(helper.get(key) in (None, '') for key in required):
            return False, 'helper identity is incomplete'
        pid = int(helper['pid'])
        pgid = int(helper['pgid'])
        session_id = int(helper['session_id'])
        try:
            process_stat = _read_process_stat(pid)
            executable = _read_process_executable(pid)
        except FileNotFoundError:
            process_stat = None
            executable = ''
        except (OSError, ValueError, ProcessIdentityError) as error:
            return False, 'helper identity cannot be verified: {}'.format(
                error
            )

        if process_stat is not None:
            if (
                process_stat['start_ticks'] == int(helper['start_ticks'])
                and process_stat['pgid'] == pgid
                and process_stat['session_id'] == session_id
                and executable
                == os.path.realpath(helper['resolved_executable'])
            ):
                return False, 'recorded helper leader is still active'

        try:
            members = _process_group_members(pgid, session_id)
        except ProcessIdentityError as error:
            return False, str(error)
        if members:
            return False, 'recorded helper process group is still active'
        return True, ''

    def retry_cleanup(self, *, allow_ros=True):
        with self._condition:
            if self._active_context is not None:
                return False
            context = self._last_cleanup_context
        if context is None:
            return True
        if context.cleanup_completed:
            return True

        helper_clean = self._terminate_and_reap(context)
        final_zero_clean = True
        unlock_clean = False
        guard_clean = self._update_guard_cleanup(context, helper_clean)
        allow_ros = bool(
            allow_ros and self._ros_cleanup_is_allowed()
        )
        if context.lock_owned and allow_ros:
            context.zero_allowed = True
            try:
                self._publish_zero(context, force=True)
            except Exception:
                final_zero_clean = False
            finally:
                context.zero_allowed = False
            if (
                helper_clean
                and final_zero_clean
                and guard_clean
                and not context.guard_update_failed
            ):
                try:
                    result = self._publish_lock_callback(False)
                    context.lock_release_command_published = bool(
                        getattr(result, 'publish_succeeded', True)
                    )
                    context.lock_generation = int(
                        getattr(
                            result,
                            'generation',
                            context.lock_generation,
                        )
                    )
                except Exception:
                    context.lock_release_command_published = False
                unlock_clean = context.lock_release_command_published
                if unlock_clean:
                    context.lock_owned = False
                    guard_clean = self._finish_guard_cleanup(context)
                    if not guard_clean:
                        context.lock_owned = True
                        try:
                            result = self._publish_lock_callback(True)
                            context.lock_generation = int(
                                getattr(
                                    result,
                                    'generation',
                                    context.lock_generation,
                                )
                            )
                        except Exception:
                            pass
        context.cleanup_completed = bool(
            helper_clean
            and final_zero_clean
            and unlock_clean
            and guard_clean
        )
        return context.cleanup_completed

    def run(
        self,
        motion_name,
        *,
        goal_id='',
        cancel_requested=None,
        gait_stop_requested=None,
        feedback_callback=None,
        reservation_token='',
        entry_type='',
        command_identity='',
    ):
        profile = self.select_profile(motion_name)
        context = _GoalContext(
            motion_name=str(motion_name),
            profile=profile,
            goal_id=str(goal_id),
            cancel_requested=cancel_requested or threading.Event(),
            gait_stop_requested=gait_stop_requested or threading.Event(),
            created_at=self._clock(),
        )
        terminal_state = 'abort'
        terminal_stage = 'created'
        terminal_reason = 'supervision_not_started'

        with self._condition:
            if self._active_context is not None:
                return FrontJumpOutcome(
                    success=False,
                    terminal_state='abort',
                    stage='acquire_gait_lock',
                    reason='another_front_jump_is_running',
                    helper_started=False,
                    sdk_request_may_have_been_sent=False,
                    cleanup_completed=True,
                    sdk_command_accepted=False,
                    post_settle_completed=False,
                )
            self._active_context = context
            self._feedback_callback = (
                feedback_callback or self._default_feedback_callback
            )

        try:
            if self.cleanup_guard is not None:
                try:
                    record = self.cleanup_guard.begin_dirty(
                        {
                            'reservation_token': reservation_token,
                            'entry_type': entry_type,
                            'motion_name': motion_name,
                            'goal_uuid': goal_id,
                            'command_identity': command_identity,
                        }
                    )
                    context.guard_fault_id = record['cleanup_fault_id']
                    context.guard_armed = True
                except Exception as error:
                    context.guard_update_failed = True
                    self._log(
                        context,
                        'cleanup_guard_arm_failed',
                        error_type=type(error).__name__,
                        error=str(error),
                    )
                    raise _FlowExit(
                        'abort',
                        context.stage,
                        'cleanup_guard_arm_failed',
                    )

            self._stage(context, 'acquire_gait_lock')
            context.lock_owned = True
            lock_result = self._publish_lock_callback(True)
            context.lock_acquire_command_published = bool(
                getattr(lock_result, 'publish_succeeded', True)
            )
            context.lock_generation = int(
                getattr(lock_result, 'generation', 0)
            )
            if not context.lock_acquire_command_published:
                raise _FlowExit(
                    'abort',
                    context.stage,
                    'gait_lock_true_publish_failed',
                )
            if not self._update_guard_lock(context):
                raise _FlowExit(
                    'abort',
                    context.stage,
                    'cleanup_guard_lock_update_failed',
                )

            self._stage(context, 'publish_locomotion_zero')
            context.zero_allowed = True
            self._publish_zero(context, force=True)

            self._stage(context, 'wait_final_cmd_zero')
            self._wait_for_final_zero(context)

            self._stage(context, 'sdk_front_jump')
            self._run_helper(context)

            self._stage(context, 'post_settle')
            self._wait_post_settle(context)
            context.post_settle_completed = True

            self._stage(context, 'stability_unavailable')
            self._log(
                context,
                'stability_unavailable',
                stability_check='unavailable',
                physical_crossing_unverified=True,
            )

            self._stage(context, 'supervised_flow_done')
            terminal_state = 'succeed'
            terminal_stage = 'supervised_flow_done'
            terminal_reason = 'supervised_flow_completed'
        except _FlowExit as flow_exit:
            terminal_state = flow_exit.terminal_state
            terminal_stage = flow_exit.stage
            terminal_reason = flow_exit.reason
        except Exception as error:
            terminal_state = 'abort'
            terminal_stage = context.stage
            terminal_reason = 'unexpected_exception:{}'.format(
                type(error).__name__
            )
            self._log(
                context,
                'supervision_exception',
                error_type=type(error).__name__,
                error=str(error),
            )
        finally:
            context.feedback_enabled = False
            with self._condition:
                self._feedback_callback = None

            helper_clean = self._terminate_and_reap(context)
            final_zero_clean = True
            unlock_clean = False
            guard_cleanup_clean = self._update_guard_cleanup(
                context,
                helper_clean,
            )

            ros_cleanup_allowed = self._ros_cleanup_is_allowed()
            if context.lock_owned and ros_cleanup_allowed:
                context.zero_allowed = True
                try:
                    self._publish_zero(context, force=True)
                except Exception as error:
                    final_zero_clean = False
                    self._log(
                        context,
                        'final_zero_publish_failed',
                        error_type=type(error).__name__,
                        error=str(error),
                    )
                context.zero_allowed = False
                if (
                    context.lock_acquire_command_published
                    and helper_clean
                    and final_zero_clean
                    and guard_cleanup_clean
                    and not context.guard_update_failed
                ):
                    try:
                        lock_result = self._publish_lock_callback(False)
                        context.lock_release_command_published = bool(
                            getattr(
                                lock_result,
                                'publish_succeeded',
                                True,
                            )
                        )
                        unlock_clean = (
                            context.lock_release_command_published
                        )
                        context.lock_generation = int(
                            getattr(
                                lock_result,
                                'generation',
                                context.lock_generation,
                            )
                        )
                    except Exception as error:
                        self._log(
                            context,
                            'gait_lock_release_failed',
                            error_type=type(error).__name__,
                            error=str(error),
                        )
                    if context.lock_release_command_published:
                        context.lock_owned = False
                        guard_cleanup_clean = (
                            self._finish_guard_cleanup(context)
                        )
                        if not guard_cleanup_clean:
                            context.lock_owned = True
                            try:
                                lock_result = (
                                    self._publish_lock_callback(True)
                                )
                                context.lock_generation = int(
                                    getattr(
                                        lock_result,
                                        'generation',
                                        context.lock_generation,
                                    )
                                )
                            except Exception:
                                pass
            elif context.lock_owned:
                final_zero_clean = False
                self._log(
                    context,
                    'ros_cleanup_skipped_context_invalid',
                    cleanup_completed=False,
                )

            with self._condition:
                context.cleanup_completed = bool(
                    helper_clean
                    and final_zero_clean
                    and unlock_clean
                    and guard_cleanup_clean
                )
                if self._active_context is context:
                    self._active_context = None
                self._last_cleanup_context = context
                self.completed_contexts.append(context)
                context.completion_event.set()
                self._condition.notify_all()

            self._log(
                context,
                'cleanup_completed',
                cleanup_completed=context.cleanup_completed,
                terminal_state=terminal_state,
                terminal_stage=terminal_stage,
                terminal_reason=terminal_reason,
            )

        success = (
            terminal_state == 'succeed'
            and context.sdk_command_accepted
            and context.post_settle_completed
            and context.cleanup_completed
        )
        if terminal_state == 'succeed' and not success:
            terminal_state = 'abort'
            terminal_stage = 'finally_keep_zero_and_release_lock'
            terminal_reason = 'cleanup_incomplete'

        return FrontJumpOutcome(
            success=success,
            terminal_state=terminal_state,
            stage=terminal_stage,
            reason=terminal_reason,
            helper_started=context.helper_started,
            sdk_request_may_have_been_sent=(
                context.sdk_request_may_have_been_sent
            ),
            cleanup_completed=context.cleanup_completed,
            sdk_command_accepted=context.sdk_command_accepted,
            post_settle_completed=context.post_settle_completed,
            lock_acquire_command_published=(
                context.lock_acquire_command_published
            ),
            lock_release_command_published=(
                context.lock_release_command_published
            ),
            helper_group_empty=context.helper_group_empty,
        )

    def _update_guard_lock(self, context):
        if self.cleanup_guard is None or not context.guard_armed:
            return True

        def mutate(record):
            record['lock']['lock_acquire_command_published'] = bool(
                context.lock_acquire_command_published
            )
            record['lock']['generation'] = int(
                context.lock_generation
            )

        try:
            self.cleanup_guard.update(mutate)
            return True
        except Exception as error:
            context.guard_update_failed = True
            self._log(
                context,
                'cleanup_guard_lock_update_failed',
                error_type=type(error).__name__,
                error=str(error),
            )
            return False

    def _update_guard_cleanup(self, context, helper_clean):
        if self.cleanup_guard is None or not context.guard_armed:
            return True

        def mutate(record):
            cleanup = record['cleanup']
            cleanup['terminate_group_sent'] = bool(
                context.helper_terminate_sent
            )
            cleanup['kill_group_sent'] = bool(
                context.helper_kill_sent
            )
            cleanup['leader_reaped'] = bool(context.helper_reaped)
            cleanup['group_empty'] = bool(context.helper_group_empty)
            cleanup['cleanup_completed'] = False

        try:
            self.cleanup_guard.update(mutate)
            return bool(helper_clean)
        except Exception as error:
            context.guard_update_failed = True
            self._log(
                context,
                'cleanup_guard_cleanup_update_failed',
                error_type=type(error).__name__,
                error=str(error),
            )
            return False

    def _finish_guard_cleanup(self, context):
        if self.cleanup_guard is None or not context.guard_armed:
            return True

        def mutate(record):
            record['lock']['lock_release_command_published'] = True
            record['lock']['generation'] = int(
                context.lock_generation
            )
            record['cleanup']['cleanup_completed'] = True

        try:
            self.cleanup_guard.update(mutate)
            self.cleanup_guard.mark_clean_and_clear(
                context.guard_fault_id
            )
            context.guard_armed = False
            return True
        except Exception as error:
            context.guard_update_failed = True
            self._log(
                context,
                'cleanup_guard_finish_failed',
                error_type=type(error).__name__,
                error=str(error),
            )
            return False

    def _stage(self, context, stage):
        context.stage = stage
        progress = _FEEDBACK_PROGRESS[stage]
        context.event_history.append(('feedback', stage, progress))
        callback = None
        with self._condition:
            if context.feedback_enabled:
                callback = self._feedback_callback
        if callback is not None:
            callback(
                '{}: {}'.format(context.motion_name, stage),
                progress,
            )
        self._log(context, 'stage_entered', progress=progress)

    def _publish_zero(self, context, *, force=False):
        if (
            not context.zero_allowed
            or not self._ros_cleanup_is_allowed()
        ):
            return False
        now = self._clock()
        period = 1.0 / self.config.zero_publish_rate_hz
        if (
            not force
            and context.last_zero_time is not None
            and now - context.last_zero_time < period
        ):
            return False
        try:
            self._publish_zero_callback()
        except Exception as error:
            context.ros_publish_faults.append(
                {
                    'operation': 'publish_locomotion_zero',
                    'error_type': type(error).__name__,
                    'error': str(error),
                }
            )
            raise
        context.last_zero_time = now
        if context.first_zero_time is None:
            context.first_zero_time = now
        context.zero_publish_count += 1
        context.event_history.append(('zero', now))
        return True

    def _wait_for_final_zero(self, context):
        profile = context.profile
        window_start = self._clock()
        deadline = window_start + profile.final_zero_timeout
        with self._condition:
            self._final_samples.clear()
            context.final_window_start = window_start
            context.final_last_processed_sequence = self._final_sequence
            context.final_last_sample_time = None
            context.final_zero_streak = 0
            context.final_zero_required = (
                profile.final_zero_confirm_samples
            )

        while True:
            self._raise_if_interrupted(context, require_fresh_estop=False)
            now = self._clock()
            if now >= deadline:
                raise _FlowExit(
                    'abort',
                    context.stage,
                    self._final_zero_timeout_reason(now),
                )

            now = self._clock()
            final_zero_ready = self._final_zero_gate_is_ready(context, now)
            estop_ready = self._typed_estop_is_fresh_false(now)
            if final_zero_ready and estop_ready:
                self._log(
                    context,
                    'final_zero_confirmed',
                    final_zero_streak=context.final_zero_streak,
                    final_zero_required=profile.final_zero_confirm_samples,
                    final_cmd_software_only=True,
                )
                return

            self._ros_supervised_wait(
                context,
                min(0.05, max(0.0, deadline - now)),
            )

    def _final_zero_gate_is_ready(self, context, now):
        profile = context.profile
        with self._condition:
            new_samples = [
                sample
                for sample in self._final_samples
                if sample.sequence
                > context.final_last_processed_sequence
            ]

        for sample in new_samples:
            if (
                sample.sequence
                <= context.final_last_processed_sequence
            ):
                context.final_zero_streak = 0
                continue
            context.final_last_processed_sequence = sample.sequence
            context.last_final_sample = sample
            context.final_last_sample_time = sample.receive_time
            if sample.receive_time < context.final_window_start:
                context.final_zero_streak = 0
                continue
            if self._is_zero_sample(
                sample, profile.final_zero_epsilon
            ):
                context.final_zero_streak += 1
            else:
                context.final_zero_streak = 0

        if (
            context.final_last_sample_time is None
            or now - context.final_last_sample_time
            > self.config.final_cmd_stale_timeout
        ):
            context.final_zero_streak = 0

        pre_stop_elapsed = (
            context.first_zero_time is not None
            and now - context.first_zero_time
            >= profile.pre_stop_duration
        )
        return bool(
            context.final_zero_streak
            >= profile.final_zero_confirm_samples
            and pre_stop_elapsed
        )

    @staticmethod
    def _is_zero_sample(sample, epsilon):
        values = (
            sample.linear_x,
            sample.linear_y,
            sample.angular_z,
        )
        return all(
            math.isfinite(value) and abs(value) <= epsilon
            for value in values
        )

    def _final_zero_timeout_reason(self, now):
        with self._condition:
            estop = self._estop_sample
        if estop is None:
            return 'final_zero_timeout_estop_state_missing'
        if now - estop.receive_time > self.config.estop_state_stale_timeout:
            return 'final_zero_timeout_estop_state_stale'
        if estop.active:
            return 'estop_active'
        return 'final_zero_timeout'

    def _typed_estop_is_fresh_false(self, now):
        with self._condition:
            estop = self._estop_sample
        return bool(
            estop is not None
            and not estop.active
            and now - estop.receive_time
            <= self.config.estop_state_stale_timeout
        )

    def _raise_if_interrupted(self, context, *, require_fresh_estop):
        if context.cancel_requested.is_set():
            raise _FlowExit('canceled', context.stage, 'cancel_requested')
        if context.gait_stop_requested.is_set():
            raise _FlowExit(
                'abort',
                context.stage,
                context.stop_reason or 'gait_stop_requested',
            )

        now = self._clock()
        with self._condition:
            estop = self._estop_sample
        if estop is not None and estop.active:
            raise _FlowExit('abort', context.stage, 'estop_active')
        if require_fresh_estop:
            if estop is None:
                raise _FlowExit(
                    'abort', context.stage, 'estop_state_missing'
                )
            if (
                now - estop.receive_time
                > self.config.estop_state_stale_timeout
            ):
                raise _FlowExit(
                    'abort', context.stage, 'estop_state_stale'
                )

    def _run_helper(self, context):
        self._raise_if_interrupted(context, require_fresh_estop=True)
        try:
            executable = self._resolve_executable(
                self.config.sdk_action_executable
            )
        except FrontJumpConfigurationError as error:
            raise _FlowExit(
                'abort', context.stage, 'helper_resolution_failed:{}'.format(
                    str(error).replace(';', ',')
                )
            )
        if (
            not os.path.isabs(executable)
            or os.path.realpath(executable) != executable
        ):
            raise _FlowExit(
                'abort',
                context.stage,
                'helper_resolution_failed:absolute_normalized_path_required',
            )
        with self._condition:
            context.helper_path = executable

        if self.config.software_smoke_mode:
            if not is_test_only_smoke_helper(executable):
                raise _FlowExit(
                    'abort',
                    context.stage,
                    'software_smoke_helper_identity_rejected',
                )
        else:
            try:
                interface_index = self._interface_index(
                    self.config.sdk_network_interface
                )
                if (
                    isinstance(interface_index, bool)
                    or not isinstance(interface_index, int)
                    or interface_index <= 0
                ):
                    raise ValueError('network interface index is invalid')
            except (OSError, TypeError, ValueError):
                raise _FlowExit(
                    'abort', context.stage, 'sdk_network_interface_not_found'
                )

        argv = [
            executable,
            self.config.sdk_network_interface,
            'front_jump',
            '0',
        ]
        self._raise_if_interrupted(context, require_fresh_estop=True)
        if not self._final_zero_gate_is_ready(context, self._clock()):
            raise _FlowExit(
                'abort',
                context.stage,
                'final_cmd_zero_gate_lost_before_helper',
            )
        try:
            process = self._process_runner.start(argv)
        except Exception as error:
            process_started = bool(
                getattr(error, 'process_started', False)
            )
            start_cleanup_completed = bool(
                getattr(error, 'cleanup_completed', not process_started)
            )
            identity_unverified = bool(
                getattr(error, 'identity_unverified', False)
            )
            if process_started:
                with self._condition:
                    context.helper_started = True
                    context.sdk_request_may_have_been_sent = True
                    context.helper_reaped = start_cleanup_completed
                    context.helper_group_empty = start_cleanup_completed
            if process_started and not start_cleanup_completed:
                context.guard_update_failed = True
            self._log(
                context,
                'helper_start_failed',
                error_type=type(error).__name__,
                error=str(error),
                argv=argv,
                process_started=process_started,
                start_cleanup_completed=start_cleanup_completed,
                identity_unverified=identity_unverified,
                start_diagnostics=getattr(error, 'diagnostics', {}),
            )
            reason = 'helper_start_failed'
            if process_started and start_cleanup_completed:
                reason = 'helper_start_failed_after_process_start'
            elif process_started:
                reason = (
                    'helper_start_failed_process_identity_cleanup_unverified'
                )
            raise _FlowExit(
                'abort', context.stage, reason
            )

        with self._condition:
            context.helper_process = process
            context.helper_started = True
            context.helper_group_empty = False
            context.sdk_request_may_have_been_sent = True
            self._condition.notify_all()
        if self.cleanup_guard is not None and context.guard_armed:
            try:
                helper_identity = {
                    'resolved_executable': str(
                        getattr(process, 'executable', executable)
                    ),
                    'pid': int(process.pid),
                    'pgid': int(process.pgid),
                    'session_id': int(process.session_id),
                    'start_ticks': int(process.start_ticks),
                    'started': True,
                }

                def mutate(record):
                    record['helper'].update(helper_identity)

                self.cleanup_guard.update(mutate)
            except Exception as error:
                context.guard_update_failed = True
                self._log(
                    context,
                    'cleanup_guard_process_update_failed',
                    error_type=type(error).__name__,
                    error=str(error),
                )
                raise _FlowExit(
                    'abort',
                    context.stage,
                    'cleanup_guard_process_update_failed',
                )
        self._log(
            context,
            'helper_started',
            argv=argv,
            pid=getattr(process, 'pid', None),
            sdk_request_may_have_been_sent=True,
        )

        start_time = self._clock()
        deadline = start_time + context.profile.sdk_timeout
        while True:
            self._raise_if_interrupted(context, require_fresh_estop=True)
            now = self._clock()
            return_code = process.poll()
            if return_code is not None:
                process_result = self._reap_finished_process(
                    context, process
                )
                elapsed = self._clock() - start_time
                self._log(
                    context,
                    'helper_exited',
                    return_code=process_result.return_code,
                    stdout=process_result.stdout,
                    stderr=process_result.stderr,
                    elapsed_sec=elapsed,
                )
                if process_result.return_code != 0:
                    raise _FlowExit(
                        'abort',
                        context.stage,
                        'helper_return_code_nonzero',
                    )
                if not context.helper_group_empty:
                    raise _FlowExit(
                        'abort',
                        context.stage,
                        'helper_process_group_still_active',
                    )
                with self._condition:
                    context.sdk_command_accepted = True
                return
            if now >= deadline:
                raise _FlowExit('abort', context.stage, 'sdk_timeout')
            self._ros_supervised_wait(
                context,
                min(0.05, max(0.0, deadline - now)),
            )

    def _reap_finished_process(self, context, process):
        try:
            result = process.reap(timeout=0.0)
        except subprocess.TimeoutExpired:
            result = process.reap(timeout=0.2)
        group_empty = self._process_group_is_empty(process)
        with self._condition:
            context.helper_reaped = True
            context.helper_group_empty = group_empty
            context.helper_return_code = result.return_code
            context.helper_stdout = result.stdout
            context.helper_stderr = result.stderr
        return result

    @staticmethod
    def _process_group_is_empty(process):
        checker = getattr(process, 'group_empty', None)
        if checker is None:
            return process.poll() is not None
        return bool(checker())

    def _wait_post_settle(self, context):
        deadline = self._clock() + context.profile.post_settle_duration
        while True:
            self._raise_if_interrupted(context, require_fresh_estop=True)
            now = self._clock()
            if now >= deadline:
                return
            self._ros_supervised_wait(
                context,
                min(0.05, max(0.0, deadline - now)),
            )

    def _ros_supervised_wait(self, context, timeout):
        """Wait during live supervision, where zero output is still valid."""

        self._publish_zero(context)
        self._local_process_wait(timeout)

    def _local_process_wait(self, timeout):
        """Local-only bounded wait; safe after the ROS Context is invalid."""

        wait_time = max(0.0, float(timeout))
        if wait_time <= 0.0:
            return
        if self._waiter is not None:
            self._waiter(wait_time)
            return
        with self._condition:
            self._condition.wait(timeout=wait_time)

    @staticmethod
    def _record_local_cleanup_event(context, event, **fields):
        """Record cleanup diagnostics without touching ROS logging/output."""

        record = {'event': event}
        record.update(fields)
        context.local_cleanup_events.append(record)
        context.event_history.append(('local_cleanup', record))

    def _ros_cleanup_is_allowed(self):
        try:
            return bool(self._ros_cleanup_allowed_callback())
        except Exception:
            return False

    def _terminate_and_reap(self, context):
        with self._condition:
            process = context.helper_process
            helper_reaped = context.helper_reaped
        if process is None:
            with self._condition:
                if context.helper_started and not context.helper_group_empty:
                    return False
                context.helper_group_empty = True
            return True

        try:
            group_empty = self._process_group_is_empty(process)
            if helper_reaped and group_empty:
                with self._condition:
                    context.helper_group_empty = True
                return True

            if not group_empty:
                process.terminate()
                with self._condition:
                    context.helper_terminate_sent = True
                terminate_deadline = self._clock() + 0.5
                while (
                    not self._process_group_is_empty(process)
                    and self._clock() < terminate_deadline
                ):
                    process.poll()
                    self._local_process_wait(0.05)

            if not self._process_group_is_empty(process):
                process.kill()
                with self._condition:
                    context.helper_kill_sent = True
                kill_deadline = self._clock() + 0.5
                while (
                    not self._process_group_is_empty(process)
                    and self._clock() < kill_deadline
                ):
                    process.poll()
                    self._local_process_wait(0.05)

            if process.poll() is None:
                self._record_local_cleanup_event(
                    context,
                    'helper_reap_failed',
                    terminate_sent=context.helper_terminate_sent,
                    kill_sent=context.helper_kill_sent,
                )
                return False

            if helper_reaped:
                result = ProcessResult(
                    context.helper_return_code,
                    context.helper_stdout,
                    context.helper_stderr,
                )
            else:
                result = self._reap_finished_process(context, process)
            group_empty = self._process_group_is_empty(process)
            with self._condition:
                context.helper_group_empty = group_empty
            if not group_empty:
                self._record_local_cleanup_event(
                    context,
                    'helper_group_cleanup_failed',
                    pgid=getattr(process, 'pgid', None),
                )
                return False
            self._record_local_cleanup_event(
                context,
                'helper_reaped',
                return_code=result.return_code,
                stdout=result.stdout,
                stderr=result.stderr,
                terminate_sent=context.helper_terminate_sent,
                kill_sent=context.helper_kill_sent,
            )
            return True
        except Exception as error:
            self._record_local_cleanup_event(
                context,
                'helper_cleanup_exception',
                error_type=type(error).__name__,
                error=str(error),
            )
            return False

    def _log(self, context, event, **fields):
        record = {
            'event': event,
            'goal_id': context.goal_id,
            'motion_name': context.motion_name,
            'profile_name': context.profile.name,
            'supervision_stage': context.stage,
            'elapsed_sec': max(0.0, self._clock() - context.created_at),
            'gait_lock_requested': context.lock_owned,
            'lock_acquire_command_published': (
                context.lock_acquire_command_published
            ),
            'lock_release_command_published': (
                context.lock_release_command_published
            ),
            'locomotion_zero_published': context.zero_publish_count > 0,
            'final_zero_streak': context.final_zero_streak,
            'final_zero_required': context.final_zero_required,
            'sdk_process_started': context.helper_started,
            'sdk_helper_path': (
                context.helper_path
                or self.config.sdk_action_executable
            ),
            'sdk_network_interface': self.config.sdk_network_interface,
            'sdk_return_code': context.helper_return_code,
            'sdk_command_accepted': context.sdk_command_accepted,
            'sdk_request_may_have_been_sent': (
                context.sdk_request_may_have_been_sent
            ),
            'post_settle_completed': context.post_settle_completed,
            'physical_crossing_unverified': True,
            'cancel_requested': context.cancel_requested.is_set(),
            'gait_stop_requested': context.gait_stop_requested.is_set(),
            'cleanup_completed': context.cleanup_completed,
        }
        if context.last_final_sample is not None:
            record.update(
                {
                    'final_vx': context.last_final_sample.linear_x,
                    'final_vy': context.last_final_sample.linear_y,
                    'final_wz': context.last_final_sample.angular_z,
                }
            )
        with self._condition:
            estop = self._estop_sample
            mux_status = self._mux_status
        record['estop_active'] = (
            None if estop is None else estop.active
        )
        if mux_status is not None and isinstance(
            mux_status.get('parsed'), dict
        ):
            parsed = mux_status['parsed']
            record['cmd_mux_active_source'] = parsed.get('active_source')
            record['cmd_mux_reason'] = parsed.get('reason')
        record.update(fields)
        context.event_history.append(('log', record))
        if (
            self._event_logger is not None
            and self._ros_cleanup_is_allowed()
        ):
            try:
                self._event_logger(record)
            except Exception:
                pass
