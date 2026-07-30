"""Provide ROS-independent command arbitration safety logic."""

from dataclasses import dataclass
import math
from typing import Dict, Optional


@dataclass(frozen=True)
class VelocityCommand:
    """Represent one planar velocity command."""

    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0


@dataclass(frozen=True)
class CommandMuxDecision:
    """Represent one arbitration decision and its diagnostic state."""

    command: VelocityCommand
    active_source: str
    reason: str
    status: Dict[str, object]


@dataclass
class _SourceState:
    command: Optional[VelocityCommand] = None
    received_at: Optional[float] = None
    valid: bool = False
    fresh: bool = False
    age_sec: Optional[float] = None
    clamped: bool = False


class CommandMuxCore:
    """Arbitrate velocity commands without depending on ROS."""

    _SOURCES = ('line', 'mission', 'locomotion')

    def __init__(
        self,
        line_cmd_timeout_sec=0.5,
        mission_cmd_timeout_sec=0.5,
        locomotion_cmd_timeout_sec=0.3,
        max_linear_x=0.60,
        max_linear_y=0.15,
        max_angular_z=1.30,
    ):
        """Initialize validated timeouts and symmetric velocity limits."""
        self._timeouts = {
            'line': self._positive_finite(
                'line_cmd_timeout_sec', line_cmd_timeout_sec
            ),
            'mission': self._positive_finite(
                'mission_cmd_timeout_sec', mission_cmd_timeout_sec
            ),
            'locomotion': self._positive_finite(
                'locomotion_cmd_timeout_sec', locomotion_cmd_timeout_sec
            ),
        }
        self._limits = {
            'linear_x': self._positive_finite('max_linear_x', max_linear_x),
            'linear_y': self._positive_finite('max_linear_y', max_linear_y),
            'angular_z': self._positive_finite(
                'max_angular_z', max_angular_z
            ),
        }
        self._sources = {
            source: _SourceState() for source in self._SOURCES
        }
        self.estop = False
        self.arm_lock = False
        self.gait_lock = False
        self.invalid_command_count = 0
        self._last_time = None
        self._pending_zero_reason = None

    @staticmethod
    def _positive_finite(name, value):
        if isinstance(value, bool):
            raise ValueError('{} must be a finite number greater than 0'.format(name))
        try:
            numeric_value = float(value)
        except (TypeError, ValueError):
            raise ValueError(
                '{} must be a finite number greater than 0'.format(name)
            )
        if not math.isfinite(numeric_value) or numeric_value <= 0.0:
            raise ValueError('{} must be a finite number greater than 0'.format(name))
        return numeric_value

    @staticmethod
    def _finite_time(now):
        if isinstance(now, bool):
            raise ValueError('now must be finite')
        try:
            numeric_time = float(now)
        except (TypeError, ValueError):
            raise ValueError('now must be finite')
        if not math.isfinite(numeric_time):
            raise ValueError('now must be finite')
        return numeric_time

    def _observe_time(self, now):
        moved_backwards = self._last_time is not None and now < self._last_time
        if moved_backwards:
            self._invalidate_all(clear_timestamps=True)
            self._pending_zero_reason = 'time_moved_backwards'
        self._last_time = now
        return moved_backwards

    def _invalidate_all(self, clear_timestamps=False):
        for state in self._sources.values():
            state.valid = False
            state.fresh = False
            state.age_sec = None
            state.clamped = False
            if clear_timestamps:
                state.command = None
                state.received_at = None

    @staticmethod
    def _clamp(value, limit):
        return max(-limit, min(limit, value))

    def update_command(self, source, command, now):
        """Store a finite, safely clamped command for one known source."""
        if source not in self._sources:
            raise ValueError('unknown command source: {}'.format(source))

        timestamp = self._finite_time(now)
        moved_backwards = self._observe_time(timestamp)
        state = self._sources[source]
        if moved_backwards:
            return False

        try:
            linear_x = float(command.linear_x)
            linear_y = float(command.linear_y)
            angular_z = float(command.angular_z)
        except (AttributeError, TypeError, ValueError):
            self._reject_all_commands(state, timestamp)
            return False

        values = (linear_x, linear_y, angular_z)
        if not all(math.isfinite(value) for value in values):
            self._reject_all_commands(state, timestamp)
            return False

        safe_linear_x = self._clamp(linear_x, self._limits['linear_x'])
        safe_linear_y = self._clamp(linear_y, self._limits['linear_y'])
        safe_angular_z = self._clamp(angular_z, self._limits['angular_z'])
        state.command = VelocityCommand(
            linear_x=safe_linear_x,
            linear_y=safe_linear_y,
            angular_z=safe_angular_z,
        )
        state.received_at = timestamp
        state.valid = True
        state.fresh = True
        state.age_sec = 0.0
        state.clamped = (
            safe_linear_x != linear_x
            or safe_linear_y != linear_y
            or safe_angular_z != angular_z
        )
        return True

    def _reject_all_commands(self, state, timestamp):
        self._invalidate_all(clear_timestamps=True)
        state.command = None
        state.received_at = timestamp
        state.valid = False
        state.fresh = False
        state.age_sec = 0.0
        state.clamped = False
        self.invalid_command_count += 1
        self._pending_zero_reason = 'invalid_command_received'

    def update_line_command(self, command, now):
        """Store a line-following command."""
        return self.update_command('line', command, now)

    def update_mission_command(self, command, now):
        """Store a mission command."""
        return self.update_command('mission', command, now)

    def update_locomotion_command(self, command, now):
        """Store a locomotion command."""
        return self.update_command('locomotion', command, now)

    def set_estop(self, enabled, now=None):
        """Set emergency stop and invalidate commands on either transition."""
        return self._set_lock_state(
            'estop',
            enabled,
            now,
            invalidate_on_any_transition=True,
            clear_timestamps_on_invalidate=True,
        )

    def set_arm_lock(self, enabled, now=None):
        """Set the arm lock state and invalidate commands on release."""
        return self._set_lock_state('arm_lock', enabled, now)

    def set_gait_lock(self, enabled, now=None):
        """Set the gait lock state and invalidate commands on release."""
        return self._set_lock_state('gait_lock', enabled, now)

    def _set_lock_state(
        self,
        attribute,
        enabled,
        now,
        invalidate_on_any_transition=False,
        clear_timestamps_on_invalidate=False,
    ):
        if now is not None:
            timestamp = self._finite_time(now)
            self._observe_time(timestamp)
        previous = getattr(self, attribute)
        current = bool(enabled)
        setattr(self, attribute, current)
        changed = previous != current
        if changed and (invalidate_on_any_transition or previous):
            self._invalidate_all(
                clear_timestamps=clear_timestamps_on_invalidate
            )
        return changed

    def _refresh_sources(self, now):
        for source, state in self._sources.items():
            if state.received_at is None:
                state.age_sec = None
                state.fresh = False
                continue
            age_sec = now - state.received_at
            if not math.isfinite(age_sec) or age_sec < 0.0:
                state.age_sec = None
                state.fresh = False
                state.valid = False
                continue
            state.age_sec = age_sec
            state.fresh = state.valid and age_sec <= self._timeouts[source]

    def evaluate(self, now):
        """Select the safe output command at the supplied time."""
        timestamp = self._finite_time(now)
        self._observe_time(timestamp)
        self._refresh_sources(timestamp)
        pending_zero_reason = self._pending_zero_reason
        self._pending_zero_reason = None

        if self.estop:
            return self._make_decision(
                VelocityCommand(), 'estop', 'emergency_stop_active', None
            )
        if self.arm_lock:
            return self._make_decision(
                VelocityCommand(), 'arm_lock', 'arm_control_lock_active', None
            )
        if pending_zero_reason is not None:
            return self._make_decision(
                VelocityCommand(), 'none', pending_zero_reason, None
            )
        if self.gait_lock:
            locomotion = self._sources['locomotion']
            if locomotion.fresh and locomotion.command is not None:
                return self._make_decision(
                    locomotion.command,
                    'locomotion',
                    'fresh_locomotion_command',
                    locomotion,
                )
            return self._make_decision(
                VelocityCommand(),
                'gait_lock_stale',
                'gait_lock_without_fresh_locomotion',
                None,
            )

        mission = self._sources['mission']
        if mission.fresh and mission.command is not None:
            return self._make_decision(
                mission.command,
                'mission',
                'fresh_mission_command',
                mission,
            )

        line = self._sources['line']
        if line.fresh and line.command is not None:
            return self._make_decision(
                line.command,
                'line',
                'fresh_line_command',
                line,
            )

        return self._make_decision(
            VelocityCommand(), 'none', 'no_fresh_command', None
        )

    def _make_decision(self, command, active_source, reason, source_state):
        clamped = source_state.clamped if source_state is not None else False
        status = {
            'active_source': active_source,
            'reason': reason,
            'estop': bool(self.estop),
            'arm_lock': bool(self.arm_lock),
            'gait_lock': bool(self.gait_lock),
            'line_fresh': bool(self._sources['line'].fresh),
            'mission_fresh': bool(self._sources['mission'].fresh),
            'locomotion_fresh': bool(self._sources['locomotion'].fresh),
            'line_age_sec': self._sources['line'].age_sec,
            'mission_age_sec': self._sources['mission'].age_sec,
            'locomotion_age_sec': self._sources['locomotion'].age_sec,
            'final_vx': command.linear_x,
            'final_vy': command.linear_y,
            'final_wz': command.angular_z,
            'clamped': bool(clamped),
            'invalid_command_count': int(self.invalid_command_count),
        }
        return CommandMuxDecision(
            command=command,
            active_source=active_source,
            reason=reason,
            status=status,
        )
