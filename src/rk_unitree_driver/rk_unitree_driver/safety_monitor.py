#!/usr/bin/env python3

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class CommandDecision:
    """Safety decision for one incoming cmd_vel sample."""

    should_stop: bool
    reason: str
    vx: float = 0.0
    vyaw: float = 0.0


class SafetyMonitor:
    """Validate cmd_vel values before they reach Go2 Sport control."""

    def __init__(self, max_linear_x, max_angular_z):
        self.update_limits(max_linear_x, max_angular_z)

    def update_limits(self, max_linear_x, max_angular_z):
        self.max_linear_x = float(max_linear_x)
        self.max_angular_z = float(max_angular_z)

    def evaluate(self, linear_x, angular_z):
        vx = float(linear_x)
        vyaw = float(angular_z)

        if not math.isfinite(vx) or not math.isfinite(vyaw):
            return CommandDecision(
                should_stop=True,
                reason=(
                    'invalid cmd_vel value: '
                    f'linear.x={vx}, angular.z={vyaw}'
                ),
            )

        if abs(vx) > self.max_linear_x:
            return CommandDecision(
                should_stop=True,
                reason=(
                    'linear.x exceeds limit: '
                    f'{vx:.3f} > +/-{self.max_linear_x:.3f}'
                ),
            )

        if abs(vyaw) > self.max_angular_z:
            return CommandDecision(
                should_stop=True,
                reason=(
                    'angular.z exceeds limit: '
                    f'{vyaw:.3f} > +/-{self.max_angular_z:.3f}'
                ),
            )

        if vx == 0.0 and vyaw == 0.0:
            return CommandDecision(
                should_stop=True,
                reason='zero velocity command',
            )

        return CommandDecision(
            should_stop=False,
            reason='ok',
            vx=vx,
            vyaw=vyaw,
        )

    @staticmethod
    def is_positive_finite(value):
        if isinstance(value, bool):
            return False

        try:
            number = float(value)
        except (TypeError, ValueError):
            return False

        return math.isfinite(number) and number > 0.0
