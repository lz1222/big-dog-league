"""Stable adapter boundary between the national FSM and ROS/hardware details."""

from abc import ABC, abstractmethod
from dataclasses import dataclass

from .mission_types import MotionCommand, TaskResult


@dataclass(frozen=True)
class ActionRequest:
    token: str
    adapter: str
    task_name: str
    target: str = ''


class LineControlAdapter(ABC):
    """Own the existing line follower's start/stop compatibility interface."""

    @abstractmethod
    def set_line_enabled(self, enabled: bool) -> None:
        raise NotImplementedError


class LocomotionActionAdapter(ABC):
    """Execute the existing ExecuteMotion schema without exposing it to FSM."""

    @abstractmethod
    def execute_motion(self, request: ActionRequest) -> None:
        raise NotImplementedError


class ArmTaskAdapter(ABC):
    """Execute the existing ExecuteArmTask schema without exposing it to FSM."""

    @abstractmethod
    def execute_arm_task(self, request: ActionRequest) -> None:
        raise NotImplementedError


class MazePlaceholderAdapter(ABC):
    """Asynchronous replaceable maze traversal boundary."""

    @abstractmethod
    def execute_maze_placeholder(self, request: ActionRequest) -> None:
        raise NotImplementedError


class MarkerRecognitionAdapter(ABC):
    """Marker input is pushed into the FSM by its ROS implementation."""

    @abstractmethod
    def latest_marker_result(self) -> TaskResult:
        raise NotImplementedError


class RouteMarkerAdapter(ABC):
    """Route-marker input is pushed into the FSM by its ROS implementation."""

    @abstractmethod
    def latest_route_marker_result(self) -> TaskResult:
        raise NotImplementedError


class MissionAdapter(
    LineControlAdapter,
    LocomotionActionAdapter,
    ArmTaskAdapter,
    MazePlaceholderAdapter,
):
    """Concrete bridge contract consumed by :class:`NationalMissionFSM`."""

    @abstractmethod
    def publish_mission_command(self, command: MotionCommand) -> None:
        raise NotImplementedError

    @abstractmethod
    def release_mission_command(self) -> None:
        """Stop refreshing mission candidate so a line candidate may win mux."""
        raise NotImplementedError

    @abstractmethod
    def emit_event(self, label: str, detail: str = '') -> None:
        raise NotImplementedError

    def dispatch(self, request: ActionRequest) -> None:
        if request.adapter == 'locomotion':
            self.execute_motion(request)
        elif request.adapter == 'arm':
            self.execute_arm_task(request)
        elif request.adapter == 'maze':
            self.execute_maze_placeholder(request)
        else:
            raise ValueError('unknown mission adapter: {}'.format(request.adapter))
