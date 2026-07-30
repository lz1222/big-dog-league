"""
Types shared by the national integrated mission implementation.

The types in this module deliberately do not import ROS.  This keeps the
mission policy testable while :mod:`national_mission_node` owns all ROS I/O.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Optional


class MissionState(str, Enum):
    """Every observable state in the national integrated route."""

    WAIT_START = 'WAIT_START'
    START_LINE_FOLLOW = 'START_LINE_FOLLOW'
    START_WHITE_LINE_CONFIRM = 'START_WHITE_LINE_CONFIRM'
    START_JUMP = 'START_JUMP'
    START_LINE_REACQUIRE = 'START_LINE_REACQUIRE'
    MAZE_ENTRY_APPROACH = 'MAZE_ENTRY_APPROACH'
    MAZE_TRAVERSE_FAKE = 'MAZE_TRAVERSE_FAKE'
    MAZE_EXIT_REACQUIRE = 'MAZE_EXIT_REACQUIRE'
    STAIRS_APPROACH = 'STAIRS_APPROACH'
    STAIRS_TRAVERSE_FAKE = 'STAIRS_TRAVERSE_FAKE'
    STAIRS_EXIT_REACQUIRE = 'STAIRS_EXIT_REACQUIRE'
    PICK_ARC_APPROACH = 'PICK_ARC_APPROACH'
    PICK_ARC_APEX_STOP = 'PICK_ARC_APEX_STOP'
    PICK_TURN_LEFT = 'PICK_TURN_LEFT'
    PICK_PATTERN_SEARCH = 'PICK_PATTERN_SEARCH'
    PICK_PATTERN_CONFIRM = 'PICK_PATTERN_CONFIRM'
    ARM_PICK_FAKE = 'ARM_PICK_FAKE'
    PICK_TURN_RIGHT = 'PICK_TURN_RIGHT'
    PICK_LINE_REACQUIRE = 'PICK_LINE_REACQUIRE'
    TRANSFER_PLATFORM_APPROACH = 'TRANSFER_PLATFORM_APPROACH'
    TRANSFER_STOP = 'TRANSFER_STOP'
    ARM_TRANSFER_PLACE_FAKE = 'ARM_TRANSFER_PLACE_FAKE'
    ARM_TRANSFER_PICK_FAKE = 'ARM_TRANSFER_PICK_FAKE'
    TRANSFER_LINE_REACQUIRE = 'TRANSFER_LINE_REACQUIRE'
    INSPECTION_APPROACH = 'INSPECTION_APPROACH'
    RED_CIRCLE_TRACK = 'RED_CIRCLE_TRACK'
    RED_CIRCLE_POST_OFFSET = 'RED_CIRCLE_POST_OFFSET'
    INSPECTION_STOP = 'INSPECTION_STOP'
    INSPECTION_TURN_LEFT = 'INSPECTION_TURN_LEFT'
    INSPECTION_SIGN_CONFIRM = 'INSPECTION_SIGN_CONFIRM'
    INSPECTION_ACTION = 'INSPECTION_ACTION'
    INSPECTION_TURN_RIGHT = 'INSPECTION_TURN_RIGHT'
    INSPECTION_LINE_REACQUIRE = 'INSPECTION_LINE_REACQUIRE'
    PLACE_PLATFORM_APPROACH = 'PLACE_PLATFORM_APPROACH'
    PLACE_PLATFORM_STOP = 'PLACE_PLATFORM_STOP'
    ARM_PLACE_SELECTED_FAKE = 'ARM_PLACE_SELECTED_FAKE'
    RETURN_LINE_FOLLOW = 'RETURN_LINE_FOLLOW'
    FINISH_WHITE_LINE_CONFIRM = 'FINISH_WHITE_LINE_CONFIRM'
    FINISH_JUMP = 'FINISH_JUMP'
    FINISH_LINE_REACQUIRE = 'FINISH_LINE_REACQUIRE'
    FINAL_ZONE_APPROACH = 'FINAL_ZONE_APPROACH'
    FINAL_STOP = 'FINAL_STOP'
    MISSION_COMPLETE = 'MISSION_COMPLETE'
    PAUSED = 'PAUSED'
    SAFE_STOP = 'SAFE_STOP'
    RECOVERY = 'RECOVERY'
    ESTOP = 'ESTOP'
    MISSION_FAILED = 'MISSION_FAILED'


class MissionFailureCode(str, Enum):
    """Fail-closed result categories exposed in state and timeline logs."""

    NONE = 'NONE'
    ACTION_FAILED = 'ACTION_FAILED'
    ACTION_TIMEOUT = 'ACTION_TIMEOUT'
    ACTION_UNAVAILABLE = 'ACTION_UNAVAILABLE'
    DETECTION_TIMEOUT = 'DETECTION_TIMEOUT'
    LINE_REACQUIRE_TIMEOUT = 'LINE_REACQUIRE_TIMEOUT'
    POSITION_GATE_TIMEOUT = 'POSITION_GATE_TIMEOUT'
    POSITION_GATE_EXCEEDED = 'POSITION_GATE_EXCEEDED'
    FINAL_COMMAND_STALE = 'FINAL_COMMAND_STALE'
    INVALID_FINAL_COMMAND = 'INVALID_FINAL_COMMAND'
    INVALID_TARGET_PLATFORM = 'INVALID_TARGET_PLATFORM'
    ESTOP_ACTIVE = 'ESTOP_ACTIVE'
    EXTERNAL_STOP = 'EXTERNAL_STOP'
    STATE_TIMEOUT = 'STATE_TIMEOUT'
    INVALID_TRANSITION = 'INVALID_TRANSITION'
    DUPLICATE_ACTION = 'DUPLICATE_ACTION'


class MissionEvent(str, Enum):
    """Input event names used by the ROS node and simulation recorder."""

    START = 'START'
    STOP = 'STOP'
    PAUSE = 'PAUSE'
    RESUME = 'RESUME'
    RESET = 'RESET'
    LINE_TRACK = 'LINE_TRACK'
    WHITE_LINE = 'WHITE_LINE'
    RED_CIRCLE = 'RED_CIRCLE'
    PICK_MARKER = 'PICK_MARKER'
    INSPECTION_SIGN = 'INSPECTION_SIGN'
    ACTION_RESULT = 'ACTION_RESULT'
    FINAL_COMMAND = 'FINAL_COMMAND'
    ESTOP = 'ESTOP'


class StationSide(str, Enum):
    LEFT = 'left'
    RIGHT = 'right'


class InspectionType(str, Enum):
    ELECTRIC_SHOCK = 'electric_shock'
    OXIDIZER = 'oxidizer'
    RADIATION = 'radiation'


@dataclass(frozen=True)
class TaskResult:
    """A normalized asynchronous result from a locomotion or arm adapter."""

    token: str
    success: bool
    message: str = ''
    task_name: str = ''
    physical_crossing_unverified: bool = False


@dataclass(frozen=True)
class MotionCommand:
    """ROS-independent planar command published as a mission candidate."""

    vx: float = 0.0
    wz: float = 0.0


@dataclass(frozen=True)
class TransitionRecord:
    """One transition used for timeline output and post-run diagnosis."""

    timestamp: float
    previous: Optional[MissionState]
    current: MissionState
    reason: str
