#!/usr/bin/env python3

"""B2 迷宫导航纯策略核心。

只计算诊断速度，不依赖 ROS 或硬件接口。
"""

from dataclasses import dataclass
import math


SECTOR_NAMES = (
    'front',
    'left_front',
    'right_front',
    'left',
    'right',
)

DIRECTION_LEFT = 'LEFT'
DIRECTION_RIGHT = 'RIGHT'
VALID_DIRECTIONS = {DIRECTION_LEFT, DIRECTION_RIGHT}

STATE_WAIT_SENSOR = 'WAIT_SENSOR'
STATE_CORRIDOR_FOLLOW = 'CORRIDOR_FOLLOW'
STATE_CORNER_APPROACH = 'CORNER_APPROACH'
STATE_TURN_LEFT = 'TURN_LEFT'
STATE_TURN_RIGHT = 'TURN_RIGHT'
STATE_TURN_FINE_ALIGN = 'TURN_FINE_ALIGN'
STATE_CORRIDOR_REACQUIRE = 'CORRIDOR_REACQUIRE'
STATE_REVERSE_RECOVERY = 'REVERSE_RECOVERY'
STATE_FINISHED = 'FINISHED'
STATE_FAULT_STOP = 'FAULT_STOP'

TERMINAL_STATES = {
    STATE_FINISHED,
    STATE_FAULT_STOP,
}


@dataclass(frozen=True)
class MazeObservation:
    """由 B1 JSON 转换得到的一帧五扇区与连续航向观测。"""

    sensor_state: str
    cloud_age_sec: float
    odom_age_sec: float
    yaw_rad: float
    turn_rad: float
    distances: dict
    # None 兼容旧录包；真机 B1 必须提供递增序号以防周期快照重复计帧。
    cloud_sequence: int = None


@dataclass(frozen=True)
class MazePolicyConfig:
    """迷宫几何、诊断速度、阈值和持续帧配置。"""

    robot_length_m: float = 0.70
    robot_width_m: float = 0.31
    robot_height_m: float = 0.40
    wall_height_m: float = 0.45
    corridor_width_m: float = 0.57
    footprint_safety_margin_m: float = 0.03

    front_max_range_m: float = 3.0
    side_max_range_m: float = 2.0
    max_cloud_age_sec: float = 0.50
    max_odom_age_sec: float = 0.20

    sensor_confirm_frames: int = 3
    side_missing_confirm_frames: int = 2
    side_unsafe_confirm_frames: int = 2
    corner_confirm_frames: int = 3
    turn_start_confirm_frames: int = 3
    turn_confirm_frames: int = 3
    reacquire_confirm_frames: int = 5
    recovery_confirm_frames: int = 3
    exit_confirm_frames: int = 5

    corridor_vx: float = 0.20
    approach_vx: float = 0.08
    turn_forward_vx: float = 0.08
    fine_align_vx: float = 0.04
    reacquire_vx: float = 0.08
    reverse_vx: float = 0.05
    max_center_wz: float = 0.25
    max_turn_wz: float = 0.55
    min_turn_wz: float = 0.16
    max_fine_wz: float = 0.22

    center_kp: float = 1.20
    turn_kp: float = 1.10
    fine_turn_kp: float = 0.90
    # Round10 真机居中静态段的左右侧距中位数约为0.25m；该值是
    # B1有效墙面回波的标定目标，不等同于几何通道宽度的一半。
    side_target_m: float = 0.25
    center_tolerance_m: float = 0.07

    front_slow_distance_m: float = 1.00
    corner_approach_distance_m: float = 0.98
    turn_start_distance_m: float = 0.62
    front_emergency_distance_m: float = 0.43
    recovery_front_clear_m: float = 0.72
    turn_open_distance_m: float = 0.50
    turn_open_hysteresis_m: float = 0.06
    exit_front_clear_m: float = 1.00
    exit_side_open_m: float = 0.50

    turn_angle_deg: float = 90.0
    fine_align_enter_deg: float = 12.0
    fine_align_exit_deg: float = 18.0
    turn_tolerance_deg: float = 4.0
    reacquire_yaw_tolerance_deg: float = 7.0

    corner_timeout_sec: float = 6.0
    turn_timeout_sec: float = 8.0
    reacquire_timeout_sec: float = 5.0
    reverse_timeout_sec: float = 1.0


class MazeNavigationPolicy:
    """按固定转向拓扑和实时雷达余量生成只读迷宫策略。"""

    def __init__(self, config, route_directions):
        self.config = config
        self.route_directions = tuple(
            str(direction).upper()
            for direction in route_directions
        )
        self._validate()

        self.half_length_m = 0.5 * self.config.robot_length_m
        self.half_width_m = 0.5 * self.config.robot_width_m
        self.sweep_radius_m = math.hypot(
            self.half_length_m,
            self.half_width_m,
        )
        self.sweep_diameter_with_margin_m = 2.0 * (
            self.sweep_radius_m
            + self.config.footprint_safety_margin_m
        )
        self.side_collision_distance_m = (
            self.half_width_m
            + self.config.footprint_safety_margin_m
        )
        self.front_collision_distance_m = (
            self.half_length_m
            + self.config.footprint_safety_margin_m
        )
        self.in_place_rotation_fits_corridor = (
            self.sweep_diameter_with_margin_m
            <= self.config.corridor_width_m
        )

        self.state = STATE_WAIT_SENSOR
        self.reason = 'startup_wait_sensor'
        self.route_index = 0
        self.desired_vx = 0.0
        self.desired_wz = 0.0

        self._state_enter_time = None
        self._sensor_ready_latched = False
        self._sensor_streak = 0
        self._side_missing_streak = 0
        self._side_unsafe_streak = 0
        self._corner_streak = 0
        self._turn_start_streak = 0
        self._turn_streak = 0
        self._reacquire_streak = 0
        self._recovery_streak = 0
        self._exit_streak = 0
        self._turn_start_rad = None
        self._turn_target_rad = None
        self._turn_start_time = None
        # 拐角低挡板会因扫描相位产生数厘米侧距波动；
        # 只有先越过严格入口门限后才允许用退出门限继续确认。
        self._turn_open_latched = False
        self._last_observation = None
        self._last_cloud_sequence = None

    def update(self, observation, now_sec):
        """使用一帧新 B1 观测推进状态机并返回 JSON 快照。"""
        now = self._finite_now(now_sec)
        self._last_observation = observation

        if self.state in TERMINAL_STATES:
            self._set_command(0.0, 0.0)
            return self.snapshot(now)

        fresh, freshness_reason = self.observation_freshness(observation)
        if not fresh:
            if self._sensor_ready_latched:
                self._enter_fault(freshness_reason, now)
            else:
                self.state = STATE_WAIT_SENSOR
                self.reason = freshness_reason
                self._set_command(0.0, 0.0)
                self._sensor_streak = 0
            return self.snapshot(now)

        if not self._accept_new_cloud_sequence(observation.cloud_sequence):
            # 同一雷达帧的周期快照只更新显示值，不推进任何持续帧状态。
            return self.snapshot(now)

        if self.state == STATE_WAIT_SENSOR:
            return self._handle_wait_sensor(observation, now)
        if self.state == STATE_CORRIDOR_FOLLOW:
            return self._handle_corridor_follow(observation, now)
        if self.state == STATE_CORNER_APPROACH:
            return self._handle_corner_approach(observation, now)
        if self.state in (STATE_TURN_LEFT, STATE_TURN_RIGHT):
            return self._handle_turn(observation, now)
        if self.state == STATE_TURN_FINE_ALIGN:
            return self._handle_fine_align(observation, now)
        if self.state == STATE_CORRIDOR_REACQUIRE:
            return self._handle_reacquire(observation, now)
        if self.state == STATE_REVERSE_RECOVERY:
            return self._handle_reverse_recovery(observation, now)

        self._enter_fault(f'unhandled_state_{self.state}', now)
        return self.snapshot(now)

    def mark_input_stale(self, reason, now_sec):
        """B2 输入 Topic 断流时立即清零诊断速度并进入保护状态。"""
        now = self._finite_now(now_sec)
        if self.state in TERMINAL_STATES:
            # 保持完成或故障终态，避免输入停止后覆盖最终结果。
            self._set_command(0.0, 0.0)
            return self.snapshot(now)
        if self._sensor_ready_latched:
            self._enter_fault(str(reason), now)
        else:
            self.state = STATE_WAIT_SENSOR
            self.reason = str(reason)
            self._set_command(0.0, 0.0)
        return self.snapshot(now)

    def snapshot(self, now_sec):
        """返回当前策略快照；该方法本身不推进持续帧计数。"""
        now = self._finite_now(now_sec)
        observation = self._last_observation
        distances = (
            dict(observation.distances)
            if observation is not None
            else {name: None for name in SECTOR_NAMES}
        )
        expected_turn = self.expected_turn()
        turn_value = (
            observation.turn_rad
            if observation is not None
            else None
        )
        turn_error = self._turn_error(turn_value)
        turn_progress = self._turn_progress(turn_value)
        center_error, center_reference = self._center_error_for_state(
            distances,
            expected_turn,
        )
        turn_start_sweep_safe = (
            self.turn_start_sweep_safe(expected_turn, distances)
            if expected_turn is not None
            else None
        )
        active_turn_clearance_safe = (
            self.active_turn_clearance_safe(
                expected_turn,
                distances,
                require_full_sweep=self.state in (
                    STATE_TURN_LEFT,
                    STATE_TURN_RIGHT,
                ),
            )
            if expected_turn is not None
            else None
        )
        if self.state in (
            STATE_TURN_LEFT,
            STATE_TURN_RIGHT,
            STATE_TURN_FINE_ALIGN,
        ):
            moving_sweep_safe = active_turn_clearance_safe
        else:
            moving_sweep_safe = turn_start_sweep_safe

        return {
            'dry_run': True,
            'motion_output': False,
            'state': self.state,
            'reason': self.reason,
            'desired_vx': self.desired_vx,
            'desired_wz': self.desired_wz,
            'route_index': self.route_index,
            'route_total': len(self.route_directions),
            'route_complete': self.route_complete(),
            'side_missing_streak': self._side_missing_streak,
            'side_unsafe_streak': self._side_unsafe_streak,
            'turn_start_streak': self._turn_start_streak,
            'turn_open_latched': self._turn_open_latched,
            'cloud_sequence': (
                observation.cloud_sequence
                if observation is not None
                else None
            ),
            'expected_turn': expected_turn,
            'turn_target_rad': self._turn_target_rad,
            'turn_target_deg': self._degrees_or_none(
                self._turn_target_rad
            ),
            'turn_error_rad': turn_error,
            'turn_error_deg': self._degrees_or_none(turn_error),
            'turn_tolerance_deg': self.config.turn_tolerance_deg,
            'fine_align_enter_deg': self.config.fine_align_enter_deg,
            # 本次进度以进入 TURN 状态时的值为零点，不受此前长时间
            # 静止累计漂移的绝对数值影响，供闭环诊断和终端 D 显示。
            'turn_progress_rad': turn_progress,
            'turn_progress_deg': self._degrees_or_none(turn_progress),
            'yaw_rad': (
                observation.yaw_rad
                if observation is not None
                else None
            ),
            'turn_rad': (
                observation.turn_rad
                if observation is not None
                else None
            ),
            'cloud_age_sec': (
                observation.cloud_age_sec
                if observation is not None
                else None
            ),
            'odom_age_sec': (
                observation.odom_age_sec
                if observation is not None
                else None
            ),
            'distances_m': distances,
            'center_error_m': center_error,
            'center_reference': center_reference,
            'sweep_radius_m': self.sweep_radius_m,
            'sweep_diameter_with_margin_m': (
                self.sweep_diameter_with_margin_m
            ),
            'in_place_rotation_fits_corridor': (
                self.in_place_rotation_fits_corridor
            ),
            'moving_turn_sweep_safe': moving_sweep_safe,
            'turn_start_sweep_safe': turn_start_sweep_safe,
            'active_turn_clearance_safe': active_turn_clearance_safe,
            'active_turn_required_side_clearance_m': (
                self.active_turn_required_side_clearance_m(
                    require_full_sweep=self.state in (
                        STATE_TURN_LEFT,
                        STATE_TURN_RIGHT,
                    ),
                )
                if expected_turn is not None
                else None
            ),
            'reverse_rear_visibility_confirmed': False,
            'geometry': {
                'robot_length_m': self.config.robot_length_m,
                'robot_width_m': self.config.robot_width_m,
                'robot_height_m': self.config.robot_height_m,
                'wall_height_m': self.config.wall_height_m,
                'corridor_width_m': self.config.corridor_width_m,
                'footprint_safety_margin_m': (
                    self.config.footprint_safety_margin_m
                ),
            },
            'state_age_sec': self._state_age(now),
        }

    def observation_freshness(self, observation):
        """校验 B1 状态、消息 age、Yaw 和五扇区数据。"""
        if not isinstance(observation, MazeObservation):
            return False, 'observation_type_invalid'
        sequence = observation.cloud_sequence
        if sequence is not None:
            if (
                isinstance(sequence, bool)
                or not isinstance(sequence, int)
                or sequence < 0
            ):
                return False, 'cloud_sequence_invalid'
            if (
                self._last_cloud_sequence is not None
                and sequence < self._last_cloud_sequence
            ):
                return False, 'cloud_sequence_regressed'
        if str(observation.sensor_state).upper() == 'STALE':
            return False, 'b1_sensor_stale'
        if str(observation.sensor_state).upper() not in (
            'CLEAR',
            'BLOCKED',
        ):
            return False, 'b1_sensor_state_invalid'

        age_checks = (
            (
                observation.cloud_age_sec,
                self.config.max_cloud_age_sec,
                'cloud',
            ),
            (
                observation.odom_age_sec,
                self.config.max_odom_age_sec,
                'odom',
            ),
        )
        for age, limit, name in age_checks:
            if not self._is_finite_nonnegative(age):
                return False, f'{name}_age_invalid'
            if float(age) > limit:
                return False, f'{name}_stale'

        if not self._is_finite(observation.yaw_rad):
            return False, 'yaw_invalid'
        if not self._is_finite(observation.turn_rad):
            return False, 'turn_invalid'
        if not isinstance(observation.distances, dict):
            return False, 'distances_invalid'

        for name in SECTOR_NAMES:
            if name not in observation.distances:
                return False, f'distance_{name}_missing'
            distance = observation.distances[name]
            if distance is None:
                continue
            if not self._is_finite_nonnegative(distance):
                return False, f'distance_{name}_invalid'
        return True, 'sensors_fresh'

    def expected_turn(self):
        """返回尚未完成的固定路线方向。"""
        if self.route_complete():
            return None
        return self.route_directions[self.route_index]

    def route_complete(self):
        return self.route_index >= len(self.route_directions)

    def corridor_center_error(self, distances):
        """计算相对走廊中心的误差，正值表示应向左修正。"""
        left = self._explicit_distance(distances.get('left'))
        right = self._explicit_distance(distances.get('right'))
        target = self.config.side_target_m

        if left is not None and right is not None:
            return 0.5 * (left - right)
        if left is not None:
            return left - target
        if right is not None:
            return target - right
        return None

    def corner_approach_center_error(self, direction, distances):
        """接近开口时只跟随对侧墙，避免开口远墙拉偏居中结果。"""
        if direction == DIRECTION_LEFT:
            right = self._explicit_distance(distances.get('right'))
            if right is None:
                return None
            return self.config.side_target_m - right
        if direction == DIRECTION_RIGHT:
            left = self._explicit_distance(distances.get('left'))
            if left is None:
                return None
            return left - self.config.side_target_m
        return None

    def rectangle_half_extents(self, yaw_rad):
        """计算矩形机身旋转到给定角度后的轴对齐半宽和半长。"""
        angle = float(yaw_rad)
        cosine = abs(math.cos(angle))
        sine = abs(math.sin(angle))
        half_x = (
            cosine * self.half_length_m
            + sine * self.half_width_m
        )
        half_y = (
            sine * self.half_length_m
            + cosine * self.half_width_m
        )
        return half_x, half_y

    def moving_turn_sweep_safe(self, direction, distances):
        """以开放侧、斜前和对侧余量保守检查移动转向包络。"""
        if direction not in VALID_DIRECTIONS:
            return False

        side_name, diagonal_name, opposite_name = self._turn_sector_names(
            direction
        )
        front = self._explicit_distance(distances.get('front'))
        turn_side = self._explicit_distance(distances.get(side_name))
        turn_diagonal = self._explicit_distance(
            distances.get(diagonal_name)
        )
        opposite = self._explicit_distance(
            distances.get(opposite_name)
        )

        # 目标侧无墙回波可表示拐角开口，但前方、目标斜前和对侧墙
        # 必须有显式量测，不能再把多个 n/a 乐观地当作最大净空。
        if (
            front is None
            or turn_diagonal is None
            or opposite is None
        ):
            return False

        required_open = (
            self.sweep_radius_m
            + self.config.footprint_safety_margin_m
        )
        turn_side_open = (
            turn_side is None
            or turn_side >= max(
                required_open,
                self.config.turn_open_distance_m,
            )
        )
        return (
            front >= self.front_collision_distance_m
            and turn_side_open
            and turn_diagonal >= self.front_collision_distance_m
            and opposite >= self.side_collision_distance_m
        )

    def turn_start_sweep_safe(self, direction, distances):
        """返回考虑已确认开口滞回后的转向启动包络状态。"""
        if direction not in VALID_DIRECTIONS:
            return False

        side_name, diagonal_name, opposite_name = self._turn_sector_names(
            direction
        )
        front = self._explicit_distance(distances.get('front'))
        turn_side = self._explicit_distance(distances.get(side_name))
        turn_diagonal = self._explicit_distance(
            distances.get(diagonal_name)
        )
        opposite = self._explicit_distance(
            distances.get(opposite_name)
        )
        if (
            front is None
            or turn_diagonal is None
            or opposite is None
        ):
            return False

        required_open = max(
            self.sweep_radius_m
            + self.config.footprint_safety_margin_m,
            self.config.turn_open_distance_m,
        )
        if self._turn_open_latched:
            # 滞回下限绝不得低于机身半宽加安全边界。
            required_open = max(
                self.side_collision_distance_m,
                required_open
                - self.config.turn_open_hysteresis_m,
            )
        turn_side_open = (
            turn_side is None
            or turn_side >= required_open
        )
        return (
            front >= self.front_collision_distance_m
            and turn_side_open
            and turn_diagonal >= self.front_collision_distance_m
            and opposite >= self.side_collision_distance_m
        )

    def _update_turn_open_latch(self, direction, distances):
        """严格门限进入、滞回门限退出，其他包络量仍每帧检查。"""
        if self.moving_turn_sweep_safe(direction, distances):
            self._turn_open_latched = True
            return True
        if (
            self._turn_open_latched
            and self.turn_start_sweep_safe(direction, distances)
        ):
            return True
        self._turn_open_latched = False
        return False

    def active_turn_required_side_clearance_m(
        self,
        require_full_sweep=False,
    ):
        """返回当前转向阶段要求的内侧最小距离。"""
        if require_full_sweep:
            # 粗转阶段机身长边会横扫内侧板，必须按整个矩形外接半径保护；
            # 只使用半宽会在约45度时漏掉左/右侧机身碰撞。
            return max(
                self.side_collision_distance_m,
                self.sweep_radius_m
                + self.config.footprint_safety_margin_m,
            )
        return self.side_collision_distance_m

    def active_turn_clearance_safe(
        self,
        direction,
        distances,
        require_full_sweep=False,
    ):
        """按粗转外接半径或精调半宽检查实时内侧净空。"""
        if direction not in VALID_DIRECTIONS:
            return False

        side_name, diagonal_name, opposite_name = self._turn_sector_names(
            direction
        )
        front = self._explicit_distance(distances.get('front'))
        turn_side = self._explicit_distance(distances.get(side_name))
        turn_diagonal = self._explicit_distance(
            distances.get(diagonal_name)
        )
        opposite = self._explicit_distance(
            distances.get(opposite_name)
        )

        if (
            front is None
            or turn_diagonal is None
            or opposite is None
        ):
            return False

        required_turn_side = (
            self.active_turn_required_side_clearance_m(
                require_full_sweep=require_full_sweep,
            )
        )
        # 启动前的开口确认不能替代动态检查。粗转时墙端重新进入内侧
        # 扇区，必须保留整机扫掠半径；精调阶段才按机身半宽判断。
        turn_side_safe = (
            turn_side is None
            or turn_side >= required_turn_side
        )
        return (
            front >= self.front_collision_distance_m
            and turn_side_safe
            and turn_diagonal >= self.front_collision_distance_m
            and opposite >= self.side_collision_distance_m
        )

    def _handle_wait_sensor(self, observation, now):
        if self._missing_required_side(observation.distances):
            # 启动阶段先等待稳定侧墙几何，不把偶发缺测计入确认帧。
            self._sensor_streak = 0
            self.reason = 'side_distance_confirmation_pending'
            self._set_command(0.0, 0.0)
            return self.snapshot(now)

        self._sensor_streak += 1
        self.reason = (
            f'sensor_confirmation_{self._sensor_streak}/'
            f'{self.config.sensor_confirm_frames}'
        )
        self._set_command(0.0, 0.0)
        if self._sensor_streak >= self.config.sensor_confirm_frames:
            self._sensor_ready_latched = True
            self._transition(
                STATE_CORRIDOR_FOLLOW,
                'sensors_confirmed',
                now,
            )
            return self._handle_corridor_follow(observation, now)
        return self.snapshot(now)

    def _handle_corridor_follow(self, observation, now):
        distances = observation.distances
        front = self._front_distance(distances)
        expected_turn = self.expected_turn()
        approaching_known_corner = (
            expected_turn is not None
            and front <= self.config.corner_approach_distance_m
        )
        allowed_missing = ()
        if approaching_known_corner:
            # 接近已知拐角时，预期转向侧墙消失可作为开口候选；
            # 对侧墙仍必须可观测，避免把整片点云缺失误判为空旷。
            allowed_missing = (self._turn_sector_names(
                expected_turn
            )[0],)

        if self._pause_or_fault_for_missing(
            self._missing_required_side(distances, allowed_missing),
            'corridor_side_distance_missing',
            now,
        ):
            return self.snapshot(now)
        if self._pause_or_fault_for_side_clearance(
            not self._side_clearance_safe(
                distances,
                allowed_missing,
            ),
            'corridor_side_clearance_unsafe',
            now,
        ):
            return self.snapshot(now)

        if front <= self.config.front_emergency_distance_m:
            self._transition(
                STATE_REVERSE_RECOVERY,
                'front_emergency_reverse_candidate',
                now,
            )
            return self._handle_reverse_recovery(observation, now)

        if self.route_complete():
            if self._exit_open(distances):
                self._exit_streak += 1
            else:
                self._exit_streak = 0
            if self._exit_streak >= self.config.exit_confirm_frames:
                self._transition(
                    STATE_FINISHED,
                    'radar_exit_confirmed',
                    now,
                )
                self._set_command(0.0, 0.0)
                return self.snapshot(now)
        else:
            self._exit_streak = 0
            if approaching_known_corner:
                self._corner_streak += 1
            else:
                self._corner_streak = 0
            if self._corner_streak >= self.config.corner_confirm_frames:
                self._transition(
                    STATE_CORNER_APPROACH,
                    'corner_distance_confirmed',
                    now,
                )
                return self._handle_corner_approach(observation, now)

        speed = self._corridor_speed(front)
        if approaching_known_corner:
            # 状态持续帧尚未确认完成时，也必须立即忽略开口侧远墙；
            # 否则短暂的错误修正方向可能把机器人推向挡板。
            correction = self._corner_center_command(
                expected_turn,
                distances,
            )
        else:
            correction = self._center_command(distances)
        self.reason = (
            'route_complete_search_exit'
            if self.route_complete()
            else 'corridor_centering'
        )
        self._set_command(speed, correction)
        return self.snapshot(now)

    def _handle_corner_approach(self, observation, now):
        if self._state_age(now) > self.config.corner_timeout_sec:
            self._enter_fault('corner_approach_timeout', now)
            return self.snapshot(now)

        distances = observation.distances
        direction = self.expected_turn()
        if direction is None:
            self._enter_fault('corner_without_expected_turn', now)
            return self.snapshot(now)

        turn_side = self._turn_sector_names(direction)[0]
        allowed_missing = (turn_side,)
        if self._pause_or_fault_for_missing(
            self._missing_required_side(distances, allowed_missing),
            'corner_side_distance_missing',
            now,
        ):
            return self.snapshot(now)
        if self._pause_or_fault_for_side_clearance(
            not self._side_clearance_safe(
                distances,
                allowed_missing,
            ),
            'corner_side_clearance_unsafe',
            now,
        ):
            return self.snapshot(now)

        front = self._front_distance(distances)
        if front <= self.config.front_emergency_distance_m:
            self._turn_start_streak = 0
            self._transition(
                STATE_REVERSE_RECOVERY,
                'turn_envelope_unsafe_reverse_candidate',
                now,
            )
            return self._handle_reverse_recovery(observation, now)

        if front <= self.config.turn_start_distance_m:
            sweep_safe = self._update_turn_open_latch(
                direction,
                distances,
            )
            if sweep_safe:
                # 开口必须连续稳定，不能由单帧远距离噪声锁存转向状态。
                self._turn_start_streak += 1
                self.reason = (
                    f'turn_start_confirmation_'
                    f'{self._turn_start_streak}/'
                    f'{self.config.turn_start_confirm_frames}'
                )
                self._set_command(0.0, 0.0)
                if (
                    self._turn_start_streak
                    >= self.config.turn_start_confirm_frames
                ):
                    self._begin_turn(direction, observation.turn_rad, now)
                    return self._handle_turn(observation, now)
                return self.snapshot(now)

            self._turn_start_streak = 0
            self.reason = 'waiting_for_turn_opening'
            self._set_command(0.0, 0.0)
        else:
            self._turn_open_latched = False
            self._turn_start_streak = 0
            self.reason = 'approaching_turn_start'
            self._set_command(
                self.config.approach_vx,
                self._corner_center_command(direction, distances),
            )
        return self.snapshot(now)

    def _handle_turn(self, observation, now):
        if self._turn_elapsed(now) > self.config.turn_timeout_sec:
            self._enter_fault('turn_timeout', now)
            return self.snapshot(now)
        direction = self.expected_turn()
        if self._pause_or_fault_for_missing(
            not self._turn_clearance_observable(
                direction,
                observation.distances,
            ),
            'turn_clearance_missing',
            now,
        ):
            return self.snapshot(now)
        if self._pause_or_fault_for_side_clearance(
            not self.active_turn_clearance_safe(
                direction,
                observation.distances,
                require_full_sweep=True,
            ),
            'turn_sweep_unsafe',
            now,
        ):
            return self.snapshot(now)
        if not self._instant_clearance_safe(observation.distances):
            self._enter_fault('turn_clearance_unsafe', now)
            return self.snapshot(now)

        error = self._turn_error(observation.turn_rad)
        if error is None:
            self._enter_fault('turn_target_missing', now)
            return self.snapshot(now)

        if abs(error) <= math.radians(
            self.config.fine_align_enter_deg
        ):
            self._transition(
                STATE_TURN_FINE_ALIGN,
                'enter_turn_fine_align',
                now,
                preserve_turn=True,
            )
            return self._handle_fine_align(observation, now)

        wz = self._bounded_turn_wz(error)
        self.reason = 'yaw_closed_loop_turn'
        self._set_command(self.config.turn_forward_vx, wz)
        return self.snapshot(now)

    def _handle_fine_align(self, observation, now):
        if self._turn_elapsed(now) > self.config.turn_timeout_sec:
            self._enter_fault('turn_fine_align_timeout', now)
            return self.snapshot(now)
        direction = self.expected_turn()
        if self._pause_or_fault_for_missing(
            not self._turn_clearance_observable(
                direction,
                observation.distances,
            ),
            'fine_align_clearance_missing',
            now,
        ):
            return self.snapshot(now)
        if self._pause_or_fault_for_side_clearance(
            not self.active_turn_clearance_safe(
                direction,
                observation.distances,
                require_full_sweep=False,
            ),
            'fine_align_sweep_unsafe',
            now,
        ):
            return self.snapshot(now)
        if not self._instant_clearance_safe(observation.distances):
            self._enter_fault('fine_align_clearance_unsafe', now)
            return self.snapshot(now)

        error = self._turn_error(observation.turn_rad)
        if error is None:
            self._enter_fault('fine_align_target_missing', now)
            return self.snapshot(now)

        if abs(error) > math.radians(self.config.fine_align_exit_deg):
            turn_state = (
                STATE_TURN_LEFT
                if error > 0.0
                else STATE_TURN_RIGHT
            )
            self._transition(
                turn_state,
                'fine_align_error_increased',
                now,
                preserve_turn=True,
            )
            return self._handle_turn(observation, now)

        if abs(error) <= math.radians(
            self.config.turn_tolerance_deg
        ):
            self._turn_streak += 1
        else:
            self._turn_streak = 0

        if self._turn_streak >= self.config.turn_confirm_frames:
            self._transition(
                STATE_CORRIDOR_REACQUIRE,
                'turn_angle_confirmed',
                now,
                preserve_turn=True,
            )
            return self._handle_reacquire(observation, now)

        wz = self._clamp(
            self.config.fine_turn_kp * error,
            -self.config.max_fine_wz,
            self.config.max_fine_wz,
        )
        self.reason = (
            f'fine_align_confirmation_{self._turn_streak}/'
            f'{self.config.turn_confirm_frames}'
        )
        self._set_command(self.config.fine_align_vx, wz)
        return self.snapshot(now)

    def _handle_reacquire(self, observation, now):
        if self._state_age(now) > self.config.reacquire_timeout_sec:
            self._enter_fault('corridor_reacquire_timeout', now)
            return self.snapshot(now)
        if self._pause_or_fault_for_missing(
            self._missing_required_side(observation.distances),
            'reacquire_side_distance_missing',
            now,
        ):
            return self.snapshot(now)
        if self._pause_or_fault_for_side_clearance(
            not self._side_clearance_safe(observation.distances),
            'reacquire_side_clearance_unsafe',
            now,
        ):
            return self.snapshot(now)

        front = self._front_distance(observation.distances)
        if front <= self.config.front_emergency_distance_m:
            # 短直段允许在62cm内完成重捕获，但绝不允许
            # 在紧急前距内继续输出向前诊断值。
            self._enter_fault(
                'reacquire_front_clearance_unsafe',
                now,
            )
            return self.snapshot(now)
        center_error = self.corridor_center_error(
            observation.distances
        )
        yaw_error = self._turn_error(observation.turn_rad)
        centered = (
            center_error is not None
            and abs(center_error) <= self.config.center_tolerance_m
        )
        yaw_aligned = (
            yaw_error is not None
            and abs(yaw_error) <= math.radians(
                self.config.reacquire_yaw_tolerance_deg
            )
        )
        if (
            centered
            and yaw_aligned
        ):
            self._reacquire_streak += 1
        else:
            self._reacquire_streak = 0

        if (
            self._reacquire_streak
            >= self.config.reacquire_confirm_frames
        ):
            self.route_index += 1
            self._clear_turn_target()
            self._transition(
                STATE_CORRIDOR_FOLLOW,
                'corridor_reacquired',
                now,
            )
            return self._handle_corridor_follow(observation, now)

        self.reason = (
            f'corridor_reacquire_{self._reacquire_streak}/'
            f'{self.config.reacquire_confirm_frames}'
        )
        self._set_command(
            self.config.reacquire_vx,
            self._center_command(observation.distances),
        )
        return self.snapshot(now)

    def _handle_reverse_recovery(self, observation, now):
        # B1 没有后扇区，B3 未增加后向保护前不得执行反向值。
        if self._state_age(now) > self.config.reverse_timeout_sec:
            self._enter_fault('reverse_recovery_timeout', now)
            return self.snapshot(now)
        direction = self.expected_turn()
        allowed_missing = ()
        if direction is not None:
            allowed_missing = (
                self._turn_sector_names(direction)[0],
            )
        if self._pause_or_fault_for_missing(
            self._missing_required_side(
                observation.distances,
                allowed_missing,
            ),
            'reverse_side_distance_missing',
            now,
        ):
            return self.snapshot(now)
        if self._pause_or_fault_for_side_clearance(
            not self._side_clearance_safe(
                observation.distances,
                allowed_missing,
            ),
            'reverse_side_clearance_unsafe',
            now,
        ):
            return self.snapshot(now)

        front = self._front_distance(observation.distances)
        sweep_safe = (
            direction is not None
            and self.moving_turn_sweep_safe(
                direction,
                observation.distances,
            )
        )
        if (
            front >= self.config.recovery_front_clear_m
            and sweep_safe
        ):
            self._recovery_streak += 1
        else:
            self._recovery_streak = 0

        if (
            self._recovery_streak
            >= self.config.recovery_confirm_frames
        ):
            self._transition(
                STATE_CORNER_APPROACH,
                'reverse_recovery_clearance_restored',
                now,
            )
            return self._handle_corner_approach(observation, now)

        self.reason = (
            'reverse_recovery_diagnostic_only_no_rear_sector'
        )
        self._set_command(-self.config.reverse_vx, 0.0)
        return self.snapshot(now)

    def _begin_turn(self, direction, turn_rad, now):
        sign = 1.0 if direction == DIRECTION_LEFT else -1.0
        self._turn_start_rad = float(turn_rad)
        self._turn_target_rad = (
            self._turn_start_rad
            + sign * math.radians(self.config.turn_angle_deg)
        )
        self._turn_start_time = now
        state = (
            STATE_TURN_LEFT
            if direction == DIRECTION_LEFT
            else STATE_TURN_RIGHT
        )
        self._transition(
            state,
            f'{direction.lower()}_turn_started',
            now,
            preserve_turn=True,
        )

    def _transition(self, state, reason, now, preserve_turn=False):
        self.state = state
        self.reason = str(reason)
        self._state_enter_time = now
        self._corner_streak = 0
        self._side_missing_streak = 0
        self._side_unsafe_streak = 0
        self._turn_start_streak = 0
        self._turn_streak = 0
        self._reacquire_streak = 0
        self._recovery_streak = 0
        self._turn_open_latched = False
        if not preserve_turn and state not in (
            STATE_TURN_LEFT,
            STATE_TURN_RIGHT,
            STATE_TURN_FINE_ALIGN,
            STATE_CORRIDOR_REACQUIRE,
        ):
            self._clear_turn_target()

    def _enter_fault(self, reason, now):
        self.state = STATE_FAULT_STOP
        self.reason = str(reason)
        self._state_enter_time = now
        self._set_command(0.0, 0.0)

    def _pause_or_fault_for_missing(self, missing, fault_reason, now):
        """侧向量测短暂缺失时先停住确认，持续缺失才锁定故障。"""
        if not missing:
            self._side_missing_streak = 0
            return False

        # 缺测与低净空不是同一连续事件，任一缺测帧都会打断低净空计数。
        self._side_unsafe_streak = 0
        self._side_missing_streak += 1
        self._set_command(0.0, 0.0)
        if (
            self._side_missing_streak
            >= self.config.side_missing_confirm_frames
        ):
            self._enter_fault(fault_reason, now)
        else:
            self.reason = (
                f'{fault_reason}_confirmation_'
                f'{self._side_missing_streak}/'
                f'{self.config.side_missing_confirm_frames}'
            )
        return True

    def _pause_or_fault_for_side_clearance(self, unsafe, fault_reason, now):
        """单帧低净空先归零停住，连续低净空才锁定故障。"""
        if not unsafe:
            self._side_unsafe_streak = 0
            return False

        self._side_unsafe_streak += 1
        self._set_command(0.0, 0.0)
        if (
            self._side_unsafe_streak
            >= self.config.side_unsafe_confirm_frames
        ):
            self._enter_fault(fault_reason, now)
        else:
            self.reason = (
                f'{fault_reason}_confirmation_'
                f'{self._side_unsafe_streak}/'
                f'{self.config.side_unsafe_confirm_frames}'
            )
        return True

    def _set_command(self, vx, wz):
        self.desired_vx = float(vx)
        self.desired_wz = float(wz)

    def _corridor_speed(self, front):
        if front >= self.config.front_slow_distance_m:
            return self.config.corridor_vx
        span = (
            self.config.front_slow_distance_m
            - self.config.corner_approach_distance_m
        )
        if span <= 0.0:
            return self.config.approach_vx
        ratio = (
            front - self.config.corner_approach_distance_m
        ) / span
        ratio = self._clamp(ratio, 0.0, 1.0)
        return (
            self.config.approach_vx
            + ratio
            * (self.config.corridor_vx - self.config.approach_vx)
        )

    def _center_command(self, distances):
        error = self.corridor_center_error(distances)
        return self._center_command_from_error(error)

    def _corner_center_command(self, direction, distances):
        error = self.corner_approach_center_error(direction, distances)
        return self._center_command_from_error(error)

    def _center_command_from_error(self, error):
        """将已选定参考墙的横向误差转换为限幅诊断角速度。"""
        if error is None:
            return 0.0
        return self._clamp(
            self.config.center_kp * error,
            -self.config.max_center_wz,
            self.config.max_center_wz,
        )

    def _center_error_for_state(self, distances, expected_turn):
        """返回当前状态实际采用的居中误差和可诊断参考墙。"""
        if self._uses_corner_reference(distances, expected_turn):
            error = self.corner_approach_center_error(
                expected_turn,
                distances,
            )
            reference = (
                'right_wall'
                if expected_turn == DIRECTION_LEFT
                else 'left_wall'
                if expected_turn == DIRECTION_RIGHT
                else 'none'
            )
            return error, reference
        return self.corridor_center_error(distances), 'both_walls'

    def _uses_corner_reference(self, distances, expected_turn):
        """确认中也提前跟随对侧墙，避免目标侧开口回波反向拉偏。"""
        if expected_turn not in VALID_DIRECTIONS:
            return False
        if self.state == STATE_CORNER_APPROACH:
            return True
        return (
            self.state == STATE_CORRIDOR_FOLLOW
            and self._front_distance(distances)
            <= self.config.corner_approach_distance_m
        )

    def _accept_new_cloud_sequence(self, sequence):
        """只让递增点云序号推进策略；None 保持旧录包兼容。"""
        if sequence is None:
            return True
        if self._last_cloud_sequence == sequence:
            return False
        self._last_cloud_sequence = sequence
        return True

    def _bounded_turn_wz(self, error):
        raw = self.config.turn_kp * float(error)
        magnitude = self._clamp(
            abs(raw),
            self.config.min_turn_wz,
            self.config.max_turn_wz,
        )
        return math.copysign(magnitude, error)

    def _front_distance(self, distances):
        return self._effective_distance(
            distances.get('front'),
            self.config.front_max_range_m,
        )

    def _side_clearance_safe(self, distances, allowed_missing=()):
        """检查侧向碰撞净距；只有显式允许的开口侧可缺测。"""
        for name in ('left', 'right'):
            distance = self._explicit_distance(distances.get(name))
            if distance is None:
                if name in allowed_missing:
                    continue
                return False
            if distance < self.side_collision_distance_m:
                return False
        return True

    @staticmethod
    def _missing_required_side(distances, allowed_missing=()):
        """返回是否缺少当前状态必须可观测的侧墙距离。"""
        for name in ('left', 'right'):
            if name in allowed_missing:
                continue
            distance = MazeNavigationPolicy._explicit_distance(
                distances.get(name)
            )
            if distance is None:
                return True
        return False

    def _turn_clearance_observable(self, direction, distances):
        """转弯时要求前方、目标斜前和对侧墙均有实测距离。"""
        if direction not in VALID_DIRECTIONS:
            return False
        _, diagonal_name, opposite_name = self._turn_sector_names(
            direction
        )
        required_names = ('front', diagonal_name, opposite_name)
        return all(
            self._explicit_distance(distances.get(name)) is not None
            for name in required_names
        )

    def _instant_clearance_safe(self, distances):
        front = self._explicit_distance(distances.get('front'))
        if (
            front is not None
            and front < self.front_collision_distance_m
        ):
            return False
        minimum = self.side_collision_distance_m
        for name in ('left_front', 'right_front', 'left', 'right'):
            distance = self._explicit_distance(distances.get(name))
            if distance is not None and distance < minimum:
                return False
        return True

    def _exit_open(self, distances):
        front = self._effective_distance(
            distances.get('front'),
            self.config.front_max_range_m,
        )
        left = self._effective_distance(
            distances.get('left'),
            self.config.side_max_range_m,
        )
        right = self._effective_distance(
            distances.get('right'),
            self.config.side_max_range_m,
        )
        return (
            front >= self.config.exit_front_clear_m
            and left >= self.config.exit_side_open_m
            and right >= self.config.exit_side_open_m
        )

    def _turn_sector_names(self, direction):
        if direction == DIRECTION_LEFT:
            return 'left', 'left_front', 'right'
        return 'right', 'right_front', 'left'

    def _turn_error(self, turn_rad):
        if self._turn_target_rad is None or turn_rad is None:
            return None
        if not self._is_finite(turn_rad):
            return None
        return self._turn_target_rad - float(turn_rad)

    def _turn_progress(self, turn_rad):
        """返回本次转向相对启动时刻的有符号角度，不使用全程累计值。"""
        if self._turn_start_rad is None or turn_rad is None:
            return None
        if not self._is_finite(turn_rad):
            return None
        return float(turn_rad) - self._turn_start_rad

    def _turn_elapsed(self, now):
        if self._turn_start_time is None:
            return math.inf
        return max(0.0, now - self._turn_start_time)

    def _state_age(self, now):
        if self._state_enter_time is None:
            return 0.0
        return max(0.0, now - self._state_enter_time)

    def _clear_turn_target(self):
        self._turn_start_rad = None
        self._turn_target_rad = None
        self._turn_start_time = None

    def _validate(self):
        if not self.route_directions:
            raise ValueError('route_directions must not be empty')
        invalid = [
            direction
            for direction in self.route_directions
            if direction not in VALID_DIRECTIONS
        ]
        if invalid:
            raise ValueError(
                f'invalid route directions: {", ".join(invalid)}'
            )

        positive_values = {
            'robot_length_m': self.config.robot_length_m,
            'robot_width_m': self.config.robot_width_m,
            'robot_height_m': self.config.robot_height_m,
            'wall_height_m': self.config.wall_height_m,
            'corridor_width_m': self.config.corridor_width_m,
            'front_max_range_m': self.config.front_max_range_m,
            'side_max_range_m': self.config.side_max_range_m,
            'max_cloud_age_sec': self.config.max_cloud_age_sec,
            'max_odom_age_sec': self.config.max_odom_age_sec,
            'front_slow_distance_m': (
                self.config.front_slow_distance_m
            ),
            'corner_approach_distance_m': (
                self.config.corner_approach_distance_m
            ),
            'turn_start_distance_m': (
                self.config.turn_start_distance_m
            ),
            'front_emergency_distance_m': (
                self.config.front_emergency_distance_m
            ),
            'recovery_front_clear_m': (
                self.config.recovery_front_clear_m
            ),
            'turn_open_distance_m': (
                self.config.turn_open_distance_m
            ),
            'exit_front_clear_m': self.config.exit_front_clear_m,
            'exit_side_open_m': self.config.exit_side_open_m,
            'side_target_m': self.config.side_target_m,
            'turn_angle_deg': self.config.turn_angle_deg,
            'corner_timeout_sec': self.config.corner_timeout_sec,
            'turn_timeout_sec': self.config.turn_timeout_sec,
            'reacquire_timeout_sec': (
                self.config.reacquire_timeout_sec
            ),
            'reverse_timeout_sec': self.config.reverse_timeout_sec,
        }
        for name, value in positive_values.items():
            if not self._is_finite(value) or float(value) <= 0.0:
                raise ValueError(f'{name} must be positive and finite')

        if self.config.corridor_width_m <= self.config.robot_width_m:
            raise ValueError(
                'corridor_width_m must be greater than robot_width_m'
            )
        if (
            not self._is_finite(
                self.config.footprint_safety_margin_m
            )
            or self.config.footprint_safety_margin_m < 0.0
        ):
            raise ValueError(
                'footprint_safety_margin_m must be nonnegative and finite'
            )
        if self.config.side_target_m >= self.config.side_max_range_m:
            raise ValueError(
                'side_target_m must be less than side_max_range_m'
            )
        if not (
            self.config.front_emergency_distance_m
            < self.config.turn_start_distance_m
            < self.config.corner_approach_distance_m
            < self.config.front_slow_distance_m
        ):
            raise ValueError(
                'front thresholds must satisfy emergency < turn_start '
                '< corner_approach < front_slow'
            )
        if (
            self.config.recovery_front_clear_m
            <= self.config.turn_start_distance_m
        ):
            raise ValueError(
                'recovery_front_clear_m must exceed '
                'turn_start_distance_m'
            )
        if (
            not self._is_finite(self.config.turn_open_hysteresis_m)
            or self.config.turn_open_hysteresis_m < 0.0
            or self.config.turn_open_hysteresis_m
            >= self.config.turn_open_distance_m
        ):
            raise ValueError(
                'turn_open_hysteresis_m must be finite, nonnegative, '
                'and less than turn_open_distance_m'
            )
        if not (
            0.0 < self.config.turn_tolerance_deg
            < self.config.fine_align_enter_deg
            < self.config.fine_align_exit_deg
            < 90.0
        ):
            raise ValueError(
                'turn tolerances must satisfy tolerance < fine enter '
                '< fine exit < 90'
            )

        frame_values = {
            'sensor_confirm_frames': self.config.sensor_confirm_frames,
            'side_missing_confirm_frames': (
                self.config.side_missing_confirm_frames
            ),
            'side_unsafe_confirm_frames': (
                self.config.side_unsafe_confirm_frames
            ),
            'corner_confirm_frames': self.config.corner_confirm_frames,
            'turn_start_confirm_frames': (
                self.config.turn_start_confirm_frames
            ),
            'turn_confirm_frames': self.config.turn_confirm_frames,
            'reacquire_confirm_frames': (
                self.config.reacquire_confirm_frames
            ),
            'recovery_confirm_frames': (
                self.config.recovery_confirm_frames
            ),
            'exit_confirm_frames': self.config.exit_confirm_frames,
        }
        for name, value in frame_values.items():
            if int(value) <= 0:
                raise ValueError(f'{name} must be positive')

        nonnegative_values = {
            'corridor_vx': self.config.corridor_vx,
            'approach_vx': self.config.approach_vx,
            'turn_forward_vx': self.config.turn_forward_vx,
            'fine_align_vx': self.config.fine_align_vx,
            'reacquire_vx': self.config.reacquire_vx,
            'reverse_vx': self.config.reverse_vx,
            'max_center_wz': self.config.max_center_wz,
            'max_turn_wz': self.config.max_turn_wz,
            'min_turn_wz': self.config.min_turn_wz,
            'max_fine_wz': self.config.max_fine_wz,
            'center_kp': self.config.center_kp,
            'turn_kp': self.config.turn_kp,
            'fine_turn_kp': self.config.fine_turn_kp,
            'center_tolerance_m': self.config.center_tolerance_m,
        }
        for name, value in nonnegative_values.items():
            if not self._is_finite(value) or float(value) < 0.0:
                raise ValueError(
                    f'{name} must be nonnegative and finite'
                )
        if self.config.min_turn_wz > self.config.max_turn_wz:
            raise ValueError(
                'min_turn_wz must not exceed max_turn_wz'
            )

    @staticmethod
    def _effective_distance(distance, maximum):
        if distance is None:
            return float(maximum)
        value = float(distance)
        if not math.isfinite(value):
            return float(maximum)
        return value

    @staticmethod
    def _explicit_distance(distance):
        if distance is None:
            return None
        value = float(distance)
        if not math.isfinite(value):
            return None
        return value

    @staticmethod
    def _finite_now(now_sec):
        now = float(now_sec)
        if not math.isfinite(now):
            raise ValueError('now_sec must be finite')
        return now

    @staticmethod
    def _is_finite(value):
        try:
            return math.isfinite(float(value))
        except (TypeError, ValueError):
            return False

    @classmethod
    def _is_finite_nonnegative(cls, value):
        return cls._is_finite(value) and float(value) >= 0.0

    @staticmethod
    def _clamp(value, minimum, maximum):
        return max(float(minimum), min(float(maximum), float(value)))

    @staticmethod
    def _degrees_or_none(value):
        if value is None:
            return None
        return math.degrees(value)
