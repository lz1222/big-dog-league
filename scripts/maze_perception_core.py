#!/usr/bin/env python3

"""B1 迷宫感知的纯计算核心，可脱离 ROS 进行录包或模拟测试。"""

import math


# 五扇区基于 base_link 坐标系：x 向前、y 向左、z 向上。
SECTOR_NAMES = (
    'front',
    'left_front',
    'right_front',
    'left',
    'right',
)

# 状态和建议只是感知层输出，不是机器人运动命令。
STATE_CLEAR = 'CLEAR'
STATE_BLOCKED = 'BLOCKED'
STATE_STALE = 'STALE'

ADVICE_FORWARD = 'FORWARD'
ADVICE_TURN_LEFT = 'TURN_LEFT'
ADVICE_TURN_RIGHT = 'TURN_RIGHT'
ADVICE_STOP = 'STOP'


class SectorExtractor:
    """过滤三维点并计算五扇区的稳健距离。"""

    def __init__(
        self,
        z_min,
        z_max,
        body_x_min,
        body_x_max,
        body_y_min,
        body_y_max,
        front_angle_deg,
        min_range,
        front_max_range,
        side_max_range,
        distance_percentile,
        diagonal_angle_max_deg=60.0,
        side_angle_max_deg=120.0,
        side_projection_angle_min_deg=15.0,
        side_projection_angle_max_deg=60.0,
        side_projection_x_min=0.45,
        side_projection_x_max=1.50,
        side_projection_min_x_span=0.12,
        side_projection_lateral_tolerance=0.04,
        side_min_points=3,
    ):
        self.z_min = float(z_min)
        self.z_max = float(z_max)
        self.body_x_min = float(body_x_min)
        self.body_x_max = float(body_x_max)
        self.body_y_min = float(body_y_min)
        self.body_y_max = float(body_y_max)
        self.front_angle_deg = float(front_angle_deg)
        self.min_range = float(min_range)
        self.front_max_range = float(front_max_range)
        self.side_max_range = float(side_max_range)
        self.distance_percentile = float(distance_percentile)
        self.diagonal_angle_max_deg = float(
            diagonal_angle_max_deg
        )
        self.side_angle_max_deg = float(side_angle_max_deg)
        self.side_projection_angle_min_deg = float(
            side_projection_angle_min_deg
        )
        self.side_projection_angle_max_deg = float(
            side_projection_angle_max_deg
        )
        self.side_projection_x_min = float(side_projection_x_min)
        self.side_projection_x_max = float(side_projection_x_max)
        self.side_projection_min_x_span = float(
            side_projection_min_x_span
        )
        self.side_projection_lateral_tolerance = float(
            side_projection_lateral_tolerance
        )
        self.side_min_points = int(side_min_points)
        self._validate()
        self._front_angle_rad = math.radians(self.front_angle_deg)
        self._diagonal_angle_max_rad = math.radians(
            self.diagonal_angle_max_deg
        )
        self._side_angle_max_rad = math.radians(
            self.side_angle_max_deg
        )
        self._side_projection_angle_min_rad = math.radians(
            self.side_projection_angle_min_deg
        )
        self._side_projection_angle_max_rad = math.radians(
            self.side_projection_angle_max_deg
        )

    def extract(self, points):
        """返回各扇区距离、点数以及点云基本质量统计。"""
        sector_values = {
            name: []
            for name in SECTOR_NAMES
        }
        projected_side_candidates = {
            'left': [],
            'right': [],
        }
        # 短侧墙候选不直接成为净距，只交给后续带历史约束的稳定器确认。
        side_continuity_candidates = {
            'left': [],
            'right': [],
        }
        primary_point_indexes = set()
        total_points = 0
        finite_points = 0

        for point_index, point in enumerate(points):
            # total/finite 统计用于区分空点云与含 NaN/Inf 的损坏点云。
            total_points += 1
            x = float(point[0])
            y = float(point[1])
            z = float(point[2])

            if not all(math.isfinite(value) for value in (x, y, z)):
                continue
            finite_points += 1

            # 依次移除地面/高处点、自身机体点和过近噪声。
            if z < self.z_min or z > self.z_max:
                continue
            if self._inside_body_filter(x, y):
                continue

            distance = math.hypot(x, y)
            if distance < self.min_range:
                continue

            angle = math.atan2(y, x)
            sector = self._classify_sector(angle)
            accepted = False

            if sector in ('front', 'left_front', 'right_front'):
                if distance <= self.front_max_range:
                    sector_values[sector].append(distance)
                    accepted = True
            elif sector in ('left', 'right'):
                # 正侧方量测本来就等于横向净距，统一使用 |y|。
                lateral_distance = abs(y)
                if lateral_distance <= self.side_max_range:
                    sector_values[sector].append(lateral_distance)
                    accepted = True

            if accepted:
                primary_point_indexes.add(point_index)

            projected_side = self._classify_projected_side(x, y, angle)
            if projected_side is not None and projected_side != sector:
                # Go2 真机的低矮挡板回波集中在斜前方；投影到 y 轴后
                # 还需在整帧中确认这些点沿 x 方向形成侧墙，而不是前墙。
                lateral_distance = abs(y)
                if lateral_distance <= self.side_max_range:
                    projected_side_candidates[projected_side].append(
                        (point_index, x, lateral_distance)
                    )

        # 正侧点优先；正侧点不足时才使用具有足够 x 跨度的斜前投影。
        # 前挡板通常近似固定 x，因此不会再被投影成左右两侧的假近障。
        selected_projection_indexes = set()
        continuity_projection_indexes = set()
        side_sources = {}
        for name in ('left', 'right'):
            direct_values = sector_values[name]
            if len(direct_values) >= self.side_min_points:
                side_sources[name] = 'direct'
                continue

            candidates = projected_side_candidates[name]
            selected_candidates, short_clusters = (
                self._select_projected_side_candidates(candidates)
            )
            if selected_candidates:
                sector_values[name] = [
                    lateral_distance
                    for _, _, lateral_distance in selected_candidates
                ]
                selected_projection_indexes.update(
                    point_index
                    for point_index, _, _ in selected_candidates
                )
                side_sources[name] = 'projected'
            else:
                side_sources[name] = 'none'
                # 拐角处侧墙可见纵向长度会暂时小于强确认门限。保留所有
                # 横向成簇的短墙候选，由稳定器与最近强确认墙距比较；这里
                # 不能直接输出距离，否则固定 x 的前挡板边缘会被误用。
                for cluster, robust_x_span in short_clusters:
                    lateral_values = [
                        lateral_distance
                        for _, _, lateral_distance in cluster
                    ]
                    side_continuity_candidates[name].append({
                        'distance': self._percentile(
                            lateral_values,
                            self.distance_percentile,
                        ),
                        'count': len(cluster),
                        'x_span': robust_x_span,
                    })
                    continuity_projection_indexes.update(
                        point_index
                        for point_index, _, _ in cluster
                    )

        # 使用百分位距离，避免单个飞点像“最小值”一样触发误报。
        counts = {
            name: len(sector_values[name])
            for name in SECTOR_NAMES
        }
        distances = {}
        for name in SECTOR_NAMES:
            # 侧墙投影必须有多点支持；稀疏单点不能被当作可靠墙面。
            if (
                name in ('left', 'right')
                and counts[name] < self.side_min_points
            ):
                distances[name] = None
                continue
            distances[name] = self._percentile(
                sector_values[name],
                self.distance_percentile,
            )
        return {
            'distances': distances,
            'counts': counts,
            'sources': {
                'front': 'angular',
                'left_front': 'angular',
                'right_front': 'angular',
                'left': side_sources['left'],
                'right': side_sources['right'],
            },
            'side_continuity_candidates': side_continuity_candidates,
            # 投影点会同时参与斜前障碍和侧墙距离，统计时只计一次。
            'valid_points': len(
                primary_point_indexes
                | selected_projection_indexes
                | continuity_projection_indexes
            ),
            'finite_points': finite_points,
            'total_points': total_points,
        }

    def _inside_body_filter(self, x, y):
        return (
            self.body_x_min <= x <= self.body_x_max
            and self.body_y_min <= y <= self.body_y_max
        )

    def _classify_sector(self, angle):
        """按水平角将点分配到前、斜前和侧方，后方点不参与判断。"""
        half_width = 0.5 * self._front_angle_rad

        if -half_width <= angle <= half_width:
            return 'front'
        if half_width < angle <= self._diagonal_angle_max_rad:
            return 'left_front'
        if -self._diagonal_angle_max_rad <= angle < -half_width:
            return 'right_front'
        if (
            self._diagonal_angle_max_rad
            < angle
            <= self._side_angle_max_rad
        ):
            return 'left'
        if (
            -self._side_angle_max_rad
            <= angle
            < -self._diagonal_angle_max_rad
        ):
            return 'right'
        return None

    def _classify_projected_side(self, x, y, angle):
        """识别可投影为左右墙净距的前向斜视点。"""
        if not (
            self.side_projection_x_min
            <= x
            <= self.side_projection_x_max
        ):
            return None
        absolute_angle = abs(angle)
        if not (
            self._side_projection_angle_min_rad
            <= absolute_angle
            <= self._side_projection_angle_max_rad
        ):
            return None
        if y > 0.0:
            return 'left'
        if y < 0.0:
            return 'right'
        return None

    def _select_projected_side_candidates(self, candidates):
        """返回强侧墙簇及仅可用于历史连续性确认的短墙簇。"""
        if len(candidates) < self.side_min_points:
            return (), ()

        best_cluster = ()
        best_score = None
        short_clusters = []
        tolerance = self.side_projection_lateral_tolerance
        # 两组错半格的横向桶避免墙面刚好落在分桶边界，同时保持 O(n)。
        for offset in (0.0, 0.5 * tolerance):
            clusters = {}
            for candidate in candidates:
                bucket = int(math.floor(
                    (candidate[2] + offset) / tolerance
                ))
                clusters.setdefault(bucket, []).append(candidate)

            for cluster in clusters.values():
                if len(cluster) < self.side_min_points:
                    continue

                x_values = [x for _, x, _ in cluster]
                robust_x_span = (
                    self._percentile(x_values, 90.0)
                    - self._percentile(x_values, 10.0)
                )
                if robust_x_span < self.side_projection_min_x_span:
                    short_clusters.append((
                        tuple(cluster),
                        robust_x_span,
                    ))
                    continue

                # 优先使用支持点更多、纵向跨度更大的稳定墙面簇。
                score = (len(cluster), robust_x_span)
                if best_score is None or score > best_score:
                    best_cluster = tuple(cluster)
                    best_score = score
        return best_cluster, tuple(short_clusters)

    @staticmethod
    def _percentile(values, percentile):
        """采用线性插值计算百分位数，兼容扇区内点数较少的情况。"""
        if not values:
            return None
        ordered = sorted(values)
        rank = (len(ordered) - 1) * float(percentile) / 100.0
        lower_index = int(math.floor(rank))
        upper_index = int(math.ceil(rank))
        if lower_index == upper_index:
            return ordered[lower_index]
        fraction = rank - lower_index
        return (
            ordered[lower_index] * (1.0 - fraction)
            + ordered[upper_index] * fraction
        )

    def _validate(self):
        # 启动时尽早拒绝不合理参数，避免运行中产生不可预测判断。
        values = (
            self.z_min,
            self.z_max,
            self.body_x_min,
            self.body_x_max,
            self.body_y_min,
            self.body_y_max,
            self.front_angle_deg,
            self.min_range,
            self.front_max_range,
            self.side_max_range,
            self.distance_percentile,
            self.diagonal_angle_max_deg,
            self.side_angle_max_deg,
            self.side_projection_angle_min_deg,
            self.side_projection_angle_max_deg,
            self.side_projection_x_min,
            self.side_projection_x_max,
            self.side_projection_min_x_span,
            self.side_projection_lateral_tolerance,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError('sector parameters must be finite')
        if self.z_max <= self.z_min:
            raise ValueError('z_max must be greater than z_min')
        if self.body_x_max <= self.body_x_min:
            raise ValueError('body_x_max must be greater than body_x_min')
        if self.body_y_max <= self.body_y_min:
            raise ValueError('body_y_max must be greater than body_y_min')
        if not 0.0 < self.front_angle_deg <= 72.0:
            raise ValueError('front_angle must be in (0, 72] degrees')
        if not (
            0.5 * self.front_angle_deg
            < self.diagonal_angle_max_deg
            < self.side_angle_max_deg
            <= 180.0
        ):
            raise ValueError(
                'sector angles must satisfy '
                'front_angle/2 < diagonal_angle_max '
                '< side_angle_max <= 180 degrees'
            )
        if self.min_range < 0.0:
            raise ValueError('min_range must be nonnegative')
        if self.front_max_range <= self.min_range:
            raise ValueError(
                'front_max_range must be greater than min_range'
            )
        if self.side_max_range <= self.min_range:
            raise ValueError(
                'side_max_range must be greater than min_range'
            )
        if not 0.0 < self.distance_percentile <= 100.0:
            raise ValueError('distance_percentile must be in (0, 100]')
        if not (
            0.0
            <= self.side_projection_angle_min_deg
            < self.side_projection_angle_max_deg
            <= 90.0
        ):
            raise ValueError(
                'side projection angles must satisfy '
                '0 <= min < max <= 90 degrees'
            )
        if self.side_projection_x_min < self.body_x_max:
            raise ValueError(
                'side_projection_x_min must not be less than '
                'body_x_max'
            )
        if (
            self.side_projection_x_max
            <= self.side_projection_x_min
        ):
            raise ValueError(
                'side_projection_x_max must be greater than '
                'side_projection_x_min'
            )
        if self.side_projection_min_x_span <= 0.0:
            raise ValueError(
                'side_projection_min_x_span must be positive'
            )
        if (
            self.side_projection_min_x_span
            > self.side_projection_x_max
            - self.side_projection_x_min
        ):
            raise ValueError(
                'side_projection_min_x_span must fit inside '
                'the side projection x window'
            )
        if not (
            0.0
            < self.side_projection_lateral_tolerance
            <= self.side_max_range
        ):
            raise ValueError(
                'side_projection_lateral_tolerance must be in '
                '(0, side_max_range]'
            )
        if self.side_min_points <= 0:
            raise ValueError('side_min_points must be positive')


class SideDistanceStabilizer:
    """保守稳定稀疏侧墙量测，并显式标记保留来源和帧龄。"""

    def __init__(
        self,
        hold_frames,
        rise_tolerance_m=0.08,
        continuity_tolerance_m=0.04,
    ):
        self.hold_frames = int(hold_frames)
        if self.hold_frames < 0:
            raise ValueError('side hold frames must be nonnegative')
        self.rise_tolerance_m = float(rise_tolerance_m)
        if (
            not math.isfinite(self.rise_tolerance_m)
            or self.rise_tolerance_m < 0.0
        ):
            raise ValueError(
                'side rise tolerance must be finite and nonnegative'
            )
        self.continuity_tolerance_m = float(continuity_tolerance_m)
        if (
            not math.isfinite(self.continuity_tolerance_m)
            or self.continuity_tolerance_m < 0.0
        ):
            raise ValueError(
                'side continuity tolerance must be finite and nonnegative'
            )
        self.reset()

    def reset(self):
        """传感器断流或输入损坏时清空缓存，禁止复用过期墙距。"""
        self._cached_distances = {'left': None, 'right': None}
        self._cached_sources = {'left': 'none', 'right': 'none'}
        self._unconfirmed_frames = {'left': 0, 'right': 0}

    def update(self, extraction):
        """返回包含侧距短时保留结果的新提取快照。"""
        if not isinstance(extraction, dict):
            raise ValueError('extraction must be a dict')
        distances = dict(extraction.get('distances', {}))
        counts = dict(extraction.get('counts', {}))
        sources = dict(extraction.get('sources', {}))
        continuity_candidates = dict(
            extraction.get('side_continuity_candidates', {})
        )
        hold_ages = {name: 0 for name in SECTOR_NAMES}

        for name in ('left', 'right'):
            distance = distances.get(name)
            if distance is not None:
                value = float(distance)
                if not math.isfinite(value) or value < 0.0:
                    raise ValueError(f'{name} distance must be finite')

                cached = self._cached_distances[name]
                if (
                    cached is None
                    or value <= cached + self.rise_tolerance_m
                ):
                    # 更近障碍必须立即生效；正常小幅噪声也直接更新。
                    self._accept_measurement(
                        name,
                        value,
                        sources.get(name, 'unknown'),
                    )
                    continue

                # 近墙回波偶发消失时，远墙点仍是有限值而非 n/a。
                # 突然变远必须连续确认，避免将远处挡板误当成侧墙开口。
                self._unconfirmed_frames[name] += 1
                age = self._unconfirmed_frames[name]
                if age <= self.hold_frames:
                    distances[name] = cached
                    sources[name] = (
                        'held_rise_' + self._cached_sources[name]
                    )
                    hold_ages[name] = age
                    continue

                self._accept_measurement(
                    name,
                    value,
                    sources.get(name, 'unknown'),
                )
                continue

            cached = self._cached_distances[name]
            continuity = self._matching_continuity_candidate(
                cached,
                continuity_candidates.get(name, ()),
            )
            if continuity is not None:
                # 短墙只能延续已确认侧墙，且不得据此增大安全净空。
                # 每帧都有当前点云支持，因此不消耗盲保留帧数。
                continued_distance = min(
                    cached,
                    continuity['distance'],
                )
                distances[name] = continued_distance
                counts[name] = continuity['count']
                sources[name] = 'continued_projected'
                self._cached_distances[name] = continued_distance
                self._cached_sources[name] = 'projected'
                self._unconfirmed_frames[name] = 0
                continue

            self._unconfirmed_frames[name] += 1
            age = self._unconfirmed_frames[name]
            if cached is not None and age <= self.hold_frames:
                distances[name] = cached
                counts[name] = 0
                sources[name] = 'held_' + self._cached_sources[name]
                hold_ages[name] = age
            else:
                distances[name] = None
                counts[name] = int(counts.get(name, 0))
                sources[name] = 'none'
                self._cached_distances[name] = None
                self._cached_sources[name] = 'none'
                self._unconfirmed_frames[name] = 0

        result = dict(extraction)
        result['distances'] = distances
        result['counts'] = counts
        result['sources'] = sources
        result['hold_frames'] = hold_ages
        return result

    def _matching_continuity_candidate(self, cached, candidates):
        """选择与强确认墙距一致的短墙候选，禁止无历史值自启动。"""
        if cached is None:
            return None

        best_candidate = None
        best_score = None
        for candidate in candidates:
            try:
                distance = float(candidate['distance'])
                count = int(candidate['count'])
                x_span = float(candidate['x_span'])
            except (KeyError, TypeError, ValueError) as error:
                raise ValueError(
                    'invalid side continuity candidate'
                ) from error
            if (
                not math.isfinite(distance)
                or distance < 0.0
                or count <= 0
                or not math.isfinite(x_span)
                or x_span < 0.0
            ):
                raise ValueError('invalid side continuity candidate')

            difference = abs(distance - cached)
            if difference > self.continuity_tolerance_m:
                continue
            # 距离最接近历史墙面优先，其次选择支持点和纵向跨度更多的簇。
            score = (difference, -count, -x_span)
            if best_score is None or score < best_score:
                best_candidate = {
                    'distance': distance,
                    'count': count,
                    'x_span': x_span,
                }
                best_score = score
        return best_candidate

    def _accept_measurement(self, name, distance, source):
        """接受已确认量测，并重新开始该侧的保守确认窗口。"""
        self._cached_distances[name] = float(distance)
        self._cached_sources[name] = str(source)
        self._unconfirmed_frames[name] = 0


class DryRunDecisionEngine:
    """通过持续帧、距离滞回和转向锁定生成只读避障建议。"""

    def __init__(
        self,
        front_max_range,
        side_max_range,
        front_block_enter,
        front_block_exit,
        diagonal_block_enter,
        diagonal_block_exit,
        blocked_confirm_frames,
        clear_confirm_frames,
        turn_min_clearance,
        turn_switch_margin,
        preferred_turn,
    ):
        self.front_max_range = float(front_max_range)
        self.side_max_range = float(side_max_range)
        self.front_block_enter = float(front_block_enter)
        self.front_block_exit = float(front_block_exit)
        self.diagonal_block_enter = float(diagonal_block_enter)
        self.diagonal_block_exit = float(diagonal_block_exit)
        self.blocked_confirm_frames = int(blocked_confirm_frames)
        self.clear_confirm_frames = int(clear_confirm_frames)
        self.turn_min_clearance = float(turn_min_clearance)
        self.turn_switch_margin = float(turn_switch_margin)
        self.preferred_turn = str(preferred_turn).lower()
        self._validate()

        # 启动时尚未确认传感器有效，因此默认 STALE/STOP。
        self.state = STATE_STALE
        self.advice = ADVICE_STOP
        self.reason = 'startup'
        self.blocked_streak = 0
        self.clear_streak = 0
        self._latched_turn = None

    def mark_stale(self, reason):
        """任一必需传感器失效时立即回到保守状态并清空历史确认。"""
        self.state = STATE_STALE
        self.advice = ADVICE_STOP
        self.reason = str(reason)
        self.blocked_streak = 0
        self.clear_streak = 0
        self._latched_turn = None

    def update(self, distances):
        blocked_candidate = self._blocked_candidate(distances)
        if blocked_candidate:
            self._update_blocked(distances)
        else:
            self._update_clear()
        return self.state, self.advice, self.reason

    def _update_blocked(self, distances):
        # 障碍必须连续出现指定帧数；确认期间只给 STOP 建议。
        self.blocked_streak += 1
        self.clear_streak = 0

        if self.state == STATE_BLOCKED:
            self.advice = self._choose_turn(distances)
            self.reason = 'blocked_latched'
            return

        if self.blocked_streak < self.blocked_confirm_frames:
            self.advice = ADVICE_STOP
            self.reason = (
                'blocked_confirmation_'
                f'{self.blocked_streak}/{self.blocked_confirm_frames}'
            )
            return

        self.state = STATE_BLOCKED
        self.advice = self._choose_turn(distances)
        self.reason = 'blocked_confirmed'

    def _update_clear(self):
        # 清障也要连续确认，防止阈值附近的点云抖动导致状态反复。
        self.clear_streak += 1
        self.blocked_streak = 0

        if self.state == STATE_CLEAR:
            self.advice = ADVICE_FORWARD
            self.reason = 'clear_latched'
            return

        if self.clear_streak < self.clear_confirm_frames:
            # 从 BLOCKED 恢复期间保持原转向建议，不提前给 FORWARD。
            if self.state == STATE_BLOCKED:
                self.advice = self._latched_turn or ADVICE_STOP
            else:
                self.advice = ADVICE_STOP
            self.reason = (
                'clear_confirmation_'
                f'{self.clear_streak}/{self.clear_confirm_frames}'
            )
            return

        self.state = STATE_CLEAR
        self.advice = ADVICE_FORWARD
        self.reason = 'clear_confirmed'
        self._latched_turn = None

    def _blocked_candidate(self, distances):
        # 已进入 BLOCKED 后改用更大的退出阈值，形成距离滞回区间。
        if self.state == STATE_BLOCKED:
            front_threshold = self.front_block_exit
            diagonal_threshold = self.diagonal_block_exit
        else:
            front_threshold = self.front_block_enter
            diagonal_threshold = self.diagonal_block_enter

        front = self._effective_distance(
            distances.get('front'),
            self.front_max_range,
        )
        left_front = self._effective_distance(
            distances.get('left_front'),
            self.front_max_range,
        )
        right_front = self._effective_distance(
            distances.get('right_front'),
            self.front_max_range,
        )
        return (
            front <= front_threshold
            or left_front <= diagonal_threshold
            or right_front <= diagonal_threshold
        )

    def _choose_turn(self, distances):
        # 每侧取“斜前与正侧”的较小值，保证整条转向通道都有余量。
        left_score = min(
            self._effective_distance(
                distances.get('left_front'),
                self.front_max_range,
            ),
            self._effective_distance(
                distances.get('left'),
                self.side_max_range,
            ),
        )
        right_score = min(
            self._effective_distance(
                distances.get('right_front'),
                self.front_max_range,
            ),
            self._effective_distance(
                distances.get('right'),
                self.side_max_range,
            ),
        )

        # 左右均过窄时不猜测方向，直接输出最保守的 STOP 建议。
        if (
            left_score < self.turn_min_clearance
            and right_score < self.turn_min_clearance
        ):
            self._latched_turn = ADVICE_STOP
            return ADVICE_STOP

        # 已选方向会被锁定；另一侧必须明显更优才允许切换。
        if (
            self._latched_turn == ADVICE_TURN_LEFT
            and left_score >= self.turn_min_clearance
            and right_score <= left_score + self.turn_switch_margin
        ):
            return ADVICE_TURN_LEFT
        if (
            self._latched_turn == ADVICE_TURN_RIGHT
            and right_score >= self.turn_min_clearance
            and left_score <= right_score + self.turn_switch_margin
        ):
            return ADVICE_TURN_RIGHT

        if left_score > right_score + self.turn_switch_margin:
            self._latched_turn = ADVICE_TURN_LEFT
        elif right_score > left_score + self.turn_switch_margin:
            self._latched_turn = ADVICE_TURN_RIGHT
        elif (
            left_score >= self.turn_min_clearance
            and right_score >= self.turn_min_clearance
        ):
            self._latched_turn = (
                ADVICE_TURN_LEFT
                if self.preferred_turn == 'left'
                else ADVICE_TURN_RIGHT
            )
        elif left_score >= self.turn_min_clearance:
            self._latched_turn = ADVICE_TURN_LEFT
        elif right_score >= self.turn_min_clearance:
            self._latched_turn = ADVICE_TURN_RIGHT
        else:
            self._latched_turn = ADVICE_STOP
        return self._latched_turn

    @staticmethod
    def _effective_distance(distance, max_range):
        # 扇区无回波不等同于整帧失效，此处按可探测最大距离处理。
        if distance is None:
            return float(max_range)
        value = float(distance)
        if not math.isfinite(value):
            return float(max_range)
        return value

    def _validate(self):
        # 进入阈值必须小于退出阈值，才能形成有效滞回。
        values = (
            self.front_max_range,
            self.side_max_range,
            self.front_block_enter,
            self.front_block_exit,
            self.diagonal_block_enter,
            self.diagonal_block_exit,
            self.turn_min_clearance,
            self.turn_switch_margin,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError('decision parameters must be finite')
        if self.front_max_range <= 0.0 or self.side_max_range <= 0.0:
            raise ValueError('maximum ranges must be positive')
        if not 0.0 < self.front_block_enter < self.front_block_exit:
            raise ValueError(
                'front block thresholds must satisfy 0 < enter < exit'
            )
        if self.front_block_exit > self.front_max_range:
            raise ValueError(
                'front_block_exit must not exceed front_max_range'
            )
        if not 0.0 < self.diagonal_block_enter:
            raise ValueError('diagonal_block_enter must be positive')
        if self.diagonal_block_enter >= self.diagonal_block_exit:
            raise ValueError(
                'diagonal thresholds must satisfy enter < exit'
            )
        if self.diagonal_block_exit > self.front_max_range:
            raise ValueError(
                'diagonal_block_exit must not exceed front_max_range'
            )
        if self.blocked_confirm_frames <= 0:
            raise ValueError('blocked_confirm_frames must be positive')
        if self.clear_confirm_frames <= 0:
            raise ValueError('clear_confirm_frames must be positive')
        if self.turn_min_clearance <= 0.0:
            raise ValueError('turn_min_clearance must be positive')
        if self.turn_switch_margin < 0.0:
            raise ValueError('turn_switch_margin must be nonnegative')
        if self.preferred_turn not in ('left', 'right'):
            raise ValueError('preferred_turn must be left or right')


def quaternion_to_rpy(x, y, z, w):
    """归一化里程计四元数并转换为 roll、pitch、yaw。"""
    values = (float(x), float(y), float(z), float(w))
    if not all(math.isfinite(value) for value in values):
        raise ValueError('odometry quaternion contains NaN or Inf')

    x, y, z, w = values
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1.0e-12:
        raise ValueError('odometry quaternion has zero norm')

    # 先归一化，避免非单位四元数放大姿态转换误差。
    x /= norm
    y /= norm
    z /= norm
    w /= norm

    sin_roll_cos_pitch = 2.0 * (w * x + y * z)
    cos_roll_cos_pitch = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll_cos_pitch, cos_roll_cos_pitch)

    sin_pitch = 2.0 * (w * y - z * x)
    # 浮点舍入可能略微越过 [-1, 1]，限幅后再调用 asin。
    sin_pitch = max(-1.0, min(1.0, sin_pitch))
    pitch = math.asin(sin_pitch)

    sin_yaw_cos_pitch = 2.0 * (w * z + x * y)
    cos_yaw_cos_pitch = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(sin_yaw_cos_pitch, cos_yaw_cos_pitch)
    return roll, pitch, yaw


def normalize_angle(angle):
    """将角差归一化到 [-pi, pi]，用于消除正负 pi 跳变。"""
    return math.atan2(math.sin(angle), math.cos(angle))
