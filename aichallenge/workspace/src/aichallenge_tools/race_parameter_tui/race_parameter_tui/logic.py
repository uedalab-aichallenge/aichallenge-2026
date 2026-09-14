"""ROS-independent pieces of the parameter TUI.

Keeping candidate discovery and input conversion here makes the safety-critical
parts straightforward to test without a running ROS graph.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import re
from typing import Iterable


CANDIDATE_NODE_NAMES = (
    'simple_pure_pursuit',
    'reverse_pure_pursuit',
    'v2x_overtaker',
    'stuck_recovery_controller',
)

PURE_PURSUIT_PARAMETERS = frozenset(
    {
        'wheel_base',
        'lookahead_gain',
        'lookahead_min_distance',
        'speed_proportional_gain',
        'use_external_target_vel',
        'external_target_vel',
        'steering_tire_angle_gain',
        'lookahead_cte_gain',
        'lookahead_slow_speed',
        'lookahead_curve_k',
        'lookahead_slow_min',
        'lookahead_slow_exp',
        'lookahead_overtake_scale',
        'lookahead_scale_zones',
        'lookahead_curve_ahead',
        'start_steer_speed',
        'start_steer_limit',
        'stuck_steer_free_speed',
        'max_acceleration',
    }
)

SIMPLE_PURE_PURSUIT_PARAMETERS = PURE_PURSUIT_PARAMETERS | frozenset(
    {
        'sat_accel_guard',
        'sat_steer_rad',
        'sat_accel_max',
        'sat_guard_min_speed',
        'wall_guard_clamp_enable',
        'wall_guard_stale_sec',
    }
)

REVERSE_PURE_PURSUIT_PARAMETERS = PURE_PURSUIT_PARAMETERS | frozenset(
    {
        'lookahead_curve_ref',
        'lookahead_curve_min',
        'lookahead_slow_full',
        'lookahead_slow_far',
        'reverse_max_speed',
    }
)

# These are intentionally curated rather than displaying every implementation
# parameter.  The intersection with the remote node's declared names below is
# the final allowlist, so stale names never appear in the UI.
V2X_PARAMETERS = frozenset(
    {
        'detect_range',
        'front_lane_half',
        'contact_vehicle_dist',
        'avoid_range',
        'collision_radius',
        'ttc_threshold',
        'big_gap_closing',
        'inside_time_gain',
        'inside_width_gain',
        'latch_width_gain',
        'pass_gap',
        'pass_gap_clear_ratio',
        'pass_side_clear',
        'pass_beside_sep',
        'offset_rate',
        'corridor_safety',
        'corridor_safety_pass',
        'corridor_safety_zone',
        'side_room_ahead',
        'funnel_ahead_m',
        'funnel_speed_floor',
        'side_flip_hold',
        'side_flip_max',
        'side_window_search_m',
        'zone_look_ahead',
        'window_full',
        'window_end',
        'window_back',
        'start_merge_dist',
        'start_lat_max',
        'stop_hold_gap',
        'stop_hold_sec',
        'stop_hold_move',
        'slow_leader_speed',
        'attempt_timeout',
        'pass_len',
        'pass_done_len',
        'spot_here_range',
        'pass_done_sec',
        'pen_speed',
        'wall_guard_enable',
        'wall_guard_horizon',
        'wall_guard_dt',
        'wall_guard_margin',
        'wall_guard_run',
        'wall_guard_ay_max',
        'boost_retry_sec',
        'boost_min_speed',
        'boost_accel',
        'boost_min_headroom',
        'vehicle_accel',
        'zone_exit_margin',
        'leader_speed_cap',
        'rank1_corner_gain',
        'rank2_corner_gain',
        'rank2_speed_cap',
        'final_dash_dist',
        'final_dash_gap',
        'free_boost_gap',
        'free_boost_min_speed',
        'free_boost_clear_ahead',
        'free_boost_straight',
        'free_boost_headroom',
        'free_boost_skip',
        'wedge_ttc',
        'wedge_room',
    }
)

STUCK_RECOVERY_PARAMETERS = frozenset(
    {
        'reverse_traj_enable',
        'wedge_backward_first',
        'rev_gear_fix',
        'rev_cmd_hold_sec',
        'wall_forward_ban',
        'wall_forward_ban_hold',
        'wall_forward_ban_gain',
        'wall_forward_ban_speed',
        'wall_forward_ban_depth',
        'wall_ban_reverse',
        'wall_ban_reverse_after',
        'wall_ban_reverse_speed',
        'wall_ban_reverse_rear',
        'queue_wait_enable',
        'queue_wait_margin',
        'queue_wait_yaw',
        'queue_wait_wall',
        'queue_wait_max',
        'queue_wait_max_stopped',
        'queue_wait_front_only',
        'deadlock_release_enable',
        'deadlock_release_sec',
        'deadlock_release_accel',
        'recovery_speed',
        'recovery_accel',
        'embed_min_reverse',
        'plan_steer_kick',
        'plan_steer_kick_m',
        'plan_steer_kick_speed',
        'plan_steer_kick_accel',
        'plan_steer_kick_sec',
        'plan_steer_band',
        'plan_local_steer',
        'recovery_simple',
        'simple_ahead_m',
        'simple_back_m',
        'simple_probe_m',
        'simple_reach_m',
        'simple_speed',
        'simple_accel',
        'simple_steer_steps',
        'simple_steer_frac',
        'simple_stall_sec',
        'simple_stall_min_m',
        'embed_prefer_reverse',
        'turn_in_abort',
        'turn_in_abort_travel',
        'turn_in_abort_worsen',
        'steer_cmd_scale',
        'reachable_wall_guard_enable',
        'reachable_wall_horizon',
        'reachable_wall_step',
        'reachable_wall_delay',
        'reachable_wall_steering_rate',
        'reachable_wall_spatial_step',
        'reachable_wall_margin',
        'reachable_wall_escape_improvement',
        'reachable_wall_candidates',
        'goal_plan_enable',
        'desperate_enable',
        'path_check_enable',
        'path_check_bad_sec',
    }
)

PARAMETER_ALLOWLIST = {
    'simple_pure_pursuit': SIMPLE_PURE_PURSUIT_PARAMETERS,
    'reverse_pure_pursuit': REVERSE_PURE_PURSUIT_PARAMETERS,
    'v2x_overtaker': V2X_PARAMETERS,
    'stuck_recovery_controller': STUCK_RECOVERY_PARAMETERS,
}

PARAMETER_TYPE_NAMES = {
    1: 'bool',
    2: 'int',
    3: 'double',
    4: 'string',
}

_INTEGER_RE = re.compile(r'^[+-]?\d+$')


@dataclass(frozen=True)
class NodeCandidate:
    name: str
    namespace: str

    @property
    def full_name(self) -> str:
        namespace = self.namespace.rstrip('/')
        return f'{namespace}/{self.name}' if namespace else f'/{self.name}'


@dataclass
class ParameterEntry:
    name: str
    type_name: str
    value: object


def base_node_name(name: str) -> str:
    return name.strip('/').split('/')[-1]


def normalize_namespace(namespace: str) -> str:
    if not namespace or namespace == '/':
        return ''
    return '/' + namespace.strip('/')


def discover_candidate_nodes(
    graph: Iterable[tuple[str, str]],
) -> list[NodeCandidate]:
    """Return matching graph nodes, preserving vehicle namespaces."""
    found = {
        NodeCandidate(base_node_name(name), normalize_namespace(namespace))
        for name, namespace in graph
        if base_node_name(name) in CANDIDATE_NODE_NAMES
    }
    return sorted(found, key=lambda item: item.full_name)


def allowed_parameter_names(node_name: str, available_names: Iterable[str]) -> list[str]:
    allowed = PARAMETER_ALLOWLIST.get(base_node_name(node_name), frozenset())
    return sorted({name for name in available_names if name in allowed})


def visible_parameter_slice(
    item_count: int,
    selected_index: int,
    viewport_size: int,
    scroll_offset: int = 0,
) -> tuple[int, int]:
    """Return the visible ``[start, end)`` range for a parameter list.

    The selected item is kept visible while retaining the existing scroll
    position whenever possible.  Keeping this calculation independent from
    curses makes the terminal scrolling behavior easy to test.
    """
    if item_count <= 0 or viewport_size <= 0:
        return 0, 0

    selected = max(0, min(selected_index, item_count - 1))
    max_offset = max(item_count - viewport_size, 0)
    start = max(0, min(scroll_offset, max_offset))
    if selected < start:
        start = selected
    elif selected >= start + viewport_size:
        start = selected - viewport_size + 1

    end = min(start + viewport_size, item_count)
    return start, end


def type_name_from_parameter_value(parameter_value: object) -> str | None:
    return PARAMETER_TYPE_NAMES.get(getattr(parameter_value, 'type', None))


def value_from_parameter_value(parameter_value: object) -> object:
    type_name = type_name_from_parameter_value(parameter_value)
    fields = {
        'bool': 'bool_value',
        'int': 'integer_value',
        'double': 'double_value',
        'string': 'string_value',
    }
    if type_name is None:
        raise ValueError('unsupported ROS parameter type')
    return getattr(parameter_value, fields[type_name])


def parse_parameter_text(text: str, type_name: str) -> object:
    """Convert one user-entered scalar, rejecting ambiguous/unsafe values."""
    if type_name == 'string':
        return text
    if type_name == 'bool':
        normalized = text.strip().lower()
        if normalized in {'true', '1', 'yes', 'on'}:
            return True
        if normalized in {'false', '0', 'no', 'off'}:
            return False
        raise ValueError('bool must be true/false')
    if type_name == 'int':
        if not _INTEGER_RE.fullmatch(text.strip()):
            raise ValueError('int must be a decimal integer')
        value = int(text.strip(), 10)
        if not -(1 << 63) <= value < (1 << 63):
            raise ValueError('int is outside int64 range')
        return value
    if type_name == 'double':
        try:
            value = float(text.strip())
        except ValueError as exc:
            raise ValueError('double must be numeric') from exc
        if not math.isfinite(value):
            raise ValueError('double must be finite')
        return value
    raise ValueError(f'unsupported parameter type: {type_name}')


def format_parameter_value(value: object, type_name: str) -> str:
    if type_name == 'bool':
        return 'true' if value else 'false'
    if type_name == 'double':
        return f'{value:.8g}'
    return str(value)
