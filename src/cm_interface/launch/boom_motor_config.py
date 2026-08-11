# Single source of truth for boom stack motor list.
# Imported by boom_stack.launch.py and can_gateway.launch.py.
#
# Per-entry fields (TWEAK when adding a motor):
#   ns                      — ROS namespace (teleop uses substring: knee/wheel/hip)
#   gear_ratio              — motor rad per joint rad; must match translator + teleop
#   motor_model             — ak70_10 | ak10_9 | ak80_64
#   can_id                  — unique on bus [0, 2047]
#   joint_angle_limit_deg   — translator clamp; 0 disables; hip also uses hip_angle_limit_deg launch arg
#   omega_max               — optional; joint_translator cap: "auto" or motor rad/s (see namespace_omega_max)

BOOM_MOTOR_STACKS = (
    {
        'ns': 'knee_motor',
        'gear_ratio': 1.6,
        'motor_model': 'ak80_64',
        'can_id': 4,
        'joint_angle_limit_deg': 0.0,
        'omega_max': 'auto',
    },
    {
        'ns': 'hip_motor',
        'gear_ratio': 33.0,
        'motor_model': 'ak70_10',
        'can_id': 3,
        'joint_angle_limit_deg': 90.0,
        'omega_max': 'auto',
    },
    {
        'ns': 'wheel_motor1',
        'gear_ratio': 1.0,
        'motor_model': 'ak10_9',
        'can_id': 1,
        'joint_angle_limit_deg': 0.0,
        'omega_max': 'auto',
    },
    {
        'ns': 'wheel_motor2',
        'gear_ratio': 1.0,
        'motor_model': 'ak10_9',
        'can_id': 2,
        'joint_angle_limit_deg': 0.0,
        'omega_max': 'auto',
    },
)

DEFAULT_NAMESPACES = ','.join(stack['ns'] for stack in BOOM_MOTOR_STACKS)
DEFAULT_MOTOR_MODELS = ','.join(stack['motor_model'] for stack in BOOM_MOTOR_STACKS)
DEFAULT_CAN_IDS = ','.join(str(stack['can_id']) for stack in BOOM_MOTOR_STACKS)
DEFAULT_NAMESPACE_GEAR_RATIOS = ','.join(
    f"{stack['ns']}:{stack['gear_ratio']}" for stack in BOOM_MOTOR_STACKS
)
DEFAULT_NAMESPACE_OMEGA_MAX = ','.join(
    f"{stack['ns']}:{stack.get('omega_max', 'auto')}" for stack in BOOM_MOTOR_STACKS
)
MOTOR_STATE_TOPICS = [f'/{stack["ns"]}/motor_state' for stack in BOOM_MOTOR_STACKS]
JOINT_CURPOS_TOPICS = [f'/{stack["ns"]}/joint_curpos' for stack in BOOM_MOTOR_STACKS]
MOTOR_COMMAND_TOPICS = [f'/{stack["ns"]}/motor_command' for stack in BOOM_MOTOR_STACKS]

# Shared launch defaults (boom_stack, can_gateway, motor_stack, boom_teleop).
DEFAULT_MOTOR_ERROR_TOLERANCE = '0.001'
DEFAULT_MOTOR_FEEDBACK_TIMEOUT_MS = '250'
DEFAULT_MOTOR_FEEDBACK_POLL_MS = '5'
DEFAULT_GATEWAY_LOOP_RATE_HZ = '200.0'
DEFAULT_GATEWAY_STARTUP_STAGGER_MS = '200'
DEFAULT_GATEWAY_ENABLE_SETTLE_MS = '100'
DEFAULT_GATEWAY_AK80_ENABLE_SETTLE_MS = '250'
DEFAULT_GATEWAY_STARTUP_ORIGIN_POLL_MS = '100'
DEFAULT_GATEWAY_BUS_WARMUP_MS = '100'
DEFAULT_GATEWAY_STANDBY_RETRY_MS = '5000'
DEFAULT_START_IN_SOFT_MODE = 'true'

DEFAULT_ORIGIN_JUMP_THRESHOLD = 2.0
DEFAULT_AK80_ORIGIN_JUMP_THRESHOLD = 4.0


def origin_jump_threshold_for_stack(stack: dict) -> float:
    """Unwrapper jump-reset threshold (rad); AK80 knee uses a higher cap."""
    if stack.get('motor_model') == 'ak80_64':
        return DEFAULT_AK80_ORIGIN_JUMP_THRESHOLD
    return DEFAULT_ORIGIN_JUMP_THRESHOLD


def origin_jump_threshold_for_motor_model(motor_model: str) -> float:
    """Unwrapper jump-reset threshold for single-motor bench launches."""
    if motor_model.strip() == 'ak80_64':
        return DEFAULT_AK80_ORIGIN_JUMP_THRESHOLD
    return DEFAULT_ORIGIN_JUMP_THRESHOLD


def parse_namespace_omega_max(param: str, default: str = 'auto') -> dict[str, str]:
    """Parse 'ns1:auto,ns2:8.0' into namespace -> omega_max string map."""
    default = default.strip()
    _validate_omega_max_value(default, label='omega_max default')

    omega_by_ns: dict[str, str] = {}
    if not param.strip():
        return omega_by_ns
    for entry in param.split(','):
        item = entry.strip()
        if not item:
            continue
        if ':' not in item:
            raise ValueError(
                f"namespace_omega_max entry '{item}' must be namespace:omega_max"
            )
        ns, value = item.split(':', 1)
        ns = ns.strip()
        value = value.strip()
        if not ns:
            raise ValueError('namespace_omega_max namespace must be non-empty')
        _validate_omega_max_value(value, label=f'namespace_omega_max for {ns}')
        omega_by_ns[ns] = value
    return omega_by_ns


def _validate_omega_max_value(value: str, *, label: str) -> None:
    if value.lower() == 'auto':
        return
    try:
        numeric = float(value)
    except ValueError as exc:
        raise ValueError(
            f"{label}: omega_max must be 'auto' or a positive number, got '{value}'"
        ) from exc
    if numeric <= 0.0:
        raise ValueError(f'{label}: omega_max must be > 0, got {numeric}')


def omega_max_for_namespace(
    namespace: str,
    per_namespace: dict[str, str],
    default: str,
) -> str:
    """Resolve omega_max for one stack entry."""
    if namespace in per_namespace:
        return per_namespace[namespace]
    return default
