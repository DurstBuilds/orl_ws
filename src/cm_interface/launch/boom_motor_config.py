# Single source of truth for boom stack motor list.
# Imported by boom_stack.launch.py and can_gateway.launch.py.
#
# Per-entry fields (TWEAK when adding a motor):
#   ns                      — ROS namespace (teleop uses substring: knee/wheel/hip)
#   gear_ratio              — motor rad per joint rad; must match translator + teleop
#   motor_model             — ak70_10 | ak10_9 | ak80_64
#   can_id                  — unique on bus [0, 2047]
#   joint_angle_limit_deg   — translator clamp; 0 disables; hip also uses hip_angle_limit_deg launch arg

BOOM_MOTOR_STACKS = (
    {
        'ns': 'knee_motor',
        'gear_ratio': 1.6,
        'motor_model': 'ak80_64',
        'can_id': 4,
        'joint_angle_limit_deg': 0.0,
    },
    {
        'ns': 'hip_motor',
        'gear_ratio': 33.0,
        'motor_model': 'ak70_10',
        'can_id': 3,
        'joint_angle_limit_deg': 45.0,
    },
    {
        'ns': 'wheel_motor1',
        'gear_ratio': 1.0,
        'motor_model': 'ak10_9',
        'can_id': 1,
        'joint_angle_limit_deg': 0.0,
    },
    {
        'ns': 'wheel_motor2',
        'gear_ratio': 1.0,
        'motor_model': 'ak10_9',
        'can_id': 2,
        'joint_angle_limit_deg': 0.0,
    },
)

DEFAULT_NAMESPACES = ','.join(stack['ns'] for stack in BOOM_MOTOR_STACKS)
DEFAULT_MOTOR_MODELS = ','.join(stack['motor_model'] for stack in BOOM_MOTOR_STACKS)
DEFAULT_CAN_IDS = ','.join(str(stack['can_id']) for stack in BOOM_MOTOR_STACKS)
DEFAULT_NAMESPACE_GEAR_RATIOS = ','.join(
    f"{stack['ns']}:{stack['gear_ratio']}" for stack in BOOM_MOTOR_STACKS
)
MOTOR_STATE_TOPICS = [f'/{stack["ns"]}/motor_state' for stack in BOOM_MOTOR_STACKS]
