import math


def normalize_angle(angle):
    """Normalize angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def shooter_position(robot_pos, offset, robot_angle):
    rx, ry = robot_pos
    ox, oy = offset

    sx = rx + ox * math.cos(robot_angle) - oy * math.sin(robot_angle)
    sy = ry + ox * math.sin(robot_angle) + oy * math.cos(robot_angle)

    return sx, sy


def calculate_robot_angle(robot_pos, shooter_offset, shooter_rotation_deg, target_pos):
    shooter_rotation = math.radians(shooter_rotation_deg)

    rx, ry = robot_pos
    tx, ty = target_pos

    # Initial guess: robot center aimed at target
    theta = math.atan2(ty - ry, tx - rx) - shooter_rotation

    for _ in range(20):
        sx, sy = shooter_position(robot_pos, shooter_offset, theta)

        angle_to_target = math.atan2(ty - sy, tx - sx)
        error = normalize_angle(angle_to_target - (theta + shooter_rotation))

        theta += error

        if abs(error) < 1e-6:
            break

    return math.degrees(theta)


# ===== INPUT DATA =====

robot_pos = (2.521347761154175, 7.23332775354)

shooter_offset = (-0.227013, -0.119366)

shooter_rotation_deg = 90

target_pos = (4.625, 4.08)


# ===== CALCULATION =====

angle = calculate_robot_angle(
    robot_pos,
    shooter_offset,
    shooter_rotation_deg,
    target_pos
)

print(f"Robot should face: {angle:.3f} degrees")