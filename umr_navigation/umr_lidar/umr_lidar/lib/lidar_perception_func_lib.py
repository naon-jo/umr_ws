from math import pi, radians

def normalize_angle(angle):
    """Normalize angle to [-pi, pi]"""
    return (angle + pi) % (2 * pi) - pi

def check_obstacle_front(msg, front_angle_range_deg):
    """Returns the minimum LIDAR range value within the front angle range.

    Args:
        msg: LaserScan message
        front_angle_range_deg (float): Front view angle in degrees (e.g. 30 → ±15 deg)

    Returns:
        float: Minimum distance within the front view range, or inf if none
    """
    angle_min = msg.angle_min
    angle_increment = msg.angle_increment
    ranges = msg.ranges
    
    front_angle_half_rad = radians(front_angle_range_deg) / 2
 
    front_ranges = []
    for i in range(len(ranges)):
        angle = angle_min + i * angle_increment
        if abs(normalize_angle(angle)) <= front_angle_half_rad:
            front_ranges.append(ranges[i])

    # Keep only valid values within sensor's measurable range
    valid_ranges = [r for r in front_ranges if msg.range_min <= r <= msg.range_max]

    return min(valid_ranges) if valid_ranges else float("inf")
