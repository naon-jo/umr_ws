import yaml
from math import hypot

from geometry_msgs.msg import Pose
from nav_msgs.msg import Path

def load_place_map(yaml_file):
    with open(yaml_file, 'r') as f:
        data = yaml.safe_load(f)
    
    place_map = {}
    for name, pose_dict in data.items():
        pose = Pose()
        pose.position.x = pose_dict["position"]["x"]
        pose.position.y = pose_dict["position"]["y"]
        pose.position.z = pose_dict["position"]["z"]
        pose.orientation.x = pose_dict["orientation"]["x"]
        pose.orientation.y = pose_dict["orientation"]["y"]
        pose.orientation.z = pose_dict["orientation"]["z"]
        pose.orientation.w = pose_dict["orientation"]["w"]
        place_map[name] = pose
    
    return place_map


def get_stop_point(path: Path, stop_distance: float) -> Pose:
    """
    Return the point on the path that is 'stop_distance' meters away from the goal.
    
    Args:
        path (Path): nav_msgs/Path containing pose list
        stop_distance (float): Distance before goal to stop
    
    Returns:
        pose: Pose: Valid stop point pose before goal
    """

    poses = path.poses  
    if len(poses) < 2:
        return poses[0].pose

    accumulated = 0.0
    for i in range(len(poses) - 1, 0, -1):
        p1 = poses[i].pose.position
        p2 = poses[i - 1].pose.position
        dist = hypot(p1.x - p2.x, p1.y - p2.y)
        accumulated += dist
        
        print(p1, p2, dist, accumulated)

        if accumulated >= stop_distance:
            return poses[i - 1].pose

    # If total path length < stop_distance, fallback to start.
    return poses[0].pose