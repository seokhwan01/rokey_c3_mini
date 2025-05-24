from geometry_msgs.msg import PoseStamped
# from builtin_interfaces.msg import Time
import os
import json

def create_pose(x, y, clock):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = clock.now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.w = 1.0
    return pose

def euclidean(p1, p2):
    return ((p1.position.x - p2.position.x)**2 + (p1.position.y - p2.position.y)**2)**0.5

def load_goals_from_json(filename):
    if not os.path.exists(filename):
        print(f"[utils] JSON 파일 없음: {filename}")
        return {}
    with open(filename, 'r') as f:
        return json.load(f)