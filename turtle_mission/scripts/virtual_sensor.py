#!/usr/bin/env python3
"""Virtual wall sensor: turns /turtle1/pose into range readings."""
import math
import rospy
from turtlesim.msg import Pose
from turtle_mission.msg import WallSensor

SIZE = 11.0889        # turtlesim arena is a square from 0 to ~11.0889
SIDE_ANGLE = 0.7      # rad (~40 deg): angle of the left/right rays


def ray(x, y, angle):
    """Distance from (x, y) to the arena wall along 'angle' (world frame)."""
    c, s = math.cos(angle), math.sin(angle)
    d = []
    if c > 1e-6:  d.append((SIZE - x) / c)
    if c < -1e-6: d.append(-x / c)
    if s > 1e-6:  d.append((SIZE - y) / s)
    if s < -1e-6: d.append(-y / s)
    return max(0.0, min(d)) if d else SIZE


class VirtualSensor:
    def __init__(self):
        self.safe = rospy.get_param("~safe_distance", 1.0)
        self.closest = float("inf")
        self.pub = rospy.Publisher("/wall_sensor", WallSensor, queue_size=10)
        rospy.Subscriber("/turtle1/pose", Pose, self.on_pose)
        rospy.on_shutdown(self.report)

    def on_pose(self, p):
        walls = {"left": p.x, "right": SIZE - p.x,
                 "bottom": p.y, "top": SIZE - p.y}
        name = min(walls, key=walls.get)
        dmin = walls[name]
        self.closest = min(self.closest, dmin)

        m = WallSensor()
        m.front_distance = ray(p.x, p.y, p.theta)
        m.left_distance = ray(p.x, p.y, p.theta + SIDE_ANGLE)
        m.right_distance = ray(p.x, p.y, p.theta - SIDE_ANGLE)
        m.min_distance = dmin
        m.nearest_wall = name
        m.near_wall = dmin < self.safe
        self.pub.publish(m)

    def report(self):
        rospy.loginfo("Closest approach to any wall this run: %.2f", self.closest)


if __name__ == "__main__":
    rospy.init_node("virtual_sensor")
    VirtualSensor()
    rospy.spin()
