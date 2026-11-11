#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtle_mission.msg import Waypoint, MissionStatus


def wrap_angle(a):
    """Normalize angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


class WaypointController:
    def __init__(self):
        self.tol    = rospy.get_param("/mission/tolerance", 0.05)
        self.kp_lin = rospy.get_param("/mission/kp_linear", 1.5)
        self.kp_ang = rospy.get_param("/mission/kp_angular", 4.0)
        self.v_max  = rospy.get_param("/mission/max_linear", 2.0)

        self.pose = None
        self.target = None

        self.cmd_pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
        self.status_pub = rospy.Publisher("/mission/status", MissionStatus, queue_size=10)
        rospy.Subscriber("/turtle1/pose", Pose, self.on_pose)
        rospy.Subscriber("/mission/target", Waypoint, self.on_target)

    def on_pose(self, msg):
        self.pose = msg

    def on_target(self, msg):
        self.target = msg
        rospy.loginfo("New target #%d: (%.2f, %.2f)", msg.index + 1, msg.x, msg.y)

    def step(self):
        cmd = Twist()
        st = MissionStatus()

        if self.pose is None or self.target is None:
            st.state = "IDLE"
        else:
            dx = self.target.x - self.pose.x
            dy = self.target.y - self.pose.y
            dist = math.hypot(dx, dy)
            heading_err = wrap_angle(math.atan2(dy, dx) - self.pose.theta)

            st.waypoint_index = self.target.index
            st.distance_to_target = dist

            if dist < self.tol:                       # arrived -> stop
                st.reached = True
                st.state = "REACHED"
            else:                                     # P-control
                cmd.angular.z = self.kp_ang * heading_err
                cmd.linear.x = min(self.kp_lin * dist, self.v_max) \
                               * max(0.0, math.cos(heading_err))
                st.state = "DRIVING"

        self.cmd_pub.publish(cmd)
        self.status_pub.publish(st)


if __name__ == "__main__":
    rospy.init_node("waypoint_controller")
    ctrl = WaypointController()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        ctrl.step()
        rate.sleep()
