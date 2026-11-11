#!/usr/bin/env python3
import math, random, rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill
from turtle_mission.msg import Score


def wrap(a): return math.atan2(math.sin(a), math.cos(a))


class Hunter:
    def __init__(self):
        self.kill_dist = rospy.get_param("~kill_distance", 0.5)
        self.rounds    = rospy.get_param("~rounds", 5)
        self.kp_lin, self.kp_ang, self.v_max = 1.5, 4.0, 2.5

        rospy.wait_for_service("/spawn"); rospy.wait_for_service("/kill")
        self.spawn = rospy.ServiceProxy("/spawn", Spawn)
        self.kill  = rospy.ServiceProxy("/kill", Kill)

        self.cmd_pub   = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
        self.score_pub = rospy.Publisher("/score", Score, queue_size=10, latch=True)
        rospy.Subscriber("/turtle1/pose", Pose, lambda m: setattr(self, "pose", m))

        self.pose = self.prey_pose = self.prey_sub = None
        self.prey_name, self.count, self.score = None, 0, 0

    def spawn_prey(self):
        self.count += 1
        self.prey_name = "prey%d" % self.count          # unique name each round
        self.prey_pose = None
        self.spawn(random.uniform(1, 10), random.uniform(1, 10),
                   random.uniform(-math.pi, math.pi), self.prey_name)
        self.prey_sub = rospy.Subscriber("/%s/pose" % self.prey_name, Pose,
                                         lambda m: setattr(self, "prey_pose", m))

    def catch(self):
        self.prey_sub.unregister()
        self.kill(self.prey_name)
        self.score += 1
        self.score_pub.publish(Score(score=self.score, last_prey=self.prey_name))
        rospy.loginfo("Caught %s! Score = %d", self.prey_name, self.score)

    def run(self):
        self.score_pub.publish(Score(score=0, last_prey=""))
        self.spawn_prey()
        rate = rospy.Rate(50)
        while not rospy.is_shutdown() and self.score < self.rounds:
            if self.pose and self.prey_pose:
                dx = self.prey_pose.x - self.pose.x
                dy = self.prey_pose.y - self.pose.y
                dist = math.hypot(dx, dy)
                if dist < self.kill_dist:
                    self.cmd_pub.publish(Twist())
                    self.catch()
                    if self.score < self.rounds:
                        self.spawn_prey()
                else:
                    err = wrap(math.atan2(dy, dx) - self.pose.theta)
                    cmd = Twist()
                    cmd.angular.z = self.kp_ang * err
                    cmd.linear.x = min(self.kp_lin * dist, self.v_max) * max(0.0, math.cos(err))
                    self.cmd_pub.publish(cmd)
            rate.sleep()
        self.cmd_pub.publish(Twist())
        rospy.loginfo("GAME OVER. Final score: %d", self.score)


if __name__ == "__main__":
    rospy.init_node("hunter")
    Hunter().run()
