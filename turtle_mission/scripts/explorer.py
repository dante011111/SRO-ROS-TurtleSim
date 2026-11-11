#!/usr/bin/env python3
"""Safe explorer: wanders randomly, reacts ONLY to /wall_sensor."""
import random
import rospy
from geometry_msgs.msg import Twist
from turtle_mission.msg import WallSensor


class Explorer:
    def __init__(self):
        self.safe = rospy.get_param("~safe_distance", 1.0)
        self.speed = rospy.get_param("~speed", 1.5)
        self.turn_speed = rospy.get_param("~turn_speed", 2.0)

        self.sensor = None
        self.last_msg_time = rospy.Time(0)
        self.avoiding = False
        self.turn_dir = 1
        self.wander = 0.0
        self.next_wander = rospy.Time(0)

        self.pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/wall_sensor", WallSensor, self.on_sensor)

    def on_sensor(self, m):
        self.sensor = m
        self.last_msg_time = rospy.Time.now()

    def wander_value(self):
        now = rospy.Time.now()
        if now >= self.next_wander:
            self.wander = random.uniform(-0.9, 0.9)
            self.next_wander = now + rospy.Duration(random.uniform(1.0, 2.5))
        return self.wander

    def step(self):
        cmd = Twist()
        m = self.sensor
        fresh = (rospy.Time.now() - self.last_msg_time).to_sec() < 0.5

        if m is not None and fresh:      # no fresh sensor data -> stay stopped
            danger = (m.front_distance < self.safe or
                      m.left_distance < 0.8 * self.safe or
                      m.right_distance < 0.8 * self.safe)
            clear = (m.front_distance > 2.0 * self.safe and
                     m.left_distance > self.safe and
                     m.right_distance > self.safe)

            if not self.avoiding and danger:
                self.avoiding = True
                # turn toward the side with more room, and keep that direction
                self.turn_dir = 1 if m.left_distance >= m.right_distance else -1
                rospy.loginfo("Wall near (%s, %.2f) -> turning %s",
                              m.nearest_wall, m.min_distance,
                              "left" if self.turn_dir > 0 else "right")
            elif self.avoiding and clear:
                self.avoiding = False

            if self.avoiding:
                cmd.angular.z = self.turn_speed * self.turn_dir
            else:
                cmd.linear.x = min(self.speed, 1.2 * m.front_distance)
                cmd.angular.z = self.wander_value()

        self.pub.publish(cmd)


if __name__ == "__main__":
    rospy.init_node("explorer")
    node = Explorer()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        node.step()
        rate.sleep()
