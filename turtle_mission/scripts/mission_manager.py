#!/usr/bin/env python3
import rospy
from turtlesim.msg import Pose
from turtle_mission.msg import Waypoint, MissionStatus
from turtle_mission.srv import StartMission, StartMissionRequest, StartMissionResponse


class MissionManager:
    def __init__(self):
        # The ONLY place where waypoints are read: the parameter server
        self.waypoints = [tuple(w) for w in rospy.get_param("/mission/waypoints")]
        self.route = []
        self.index = 0
        self.running = False

        self.target_pub = rospy.Publisher("/mission/target", Waypoint, queue_size=1, latch=True)
        rospy.Subscriber("/mission/status", MissionStatus, self.on_status)
        rospy.Service("/start_mission", StartMission, self.on_start)

        if rospy.get_param("~auto_start", False):
            rospy.Timer(rospy.Duration(1.5),
                        lambda e: self.on_start(StartMissionRequest(return_home=True)),
                        oneshot=True)
        rospy.loginfo("Mission manager ready with %d waypoints. Call /start_mission.",
                      len(self.waypoints))

    def on_start(self, req):
        if self.running:
            return StartMissionResponse(False, "Mission already running")
        self.route = list(self.waypoints)
        if req.return_home:
            home = rospy.wait_for_message("/turtle1/pose", Pose, timeout=2.0)
            self.route.append((home.x, home.y))
        self.index = 0
        self.running = True
        self.send_target()
        return StartMissionResponse(True, "Started with %d waypoints" % len(self.route))

    def send_target(self):
        x, y = self.route[self.index]
        self.target_pub.publish(Waypoint(index=self.index, total=len(self.route), x=x, y=y))

    def on_status(self, msg):
        if not self.running:
            return
        # index check prevents advancing twice on repeated "reached" messages
        if msg.reached and msg.waypoint_index == self.index:
            rospy.loginfo("Waypoint %d/%d reached", self.index + 1, len(self.route))
            self.index += 1
            if self.index >= len(self.route):
                self.running = False
                rospy.loginfo("MISSION COMPLETE")
            else:
                self.send_target()


if __name__ == "__main__":
    rospy.init_node("mission_manager")
    MissionManager()
    rospy.spin()
