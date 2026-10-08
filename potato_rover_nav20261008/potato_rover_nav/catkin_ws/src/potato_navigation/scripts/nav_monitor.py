#!/usr/bin/env python3
import threading
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Path

class NavMonitor:
    def __init__(self):
        self.lock = threading.Lock()
        self.last = {}
        self.nav_cmd = Twist()
        self.safe_cmd = Twist()
        topics = [
            ("/move_base/GlobalPlanner/plan", Path, "global_plan"),
            ("/move_base/TebLocalPlannerROS/local_plan", Path, "local_plan"),
            ("/move_base/global_costmap/costmap", OccupancyGrid, "global_costmap"),
            ("/move_base/local_costmap/costmap", OccupancyGrid, "local_costmap"),
        ]
        for topic, msg_type, key in topics:
            rospy.Subscriber(topic, msg_type, self.stamp, callback_args=key, queue_size=1)
        rospy.Subscriber("/cmd_vel/nav", Twist, self.nav_cb, queue_size=1)
        rospy.Subscriber("/cmd_vel/safe", Twist, self.safe_cb, queue_size=1)
        rospy.Timer(rospy.Duration(1.0), self.report)

    def stamp(self, _msg, key):
        with self.lock:
            self.last[key] = rospy.Time.now()

    def nav_cb(self, msg):
        with self.lock:
            self.nav_cmd = msg
            self.last["nav_cmd"] = rospy.Time.now()

    def safe_cb(self, msg):
        with self.lock:
            self.safe_cmd = msg
            self.last["safe_cmd"] = rospy.Time.now()

    def report(self, _event):
        now = rospy.Time.now()
        with self.lock:
            def state(key, limit=2.0):
                if key not in self.last:
                    return "WAIT"
                return "OK" if (now - self.last[key]).to_sec() < limit else "STALE"
            rospy.loginfo(
                "global=%s local=%s gcost=%s lcost=%s | nav(v=%.3f,w=%.3f) safe(v=%.3f,w=%.3f)",
                state("global_plan"), state("local_plan"),
                state("global_costmap", 3.0), state("local_costmap"),
                self.nav_cmd.linear.x, self.nav_cmd.angular.z,
                self.safe_cmd.linear.x, self.safe_cmd.angular.z)

if __name__ == "__main__":
    rospy.init_node("nav_monitor")
    NavMonitor()
    rospy.spin()
