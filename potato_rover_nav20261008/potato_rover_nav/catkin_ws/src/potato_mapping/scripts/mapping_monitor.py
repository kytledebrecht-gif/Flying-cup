#!/usr/bin/env python3
import threading
import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan


class Monitor:
    def __init__(self):
        self.lock = threading.Lock()
        self.last = {"scan": None, "odom": None, "map": None}
        self.count = {"scan": 0, "odom": 0, "map": 0}
        rospy.Subscriber("/scan", LaserScan, self.cb, callback_args="scan", queue_size=1)
        rospy.Subscriber("/odom", Odometry, self.cb, callback_args="odom", queue_size=1)
        rospy.Subscriber("/map", OccupancyGrid, self.cb, callback_args="map", queue_size=1)
        rospy.Timer(rospy.Duration(1.0), self.report)

    def cb(self, _msg, key):
        with self.lock:
            self.last[key] = rospy.Time.now()
            self.count[key] += 1

    def report(self, _event):
        now = rospy.Time.now()
        with self.lock:
            states = []
            for key in ("scan", "odom", "map"):
                age = None if self.last[key] is None else (now - self.last[key]).to_sec()
                ok = age is not None and age < (3.0 if key == "map" else 1.0)
                states.append("%s=%s %.1fHz age=%s" % (
                    key, "OK" if ok else "LOST", self.count[key],
                    "--" if age is None else "%.2fs" % age))
                self.count[key] = 0
        rospy.loginfo(" | ".join(states))


if __name__ == "__main__":
    rospy.init_node("mapping_monitor")
    Monitor()
    rospy.spin()
