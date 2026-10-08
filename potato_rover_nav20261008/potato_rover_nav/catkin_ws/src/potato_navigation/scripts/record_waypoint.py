#!/usr/bin/env python3
import os
import tempfile
import threading
import yaml
import rospy
from geometry_msgs.msg import PointStamped

class WaypointRecorder:
    def __init__(self):
        self.output = os.path.abspath(rospy.get_param(
            "~output_file", "/rover_ws/src/potato_navigation/config/waypoints.yaml"))
        self.yaw = float(rospy.get_param("~default_yaw", 0.0))
        self.wait_time = float(rospy.get_param("~wait_time", 2.0))
        self.prefix = str(rospy.get_param("~name_prefix", "point"))
        self.lock = threading.Lock()
        os.makedirs(os.path.dirname(self.output), exist_ok=True)
        rospy.Subscriber("/clicked_point", PointStamped, self.callback, queue_size=10)
        rospy.loginfo("[Waypoint Recorder] File: %s", self.output)
        rospy.loginfo("[Waypoint Recorder] Select RViz Publish Point and click the map.")

    def load(self):
        if not os.path.exists(self.output):
            return {"loop": False, "default_tolerance": 0.15, "waypoints": []}
        with open(self.output, "r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
        data.setdefault("loop", False)
        data.setdefault("default_tolerance", 0.15)
        if data.get("waypoints") is None:
            data["waypoints"] = []
        if not isinstance(data["waypoints"], list):
            raise ValueError("waypoints must be a list")
        return data

    def save(self, data):
        directory = os.path.dirname(self.output)
        fd, temp_path = tempfile.mkstemp(prefix=".waypoints_", suffix=".yaml", dir=directory)
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as stream:
                yaml.safe_dump(data, stream, allow_unicode=True, sort_keys=False)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temp_path, self.output)
        finally:
            if os.path.exists(temp_path):
                os.unlink(temp_path)

    def callback(self, msg):
        if msg.header.frame_id and msg.header.frame_id != "map":
            rospy.logerr("[Waypoint Recorder] Expected frame map, got %s", msg.header.frame_id)
            return
        with self.lock:
            try:
                data = self.load()
                index = len(data["waypoints"]) + 1
                waypoint = {
                    "name": "%s_%02d" % (self.prefix, index),
                    "x": round(float(msg.point.x), 4),
                    "y": round(float(msg.point.y), 4),
                    "yaw": round(self.yaw, 4),
                    "wait_time": round(self.wait_time, 2),
                }
                data["waypoints"].append(waypoint)
                self.save(data)
                snippet = yaml.safe_dump([waypoint], allow_unicode=True, sort_keys=False).rstrip()
                rospy.loginfo("[Waypoint %d] Appended to file:\n%s", index, snippet)
            except Exception as exc:
                rospy.logerr("[Waypoint Recorder] Save failed: %s", exc)

if __name__ == "__main__":
    rospy.init_node("record_waypoint")
    WaypointRecorder()
    rospy.spin()
