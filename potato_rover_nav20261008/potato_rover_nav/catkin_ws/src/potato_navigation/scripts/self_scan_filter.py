#!/usr/bin/env python3
"""Remove only near-field rear chassis returns from a base_link LaserScan.

The forward half-plane is never masked; it retains the upstream 0.20 m limit.
"""
import math

import rospy
from sensor_msgs.msg import LaserScan


class SelfScanFilter:
    def __init__(self):
        self.rear_x_min = float(rospy.get_param("~rear_x_min", -0.23))
        self.rear_x_max = float(rospy.get_param("~rear_x_max", 0.0))
        self.rear_half_width = float(rospy.get_param("~rear_half_width", 0.115))
        self.pub = rospy.Publisher("/scan", LaserScan, queue_size=1)
        rospy.Subscriber("/scan_raw", LaserScan, self.on_scan, queue_size=1,
                         tcp_nodelay=True)
        rospy.loginfo("[Self scan] rear mask x=[%.3f, %.3f), |y|<=%.3f m; front untouched",
                      self.rear_x_min, self.rear_x_max, self.rear_half_width)

    def on_scan(self, scan):
        if scan.header.frame_id.lstrip("/") != "base_link":
            rospy.logerr_throttle(5.0, "[Self scan] Expected base_link, got %s; no mask applied",
                                  scan.header.frame_id)
            self.pub.publish(scan)
            return
        ranges = list(scan.ranges)
        intensities = list(scan.intensities)
        has_intensities = len(intensities) == len(ranges)
        masked = 0
        for i, distance in enumerate(ranges):
            if not math.isfinite(distance) or distance < scan.range_min:
                continue
            angle = scan.angle_min + i * scan.angle_increment
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)
            if (self.rear_x_min <= x < self.rear_x_max and
                    abs(y) <= self.rear_half_width):
                ranges[i] = float("inf")
                if has_intensities:
                    intensities[i] = 0.0
                masked += 1
        scan.ranges = ranges
        if has_intensities:
            scan.intensities = intensities
        self.pub.publish(scan)
        rospy.loginfo_throttle(5.0, "[Self scan] masked %d rear chassis rays", masked)


if __name__ == "__main__":
    rospy.init_node("nav_self_scan_filter")
    SelfScanFilter()
    rospy.spin()
