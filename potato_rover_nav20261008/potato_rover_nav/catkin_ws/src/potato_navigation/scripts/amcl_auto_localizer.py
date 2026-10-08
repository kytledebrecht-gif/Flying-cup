#!/usr/bin/env python3
import threading
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import Empty


class AmclAutoLocalizer:
    def __init__(self):
        self.lock = threading.Lock()
        self.active = False
        self.manual_received = False
        self.converged = False
        self.good_count = 0
        self.bad_count = 0
        self.xy_threshold = float(rospy.get_param("~xy_cov_threshold", 0.20))
        self.yaw_threshold = float(rospy.get_param("~yaw_cov_threshold", 0.15))
        self.required = int(rospy.get_param("~required_samples", 8))
        self.lost_samples = int(rospy.get_param("~lost_samples", 3))
        self.grace = float(rospy.get_param("~manual_init_grace_sec", 5.0))
        self.nomotion_period = float(rospy.get_param("~nomotion_update_period", 0.5))
        self.pub = rospy.Publisher("/amcl_converged", Bool, queue_size=1, latch=True)
        self.pub.publish(Bool(data=False))
        rospy.Subscriber("/initialpose", PoseWithCovarianceStamped,
                         self.initial_cb, queue_size=1)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped,
                         self.pose_cb, queue_size=10)
        threading.Thread(target=self.auto_initialize, daemon=True).start()
        threading.Thread(target=self.nomotion_updates, daemon=True).start()

    def activate(self, source):
        with self.lock:
            self.active = True
            self.converged = False
            self.good_count = 0
            self.bad_count = 0
        self.pub.publish(Bool(data=False))
        rospy.logwarn("[AMCL] Localization evaluation activated after %s.", source)

    def initial_cb(self, _msg):
        with self.lock:
            self.manual_received = True
        self.activate("manual 2D Pose")
        rospy.logwarn("[AMCL] Manual 2D Pose received; global spreading is cancelled.")

    def wait_inputs(self):
        rospy.wait_for_message("/map", OccupancyGrid, timeout=30.0)
        rospy.wait_for_message("/scan", LaserScan, timeout=30.0)
        rospy.wait_for_service("/global_localization", timeout=30.0)

    def auto_initialize(self):
        try:
            rospy.loginfo("[AMCL] Waiting for map, scan and localization services...")
            self.wait_inputs()
        except (rospy.ROSException, rospy.ROSInterruptException):
            rospy.logerr("[AMCL] Map, scan or global_localization service is unavailable.")
            return
        deadline = rospy.Time.now() + rospy.Duration(self.grace)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            with self.lock:
                if self.manual_received:
                    rospy.loginfo("[AMCL] Using the manual initial pose.")
                    return
            rospy.sleep(0.1)
        with self.lock:
            if self.manual_received:
                return
        for attempt in range(1, 4):
            try:
                rospy.ServiceProxy("/global_localization", Empty)()
                self.activate("global particle spreading")
                rospy.logwarn("[AMCL] No manual pose; global particle spreading requested (attempt %d).",
                              attempt)
                rospy.logwarn("[AMCL] Do not start waypoint motion until convergence is reported.")
                return
            except rospy.ServiceException as exc:
                rospy.logerr("[AMCL] global_localization failed: %s", exc)
                rospy.sleep(1.0)

    def nomotion_updates(self):
        try:
            rospy.wait_for_service("/request_nomotion_update", timeout=40.0)
            service = rospy.ServiceProxy("/request_nomotion_update", Empty)
        except (rospy.ROSException, rospy.ROSInterruptException):
            rospy.logerr("[AMCL] request_nomotion_update service is unavailable.")
            return
        rate = rospy.Rate(max(0.2, 1.0 / max(self.nomotion_period, 0.1)))
        while not rospy.is_shutdown():
            with self.lock:
                should_update = self.active and not self.converged
            if should_update:
                try:
                    service()
                except rospy.ServiceException as exc:
                    rospy.logwarn_throttle(5.0, "[AMCL] nomotion update failed: %s", exc)
            rate.sleep()

    def pose_cb(self, msg):
        cov = msg.pose.covariance
        x_cov, y_cov, yaw_cov = cov[0], cov[7], cov[35]
        good = (0.0 <= x_cov <= self.xy_threshold and
                0.0 <= y_cov <= self.xy_threshold and
                0.0 <= yaw_cov <= self.yaw_threshold)
        with self.lock:
            if not self.active:
                return
            self.good_count = self.good_count + 1 if good else 0
            self.bad_count = 0 if good else self.bad_count + 1
            just_converged = not self.converged and self.good_count >= self.required
            just_lost = self.converged and self.bad_count >= self.lost_samples
            if just_converged:
                self.converged = True
                self.bad_count = 0
            elif just_lost:
                self.converged = False
                self.good_count = 0
        if just_converged:
            self.pub.publish(Bool(data=True))
            rospy.loginfo("\033[1;32m[AMCL] \u91cd\u5b9a\u4f4d\u6536\u655b\u5b8c\u6210\uff0c\u53ef\u4ee5\u5f00\u59cb\u5bfc\u822a\uff01 "
                          "cov=(%.4f, %.4f, %.4f)\033[0m",
                          x_cov, y_cov, yaw_cov)
        elif just_lost:
            self.pub.publish(Bool(data=False))
            rospy.logerr("[AMCL] Localization uncertainty increased; navigation lock restored. "
                         "cov=(%.4f, %.4f, %.4f)", x_cov, y_cov, yaw_cov)
        elif not self.converged:
            rospy.loginfo_throttle(
                2.0, "[AMCL] Converging cov=(%.4f, %.4f, %.4f), good=%d/%d",
                x_cov, y_cov, yaw_cov, self.good_count, self.required)


if __name__ == "__main__":
    rospy.init_node("amcl_auto_localizer")
    AmclAutoLocalizer()
    rospy.spin()
