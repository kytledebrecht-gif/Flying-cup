#!/usr/bin/env python3
import math
import os
import time
import yaml
import rospy
import actionlib
import tf
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool
from std_srvs.srv import Empty

class WaypointNavigator:
    TERMINAL_STATES = {
        GoalStatus.PREEMPTED,
        GoalStatus.SUCCEEDED,
        GoalStatus.ABORTED,
        GoalStatus.REJECTED,
        GoalStatus.RECALLED,
        GoalStatus.LOST,
    }

    def __init__(self):
        self.file = os.path.abspath(rospy.get_param(
            "~waypoints_file", "/rover_ws/src/potato_navigation/config/waypoints.yaml"))
        self.goal_timeout = float(rospy.get_param("~goal_timeout", 120.0))
        self.max_retries = int(rospy.get_param("~max_retries", 1))
        self.failure_policy = str(rospy.get_param("~failure_policy", "skip")).lower()
        self.wait_for_amcl = bool(rospy.get_param("~wait_for_amcl", True))
        self.amcl_timeout = float(rospy.get_param("~amcl_timeout", 300.0))
        self.final_yaw_tolerance = float(rospy.get_param("~final_yaw_tolerance", 0.30))
        self.final_xy_tolerance = float(rospy.get_param("~final_xy_tolerance", 0.20))
        self.final_yaw_timeout = float(rospy.get_param("~final_yaw_timeout", 45.0))
        self.departure_yaw_threshold = float(rospy.get_param("~departure_yaw_threshold", 1.20))
        self.departure_yaw_tolerance = float(rospy.get_param("~departure_yaw_tolerance", 0.35))
        self.departure_yaw_timeout = float(rospy.get_param("~departure_yaw_timeout", 30.0))
        self.navigation_mode = str(rospy.get_param("~navigation_mode", "smooth")).lower()
        self.pass_radius = float(rospy.get_param("~pass_radius", 0.25))
        self.smooth_angle_limit = float(rospy.get_param("~smooth_angle_limit", 1.05))
        if self.navigation_mode not in ("smooth", "precise"):
            raise ValueError("navigation_mode must be smooth or precise")
        if self.pass_radius <= 0.0 or not 0.0 < self.smooth_angle_limit <= math.pi:
            raise ValueError("pass_radius and smooth_angle_limit must be positive")
        self.converged = not self.wait_for_amcl
        self.tf_listener = tf.TransformListener()
        rospy.Subscriber("/amcl_converged", Bool, self.converged_cb, queue_size=1)
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.on_shutdown(self.shutdown)

    def converged_cb(self, msg):
        self.converged = bool(msg.data)

    def load(self):
        with open(self.file, "r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
        points = data.get("waypoints")
        if points is None:
            points = []
        if not isinstance(points, list):
            raise ValueError("waypoints must be a list")
        for index, point in enumerate(points, 1):
            for field in ("name", "x", "y", "yaw", "wait_time"):
                if field not in point:
                    raise ValueError("waypoint %d missing %s" % (index, field))
        return bool(data.get("loop", False)), float(data.get("default_tolerance", 0.15)), points

    def wait_localization(self):
        if not self.wait_for_amcl:
            return True
        rospy.loginfo("[Waypoint] Waiting for /amcl_converged=true...")
        deadline = rospy.Time.now() + rospy.Duration(self.amcl_timeout)
        rate = rospy.Rate(5)
        while not rospy.is_shutdown() and not self.converged:
            if rospy.Time.now() >= deadline:
                rospy.logerr("[Waypoint] AMCL convergence timeout; patrol rejected.")
                return False
            rate.sleep()
        if rospy.is_shutdown():
            return False
        # AMCL's convergence flag can precede the first usable composite TF.
        deadline = time.monotonic() + 15.0
        stable = 0
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if not self.converged:
                stable = 0
            else:
                try:
                    self.tf_listener.lookupTransform("map", "base_link", rospy.Time(0))
                    stable += 1
                    if stable >= 5:
                        return True
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    stable = 0
            time.sleep(0.2)
        rospy.logerr("[Waypoint] map->base_link TF did not become stable after AMCL convergence.")
        return False

    def pose_error(self, point):
        try:
            position, orientation = self.tf_listener.lookupTransform(
                "map", "base_link", rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as exc:
            rospy.logwarn_throttle(10.0, "[Waypoint] TF map->base_link unavailable: %s", exc)
            return None
        xy = math.hypot(position[0] - float(point["x"]),
                        position[1] - float(point["y"]))
        heading = tf.transformations.euler_from_quaternion(orientation)[2]
        yaw = math.atan2(math.sin(float(point["yaw"]) - heading),
                         math.cos(float(point["yaw"]) - heading))
        return xy, abs(yaw), heading, position[0], position[1]

    @staticmethod
    def goal(point, yaw=None, position=None):
        result = MoveBaseGoal()
        result.target_pose.header.frame_id = "map"
        result.target_pose.header.stamp = rospy.Time.now()
        result.target_pose.pose.position.x = float(point["x"] if position is None else position[0])
        result.target_pose.pose.position.y = float(point["y"] if position is None else position[1])
        yaw = float(point["yaw"] if yaw is None else yaw)
        result.target_pose.pose.orientation.z = math.sin(yaw * 0.5)
        result.target_pose.pose.orientation.w = math.cos(yaw * 0.5)
        return result

    def clear_costmaps(self):
        try:
            rospy.wait_for_service("/move_base/clear_costmaps", timeout=3.0)
            rospy.ServiceProxy("/move_base/clear_costmaps", Empty)()
            rospy.logwarn("[Waypoint] Costmaps cleared before retry.")
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logwarn("[Waypoint] Costmap clear failed: %s", exc)

    def settle_goal(self, cancel=False, timeout=5.0):
        """Finish the tracked action before another waypoint is submitted."""
        state = self.client.get_state()
        if state not in self.TERMINAL_STATES and cancel:
            self.client.cancel_goal()
        if state not in self.TERMINAL_STATES:
            self.client.wait_for_result(rospy.Duration(timeout))
            state = self.client.get_state()
        if state not in self.TERMINAL_STATES:
            rospy.logwarn("[Waypoint] Goal did not settle after %.1f s; cancelling this goal.", timeout)
            self.client.cancel_goal()
            self.client.wait_for_result(rospy.Duration(timeout))
            state = self.client.get_state()
        settled = state in self.TERMINAL_STATES
        if not settled:
            rospy.logerr("[Waypoint] move_base goal remains non-terminal (state=%d).", state)
        return state, settled

    def align_departure(self, point, index, total):
        """Turn toward a distant next waypoint before asking TEB to translate."""
        error = self.pose_error(point)
        if error is None or error[0] <= 0.05:
            return True
        bearing = math.atan2(float(point["y"]) - error[4],
                             float(point["x"]) - error[3])
        yaw_error = abs(math.atan2(math.sin(bearing - error[2]),
                                   math.cos(bearing - error[2])))
        if yaw_error < self.departure_yaw_threshold:
            return True

        rospy.loginfo("[Waypoint %d/%d] Departure turn %.3f rad toward %s.",
                      index, total, yaw_error, point["name"])
        self.client.send_goal(self.goal(point, yaw=bearing,
                                        position=(error[3], error[4])))
        deadline = time.monotonic() + self.departure_yaw_timeout
        phase_state = GoalStatus.PENDING
        settled = True
        while not rospy.is_shutdown():
            current = self.pose_error(point)
            if current is not None:
                remaining = abs(math.atan2(math.sin(bearing - current[2]),
                                           math.cos(bearing - current[2])))
                if remaining <= self.departure_yaw_tolerance:
                    phase_state, settled = self.settle_goal(cancel=True)
                    if settled:
                        rospy.loginfo("[Waypoint %d/%d] Departure turn complete (yaw=%.3f rad).",
                                      index, total, remaining)
                    return settled
            phase_state = self.client.get_state()
            if phase_state in self.TERMINAL_STATES:
                phase_state, settled = self.settle_goal(cancel=False)
                current = self.pose_error(point)
                if current is not None:
                    remaining = abs(math.atan2(math.sin(bearing - current[2]),
                                               math.cos(bearing - current[2])))
                    if remaining <= self.departure_yaw_tolerance:
                        return settled
                rospy.logwarn("[Waypoint %d/%d] Departure turn failed, state=%d.",
                              index, total, phase_state)
                return False
            if time.monotonic() >= deadline:
                phase_state, settled = self.settle_goal(cancel=True)
                rospy.logwarn("[Waypoint %d/%d] Departure turn timeout, state=%d.",
                              index, total, phase_state)
                return False
            time.sleep(0.1)
        return False

    def allow_smooth_pass(self, previous, point, following):
        """Allow continuous passage only through a reasonably gentle corner."""
        if self.navigation_mode != "smooth" or following is None:
            return False
        if previous is None:
            error = self.pose_error(point)
            if error is None:
                return False
            start_x, start_y = error[3], error[4]
        else:
            start_x, start_y = float(previous["x"]), float(previous["y"])
        in_x = float(point["x"]) - start_x
        in_y = float(point["y"]) - start_y
        out_x = float(following["x"]) - float(point["x"])
        out_y = float(following["y"]) - float(point["y"])
        in_norm = math.hypot(in_x, in_y)
        out_norm = math.hypot(out_x, out_y)
        if in_norm < 0.05 or out_norm < 0.05:
            return False
        cosine = max(-1.0, min(1.0,
                     (in_x * out_x + in_y * out_y) / (in_norm * out_norm)))
        corner = math.acos(cosine)
        allowed = corner <= self.smooth_angle_limit
        rospy.loginfo("[Waypoint] %s corner=%.1f deg: %s.", point["name"],
                      math.degrees(corner), "smooth pass" if allowed else "precise stop")
        return allowed

    def execute(self, point, index, total, tolerance, pass_through=False,
                allow_departure_align=True):
        tolerance = max(0.05, tolerance)
        reach_tolerance = max(tolerance, self.pass_radius) if pass_through else tolerance
        final_xy_tolerance = max(tolerance, self.final_xy_tolerance)
        for attempt in range(self.max_retries + 1):
            if rospy.is_shutdown():
                return False
            error = self.pose_error(point)
            if error is None or error[0] > reach_tolerance:
                if (index > 1 and allow_departure_align and error is not None and
                        not self.align_departure(point, index, total)):
                    if rospy.is_shutdown():
                        return False
                    if attempt < self.max_retries:
                        self.clear_costmaps()
                        time.sleep(1.0)
                    continue
                error = self.pose_error(point)
                # Phase 1: approach position, cancel as soon as x/y is reached.
                # The intermediate yaw is only a hint; final yaw is phase 2.
                bearing = (math.atan2(float(point["y"]) - error[4],
                                      float(point["x"]) - error[3])
                           if error is not None else float(point["yaw"]))
                rospy.loginfo("[Waypoint %d/%d] Position phase to %s (attempt %d/%d).",
                              index, total, point["name"], attempt + 1, self.max_retries + 1)
                self.client.send_goal(self.goal(point, bearing))
                deadline = time.monotonic() + self.goal_timeout
                phase_state = GoalStatus.PENDING
                settled = True
                while not rospy.is_shutdown():
                    error = self.pose_error(point)
                    if error is not None and error[0] <= reach_tolerance:
                        if pass_through:
                            rospy.loginfo("[Waypoint %d/%d] Passing %s continuously (xy=%.3f m).",
                                          index, total, point["name"], error[0])
                            return True
                        phase_state, settled = self.settle_goal(cancel=True)
                        break
                    phase_state = self.client.get_state()
                    if phase_state in self.TERMINAL_STATES:
                        phase_state, settled = self.settle_goal(cancel=False)
                        break
                    if time.monotonic() >= deadline:
                        phase_state, settled = self.settle_goal(cancel=True)
                        rospy.logwarn("[Waypoint %d/%d] Position hard timeout after %.1f s.",
                                      index, total, self.goal_timeout)
                        break
                    rospy.loginfo_throttle(5.0, "[Waypoint %d/%d] En route to %s...",
                                           index, total, point["name"])
                    time.sleep(0.2)
                if rospy.is_shutdown():
                    return False
                if not settled:
                    return False
                error = self.pose_error(point)
                if error is None or error[0] > reach_tolerance:
                    rospy.logwarn("[Waypoint %d/%d] Position phase did not reach tolerance, state=%d, xy=%s.",
                                  index, total, phase_state,
                                  "TF unavailable" if error is None else "%.3f" % error[0])
                    if attempt < self.max_retries:
                        self.clear_costmaps()
                        time.sleep(1.0)
                    continue

            if pass_through:
                error = self.pose_error(point)
                if error is not None and error[0] <= reach_tolerance:
                    rospy.loginfo("[Waypoint %d/%d] Passing %s continuously (xy=%.3f m).",
                                  index, total, point["name"], error[0])
                    return True

            # Phase 2: only after x/y is reached, request final yaw via move_base.
            # This remains inside move_base -> cmd_vel_mux; no competing cmd_vel publisher.
            error = self.pose_error(point)
            if error is not None and error[1] <= self.final_yaw_tolerance:
                rospy.loginfo("[Waypoint %d/%d] Reached %s (xy=%.3f m, yaw=%.3f rad).",
                              index, total, point["name"], error[0], error[1])
                return True
            rospy.loginfo("[Waypoint %d/%d] Final yaw phase for %s.", index, total, point["name"])
            current_xy = ((error[3], error[4]) if error is not None
                          else (float(point["x"]), float(point["y"])))
            self.client.send_goal(self.goal(point, position=current_xy))
            deadline = time.monotonic() + self.final_yaw_timeout
            phase_state = GoalStatus.PENDING
            settled = True
            while not rospy.is_shutdown():
                error = self.pose_error(point)
                if error is not None and error[0] <= final_xy_tolerance and error[1] <= self.final_yaw_tolerance:
                    phase_state, settled = self.settle_goal(cancel=True)
                    if not settled:
                        return False
                    rospy.loginfo("[Waypoint %d/%d] Reached %s (xy=%.3f m, yaw=%.3f rad).",
                                  index, total, point["name"], error[0], error[1])
                    return True
                phase_state = self.client.get_state()
                if phase_state in self.TERMINAL_STATES:
                    phase_state, settled = self.settle_goal(cancel=False)
                    break
                if time.monotonic() >= deadline:
                    phase_state, settled = self.settle_goal(cancel=True)
                    break
                time.sleep(0.2)
            if rospy.is_shutdown():
                return False
            if not settled:
                return False
            error = self.pose_error(point)
            if error is not None and error[0] <= final_xy_tolerance and error[1] <= self.final_yaw_tolerance:
                rospy.loginfo("[Waypoint %d/%d] Reached %s after action result.", index, total, point["name"])
                return True
            rospy.logwarn("[Waypoint %d/%d] Final yaw phase failed, state=%d, error=%s.",
                          index, total, phase_state, error)
            if attempt < self.max_retries:
                self.clear_costmaps()
                time.sleep(1.0)
        return False

    def run(self):
        loop, tolerance, points = self.load()
        if not points:
            rospy.logerr("[Waypoint] No points in %s; record points first.", self.file)
            return 2
        if not self.client.wait_for_server(rospy.Duration(30.0)):
            rospy.logerr("[Waypoint] move_base action server unavailable.")
            return 3
        if not self.wait_localization():
            return 4
        cycle = 0
        while not rospy.is_shutdown():
            cycle += 1
            succeeded = 0
            skipped = 0
            rospy.loginfo("[Waypoint] Starting patrol cycle %d with %d points.", cycle, len(points))
            previous = None
            previous_was_pass = False
            for index, point in enumerate(points, 1):
                if rospy.is_shutdown():
                    break
                following = points[index] if index < len(points) else None
                pass_through = self.allow_smooth_pass(previous, point, following)
                if not self.execute(point, index, len(points), tolerance,
                                    pass_through=pass_through,
                                    allow_departure_align=not previous_was_pass):
                    if rospy.is_shutdown():
                        return 0
                    if self.failure_policy == "stop":
                        rospy.logerr("[Waypoint] failure_policy=stop; patrol stopped.")
                        return 5
                    skipped += 1
                    rospy.logwarn("[Waypoint] Skipping %s.", point["name"])
                    previous_was_pass = False
                    previous = point
                    continue
                succeeded += 1
                previous_was_pass = pass_through
                previous = point
                if pass_through:
                    rospy.loginfo("[Waypoint %d/%d] Continuing without stop or yaw alignment.",
                                  index, len(points))
                else:
                    delay = max(0.0, float(point["wait_time"]))
                    rospy.loginfo("[Waypoint %d/%d] Waiting %.1f seconds.",
                                  index, len(points), delay)
                    rospy.sleep(delay)
            if not loop:
                if skipped:
                    rospy.logerr("[Waypoint] Patrol incomplete: reached %d/%d, skipped %d.",
                                 succeeded, len(points), skipped)
                    return 6
                rospy.loginfo("[Waypoint] All %d waypoints reached.", succeeded)
                return 0
            rospy.loginfo("[Waypoint] Cycle %d: reached %d/%d, skipped %d.",
                          cycle, succeeded, len(points), skipped)
        return 0

    def shutdown(self):
        if self.client.get_state() not in self.TERMINAL_STATES:
            self.client.cancel_goal()
            rospy.logwarn("[Waypoint] Active goal cancelled on shutdown.")

if __name__ == "__main__":
    rospy.init_node("waypoint_navigator")
    try:
        raise SystemExit(WaypointNavigator().run())
    except (IOError, ValueError, yaml.YAMLError) as exc:
        rospy.logfatal("[Waypoint] Configuration error: %s", exc)
        raise SystemExit(2)
