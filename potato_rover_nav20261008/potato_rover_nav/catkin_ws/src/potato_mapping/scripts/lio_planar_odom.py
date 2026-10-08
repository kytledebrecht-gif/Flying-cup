#!/usr/bin/env python3
import math
import threading

import rospy
import tf
from nav_msgs.msg import Odometry


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


class LioPlanarOdom:
    def __init__(self):
        self.input_topic = rospy.get_param("~input", "/Odometry")
        self.output_topic = rospy.get_param("~output", "/odom")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.base_to_imu_x = float(rospy.get_param("~base_to_imu_x", 0.076))
        self.base_to_imu_y = float(rospy.get_param("~base_to_imu_y", 0.02329))
        self.velocity_alpha = float(rospy.get_param("~velocity_alpha", 0.35))
        self.pub = rospy.Publisher(self.output_topic, Odometry, queue_size=5)
        self.tf_pub = tf.TransformBroadcaster()
        self.lock = threading.Lock()
        self.last_stamp = None
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_yaw = 0.0
        self.vx = self.vy = self.wz = 0.0
        self.sub = rospy.Subscriber(self.input_topic, Odometry, self.callback, queue_size=5)
        rospy.loginfo("FAST-LIO planar odom: %s -> %s, TF %s -> %s",
                      self.input_topic, self.output_topic, self.odom_frame, self.base_frame)

    def callback(self, raw):
        stamp = raw.header.stamp
        if stamp.is_zero():
            stamp = rospy.Time.now()
        q = raw.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))

        c = math.cos(yaw)
        s = math.sin(yaw)
        imu_x = raw.pose.pose.position.x
        imu_y = raw.pose.pose.position.y
        base_x = imu_x - (c * self.base_to_imu_x - s * self.base_to_imu_y)
        base_y = imu_y - (s * self.base_to_imu_x + c * self.base_to_imu_y)

        with self.lock:
            if self.last_stamp is not None:
                dt = (stamp - self.last_stamp).to_sec()
                if 0.002 < dt < 0.5:
                    dx = base_x - self.last_x
                    dy = base_y - self.last_y
                    vx_world = dx / dt
                    vy_world = dy / dt
                    vx = c * vx_world + s * vy_world
                    vy = -s * vx_world + c * vy_world
                    wz = wrap_angle(yaw - self.last_yaw) / dt
                    a = self.velocity_alpha
                    self.vx = a * vx + (1.0 - a) * self.vx
                    self.vy = a * vy + (1.0 - a) * self.vy
                    self.wz = a * wz + (1.0 - a) * self.wz
            self.last_stamp = stamp
            self.last_x, self.last_y, self.last_yaw = base_x, base_y, yaw

        yaw_q = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = base_x
        msg.pose.pose.position.y = base_y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = yaw_q[0]
        msg.pose.pose.orientation.y = yaw_q[1]
        msg.pose.pose.orientation.z = yaw_q[2]
        msg.pose.pose.orientation.w = yaw_q[3]
        msg.pose.covariance = list(raw.pose.covariance)
        msg.twist.twist.linear.x = self.vx
        msg.twist.twist.linear.y = self.vy
        msg.twist.twist.angular.z = self.wz
        msg.twist.covariance[0] = 0.02
        msg.twist.covariance[7] = 0.04
        msg.twist.covariance[35] = 0.03
        self.pub.publish(msg)
        self.tf_pub.sendTransform(
            (base_x, base_y, 0.0), yaw_q, stamp, self.base_frame, self.odom_frame)


if __name__ == "__main__":
    rospy.init_node("lio_planar_odom")
    LioPlanarOdom()
    rospy.spin()
