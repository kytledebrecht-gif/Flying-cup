#!/usr/bin/env python3
import os
import select
import sys
import termios
import time
import tty
import rospy
from geometry_msgs.msg import Twist

def make_twist(v=0.0, w=0.0):
    msg = Twist()
    msg.linear.x, msg.angular.z = v, w
    return msg

def main():
    rospy.init_node("wasd_teleop")
    max_linear = float(rospy.get_param("~max_linear_speed", 0.30))
    max_angular = float(rospy.get_param("~max_angular_speed", 0.80))
    scale = float(rospy.get_param("~initial_speed_scale", 0.50))
    scale_step = float(rospy.get_param("~speed_scale_step", 0.10))
    timeout = float(rospy.get_param("~key_timeout", 0.22))
    pub = rospy.Publisher(rospy.get_param("~topic", "/cmd_vel/teleop"), Twist, queue_size=1)
    if not sys.stdin.isatty():
        raise RuntimeError("WASD requires an interactive terminal")
    scale = max(scale_step, min(1.0, scale))
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    active, last_key, zero_sent = None, 0.0, True

    def speeds():
        return max_linear * scale, max_angular * scale

    def show_speed():
        linear, angular = speeds()
        print("\rSpeed %3d%%: linear %.2f m/s, rotate %.2f rad/s      " %
              (round(scale * 100), linear, angular), end="", flush=True)

    print("W/S forward/reverse | A/D rotate | SPACE stop | [ ] speed | Q quit")
    print("No key for %.0f ms -> immediate zero command." % (timeout * 1000.0))
    show_speed()
    try:
        tty.setcbreak(fd)
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            ready, _, _ = select.select([sys.stdin], [], [], 0.0)
            now = time.monotonic()
            if ready:
                key = os.read(fd, 1).decode(errors="ignore").lower()
                if key in ("q", "\x03"):
                    break
                if key == " ":
                    active = None
                    pub.publish(Twist())
                    zero_sent = True
                elif key == "[":
                    scale = max(scale_step, scale - scale_step)
                    show_speed()
                elif key == "]":
                    scale = min(1.0, scale + scale_step)
                    show_speed()
                elif key in ("w", "a", "s", "d"):
                    active, last_key, zero_sent = key, now, False
            if active is not None and now - last_key <= timeout:
                linear, angular = speeds()
                commands = {
                    "w": make_twist(linear, 0.0),
                    "s": make_twist(-linear, 0.0),
                    "a": make_twist(0.0, angular),
                    "d": make_twist(0.0, -angular),
                }
                pub.publish(commands[active])
            elif not zero_sent:
                active = None
                pub.publish(Twist())
                zero_sent = True
            rate.sleep()
    finally:
        for _ in range(3):
            pub.publish(Twist())
            time.sleep(0.03)
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print("\nStopped: zero command sent.")

if __name__ == "__main__":
    main()
