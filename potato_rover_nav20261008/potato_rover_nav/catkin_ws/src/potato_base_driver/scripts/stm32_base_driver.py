#!/usr/bin/env python3
import math
import threading
import time
import rospy
import serial
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class Stm32BaseDriver:
    def __init__(self):
        self.port = rospy.get_param("~port", "/dev/potato_stm32")
        self.baud = int(rospy.get_param("~baud", 115200))
        self.timeout = float(rospy.get_param("~command_timeout", 0.25))
        self.rate_hz = float(rospy.get_param("~send_rate", 20.0))
        self.max_v = float(rospy.get_param("~max_linear_mps", 0.30))
        self.max_w = float(rospy.get_param("~max_angular_rps", 0.80))
        self.track = float(rospy.get_param("~track_width_m", 0.30))
        self.max_units = int(rospy.get_param("~max_speed_units", 200))
        self.max_accel = float(rospy.get_param("~max_accel_units_per_s", 400.0))
        self.turn_gain = float(rospy.get_param("~turn_gain", 1.0))
        self.min_turn_units = int(rospy.get_param("~min_turn_units", 0))
        self.signs = [int(x) for x in rospy.get_param("~motor_signs", [1, 1, 1, 1])]
        self.command = str(rospy.get_param("~protocol_command", "spd"))
        if len(self.signs) != 4 or any(x not in (-1, 1) for x in self.signs):
            raise ValueError("motor_signs must be four +1/-1 values")
        if min(self.timeout, self.rate_hz, self.max_v, self.max_w, self.track,
               self.max_units, self.max_accel, self.turn_gain) <= 0:
            raise ValueError("control limits must be positive")
        if not 0 <= self.min_turn_units <= self.max_units:
            raise ValueError("min_turn_units must be between zero and max_speed_units")
        self.lock = threading.Lock()
        self.latest = Twist()
        self.last_cmd = 0.0
        self.ser = None
        self.next_retry = 0.0
        self.current = [0, 0, 0, 0]
        self.rx_pub = rospy.Publisher("/stm32/raw_rx", String, queue_size=10)
        rospy.Subscriber("/cmd_vel/safe", Twist, self.cmd_cb, queue_size=1, tcp_nodelay=True)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("STM32 %s @ %d max=%d ramp=%.0f unit/s watchdog=%.3fs",
                      self.port, self.baud, self.max_units, self.max_accel, self.timeout)

    def cmd_cb(self, msg):
        with self.lock:
            self.latest, self.last_cmd = msg, time.monotonic()

    def connect(self):
        now = time.monotonic()
        if self.ser is not None or now < self.next_retry:
            return
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0,
                                     write_timeout=0.2, exclusive=True)
            self.ser.reset_input_buffer()
            self.current = [0, 0, 0, 0]
            self.send(self.current)
            rospy.loginfo("STM32 connected exclusively: %s", self.port)
        except Exception as exc:
            self.ser = None
            self.next_retry = now + 1.0
            rospy.logwarn_throttle(5.0, "Waiting for STM32: %s", exc)

    def desired_targets(self):
        with self.lock:
            msg, stamp = self.latest, self.last_cmd
        if time.monotonic() - stamp > self.timeout:
            return [0, 0, 0, 0]
        v = max(-self.max_v, min(self.max_v, float(msg.linear.x)))
        w = max(-self.max_w, min(self.max_w, float(msg.angular.z)))
        if abs(v) < 1e-6 and abs(w) < 1e-6:
            return [0, 0, 0, 0]
        left = v - w * self.track * 0.5
        right = v + w * self.track * 0.5
        peak = max(abs(left), abs(right))
        if peak > self.max_v:
            left *= self.max_v / peak
            right *= self.max_v / peak
        scale = self.max_units / self.max_v
        raw = [round(left * scale), round(left * scale),
               round(right * scale), round(right * scale)]
        if abs(v) < 0.02 and abs(w) > 0.01:
            boosted = []
            for value in raw:
                value = round(value * self.turn_gain)
                if value:
                    value = int(math.copysign(max(abs(value), self.min_turn_units), value))
                boosted.append(value)
            raw = boosted
        return [max(-self.max_units, min(self.max_units, raw[i] * self.signs[i]))
                for i in range(4)]

    def ramp(self, desired):
        if not any(desired):
            self.current = [0, 0, 0, 0]
            return list(self.current)
        step = max(1, int(round(self.max_accel / self.rate_hz)))
        result = []
        for current, target in zip(self.current, desired):
            if current * target < 0:
                target = 0
            delta = target - current
            value = target if abs(delta) <= step else current + (step if delta > 0 else -step)
            result.append(int(value))
        self.current = result
        return list(result)

    def send(self, values):
        frame = ("$" + "{}:" + ",".join(["{}"] * 4) + "#").format(self.command, *values)
        self.ser.write(frame.encode("ascii"))
        self.ser.flush()

    def read_rx(self):
        count = self.ser.in_waiting
        if count:
            self.rx_pub.publish(self.ser.read(min(count, 512)).decode("ascii", errors="replace"))

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.connect()
            if self.ser is not None:
                try:
                    self.send(self.ramp(self.desired_targets()))
                    self.read_rx()
                except Exception as exc:
                    rospy.logerr("STM32 serial error: %s; closing", exc)
                    try:
                        self.ser.close()
                    except Exception:
                        pass
                    self.ser = None
                    self.current = [0, 0, 0, 0]
                    self.next_retry = time.monotonic() + 1.0
            rate.sleep()

    def shutdown(self):
        if self.ser is not None:
            try:
                for _ in range(3):
                    self.send([0, 0, 0, 0])
                    time.sleep(0.02)
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        self.current = [0, 0, 0, 0]

if __name__ == "__main__":
    rospy.init_node("stm32_base_driver")
    Stm32BaseDriver().run()
