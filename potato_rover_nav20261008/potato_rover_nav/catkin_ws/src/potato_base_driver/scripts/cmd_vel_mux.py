#!/usr/bin/env python3
import threading
import rospy
from geometry_msgs.msg import Twist

class CmdVelMux:
    def __init__(self):
        self.lock=threading.Lock(); self.teleop=Twist(); self.nav=Twist()
        self.teleop_stamp=rospy.Time(0); self.nav_stamp=rospy.Time(0)
        self.teleop_timeout=rospy.Duration(float(rospy.get_param('~teleop_timeout',0.30)))
        self.nav_timeout=rospy.Duration(float(rospy.get_param('~nav_timeout',0.30)))
        teleop=rospy.get_param('~teleop_topic','/cmd_vel/teleop'); nav=rospy.get_param('~nav_topic','/cmd_vel/nav'); output=rospy.get_param('~output_topic','/cmd_vel/safe')
        self.pub=rospy.Publisher(output,Twist,queue_size=1)
        rospy.Subscriber(teleop,Twist,self.teleop_cb,queue_size=1,tcp_nodelay=True)
        rospy.Subscriber(nav,Twist,self.nav_cb,queue_size=1,tcp_nodelay=True)
        self.timer=rospy.Timer(rospy.Duration(1.0/float(rospy.get_param('~publish_rate',20.0))),self.tick)
        rospy.loginfo('cmd_vel mux: teleop=%s priority; nav=%s; output=%s',teleop,nav,output)
    def teleop_cb(self,msg):
        with self.lock: self.teleop,self.teleop_stamp=msg,rospy.Time.now()
    def nav_cb(self,msg):
        with self.lock: self.nav,self.nav_stamp=msg,rospy.Time.now()
    def tick(self,_event):
        now=rospy.Time.now()
        with self.lock:
            if now-self.teleop_stamp<=self.teleop_timeout: out=self.teleop
            elif now-self.nav_stamp<=self.nav_timeout: out=self.nav
            else: out=Twist()
        self.pub.publish(out)
if __name__=='__main__':
    rospy.init_node('cmd_vel_mux'); CmdVelMux(); rospy.spin()
