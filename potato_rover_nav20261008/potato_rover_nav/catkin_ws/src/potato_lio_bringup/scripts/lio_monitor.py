#!/usr/bin/env python3
import threading
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from livox_ros_driver2.msg import CustomMsg
class Monitor:
    def __init__(self):
        self.lock=threading.Lock(); self.last={'lidar':None,'imu':None,'odom':None}; self.count={k:0 for k in self.last}
        rospy.Subscriber('/livox/lidar',CustomMsg,lambda m:self.cb('lidar'),queue_size=1)
        rospy.Subscriber('/livox/imu',Imu,lambda m:self.cb('imu'),queue_size=20)
        rospy.Subscriber('/Odometry',Odometry,lambda m:self.cb('odom'),queue_size=5)
        rospy.Timer(rospy.Duration(1.0),self.report)
    def cb(self,key):
        with self.lock:self.last[key]=rospy.Time.now();self.count[key]+=1
    def report(self,_):
        now=rospy.Time.now()
        with self.lock:
            text=[]
            for key in ('lidar','imu','odom'):
                age=float('inf') if self.last[key] is None else (now-self.last[key]).to_sec(); text.append('%s=%s age=%.2fs count=%d'%(key,'OK' if age<1.0 else 'LOST',age,self.count[key]));self.count[key]=0
        rospy.loginfo('LIO STATUS %s',' | '.join(text))
if __name__=='__main__':rospy.init_node('lio_monitor');Monitor();rospy.spin()
