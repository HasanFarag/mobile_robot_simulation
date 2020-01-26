#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist 


class drone(object):

    def __init__(self):
        self.take_off_pub = rospy.Publisher('/drone/takeoff', Empty, queue_size=10)
        self.land_pub = rospy.Publisher('/drone/land', Empty, queue_size=10)
        self.posctrl_pub = rospy.Publisher('/drone/posctrl', Bool, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/drone/cmd_val', Twist, queue_size=10)
        rospy.loginfo('drone started working')

    
    def takeoff(self):
        start = rospy.get_time()
        empty = Empty()
        while (rospy.get_time()-start) < 0.5:     
            self.take_off_pub.publish(empty)  
        rospy.loginfo('drone take_off')
        rospy.sleep(2.0)
        

    def land(self):
        start = rospy.get_time()
        empty = Empty()
        while (rospy.get_time()-start) < 0.5:     
            self.land_pub.publish(empty)
        rospy.loginfo('drone land')
        rospy.sleep(2.0)

    def posctrl(self):
        start = rospy.get_time()
        bool = Bool()
        bool.data = True
        while (rospy.get_time()-start) < 0.5:     
            self.posctrl_pub.publish(bool)
        rospy.loginfo('drone posctrl')
        rospy.sleep(1.0)

    def move(self,vx = 0.0,vy = 0.0 ,vz = 0.0,wx = 0.0 ,wy = 0.0,wz = 0.0):
        start = rospy.get_time()
        cmd_vel = Twist()
        cmd_vel.linear.x = vx
        cmd_vel.linear.y = vy
        cmd_vel.linear.z = vz
        cmd_vel.angular.x = wx
        cmd_vel.angular.y = wy
        cmd_vel.angular.z = wz
        while (rospy.get_time()-start) < 0.5: 
            self.cmd_vel_pub.publish(cmd_vel)
        rospy.loginfo('drone velocity control')
        rospy.sleep(10.0)
if __name__ == "__main__":

    rospy.init_node('drone')
    drone = drone()
    #rate = rospy.Rate(10) # 10hz
    #while not rospy.is_shutdown():
    #drone.takeoff()
    drone.posctrl()
    #drone.move(0.0,0.0,5.0)
    drone.move(5.0,0.0,0.0)
    
    #drone.land()
    #rate.sleep()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
