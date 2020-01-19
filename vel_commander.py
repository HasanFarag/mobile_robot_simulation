#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist 
import math
import tf
from tf.transformations import euler_from_quaternion
import sys
class robot(object):
    def __init__(self,robot_no):
        self.pub = rospy.Publisher('robot' + str(robot_no) + '/cmd_vel', Twist, queue_size=10)
        
    def move(self,lin_vel,ang_vel):
        cmd_vel = Twist()
        cmd_vel.linear.x = lin_vel
        cmd_vel.angular.z = ang_vel
        self.pub.publish(cmd_vel)

    def stop(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0
        cmd_vel.angular.z = 0
        self.pub.publish(cmd_vel)
        

if __name__ == '__main__':
    rospy.init_node('cat_mouse_game', anonymous=True,disable_signals=False)
    cat = robot(1)
    mouse = robot(2)
    tf_listener = tf.TransformListener()
    rate = rospy.Rate(10) # 10hz
    first_time = True
    while not rospy.is_shutdown():    
        if first_time:
            tf_listener.waitForTransform('/robot1/odom','/robot2/odom',rospy.Time(0), rospy.Duration(5.0))
            start_time = rospy.get_time()
            first_time = False
        # cat_ang_vel <= 0.2 
        cat.move(0.7,cat_ang_vel)
        mouse.move(0.2,mouse_ang_vel)
        # mouse_ang_vel <= 0.9
        tf_listener.waitForTransform('/robot1/robot_footprint','/robot2/robot_footprint',rospy.Time(0), rospy.Duration(1.0))
        (trans,rot) = tf_listener.lookupTransform('/robot1/robot_footprint', '/robot2/robot_footprint', rospy.Time(0))
        frame_shift = 1
        #print('x',trans[0]-frame_shift)
        #print('y',trans[1]-frame_shift)
        inter_distance = math.sqrt((trans[0]-frame_shift) ** 2 + (trans[1]-frame_shift) ** 2)
        #print('inter-distance is ' + str(inter_distance))
        inter_angle_rpy =  euler_from_quaternion(rot)
        inter_angle = inter_angle_rpy[2]
        #print('inter-angle is ' + str(inter_angle))
        elapsed_time = rospy.get_time() - start_time
        print('elapsed time is ' + str(elapsed_time))
        if inter_distance < 0.5 and elapsed_time < 60.0:
            print('The cat won the game')
            cat.stop()
            mouse.stop()
            rospy.signal_shutdown('Shutdown')
            sys.exit(1)
        elif elapsed_time > 60.0:  
            print('The mouse won the game')
            cat.stop()
            mouse.stop()  
            rospy.signal_shutdown('Shutdown')
            sys.exit(1)                                                                                                                                                                                                                                                                                                                                                                                                                              
        rate.sleep()
    
    rospy.spin() 
    
   
