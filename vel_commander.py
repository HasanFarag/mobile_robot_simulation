#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist 
import math
import tf
from tf.transformations import euler_from_quaternion
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
    rospy.init_node('cat_mouse_game', anonymous=True)
    cat = robot(1)
    mouse = robot(2)
    tf_listener = tf.TransformListener()
    rate = rospy.Rate(10) # 10hz
    first_time = True
    while not rospy.is_shutdown():
        start = rospy.get_time()
        mouse.move(0.1,0.0)
        if first_time:
            tf_listener.waitForTransform('/robot1/odom','/robot2/odom',rospy.Time(0), rospy.Duration(5.0))
            first_time = False
        tf_listener.waitForTransform('/robot1/robot_footprint','/robot2/robot_footprint',rospy.Time(0), rospy.Duration(1.0))
        (trans,rot) = tf_listener.lookupTransform('/robot1/robot_footprint', '/robot2/robot_footprint', rospy.Time(0))
        frame_shift = 1
        print('x',trans[0]-frame_shift)
        print('y',trans[1]-frame_shift)
        inter_distance = math.sqrt((trans[0]-frame_shift) ** 2 + (trans[1]-frame_shift) ** 2)
        inter_angle_rpy =  euler_from_quaternion(rot)
        inter_angle = inter_angle_rpy[2]
        time_elapsed = rospy.get_time()-start
        print('inter-distance is ' + str(inter_distance))
        print('inter-angle is ' + str(inter_angle))
        print('elapsed time is ' + str(time_elapsed))
            #
            #mouse.move(5.0,4.0)
            #cat.stop()
            #mouse.stop()
        rate.sleep()
    try:
        rospy.spin() 
    except rospy.ROSInterruptException:
        pass
   
