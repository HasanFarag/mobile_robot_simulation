#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist 
import math
import tf
from tf.transformations import euler_from_quaternion
import sys
import numpy as np
from geometry_msgs.msg import TransformStamped, Vector3Stamped
from geometry_msgs.msg import Point
import random as rnd
import tf2_geometry_msgs
cat_max_lin_vel = 0.7
mouse_max_lin_vel = 0.2
cat_max_ang_vel = 0.2
mouse_max_ang_vel = 0.9
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
        
def test_similar(test_name, v, x, y, z):
    tolerance = 1e-6
    if np.abs(np.array([v.x, v.y, v.z]) - np.array([x, y, z])).max() > tolerance:
        print 'FAIL:', test_name
        print 'expected:', x, y, z
        print 'actual  :', v.x, v.y, v.z

def limit_cos(cos_angle):
    return min(1,max(cos_angle,-1))
if __name__ == '__main__':
    rospy.init_node('cat_mouse_game', anonymous=True,disable_signals=False)
    cat = robot(1)
    mouse = robot(2)
    tf_listener = tf.TransformListener()
    rate = rospy.Rate(10) # 100hz
    first_time = True
    inter_distance_prev = 0
    angle1_prev=0
    angle2_prev=0

    while not rospy.is_shutdown():    
        if first_time:
            tf_listener.waitForTransform('/robot1/odom','/robot2/odom',rospy.Time(0), rospy.Duration(5.0))
            start_time = rospy.get_time()
            first_time = False
        # cat_ang_vel <= 0.2 
        #cat.move(cat_max_lin_vel,rnd.uniform(0.0,cat_max_ang_vel))
        cat.move(cat_max_lin_vel,0.0)
        #mouse.move(mouse_max_lin_vel,rnd.uniform(0.0,mouse_max_ang_vel))
        mouse.move(cat_max_lin_vel,1.8)
        # mouse_ang_vel <= 0.9 
        try:
            tf_listener.waitForTransform('/robot1/robot_footprint','/robot2/robot_footprint',rospy.Time(0), rospy.Duration(1.0))
            (trans,rot) = tf_listener.lookupTransform('/robot1/robot_footprint', '/robot2/robot_footprint', rospy.Time(0))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        frame_shift = 1
 
        inter_distance = math.sqrt((trans[0]-frame_shift) ** 2 + (trans[1]-frame_shift) ** 2)
        
        #print('inter_distance is ' + str(inter_distance))
        ##angle = math.atan2(trans[1]-frame_shift, trans[0]-frame_shift)
        v1 = Vector3Stamped()
        v1.vector.x = trans[0]
        v1.vector.y = trans[1]
        v1.vector.z = 0
        v2 = Vector3Stamped()
        v2.vector.x = -trans[0]
        v2.vector.y = -trans[1]
        v2.vector.z = 0
        tf_listener.waitForTransform('/world','/robot1/robot_footprint',rospy.Time(0), rospy.Duration(1.0))
        (trans1,rot1) = tf_listener.lookupTransform('/world', '/robot1/robot_footprint', rospy.Time(0))
        #print(trans1)
        #print(rot1)
        tf_listener.waitForTransform('/world','/robot2/robot_footprint',rospy.Time(0), rospy.Duration(1.0))
        (trans2,rot2) = tf_listener.lookupTransform('/world', '/robot2/robot_footprint', rospy.Time(0))
        trans2[0] = trans2[0]-frame_shift
        trans2[1] = trans2[1]-frame_shift
        #print(trans2)
        #print(rot2)
        x = Vector3Stamped()
        x.vector.x = 1
        x.vector.y = 0
        x.vector.z = 0
        t1 = TransformStamped()
        t1.header.frame_id = "/robot1/robot_footprint"
        t1.header.stamp = rospy.get_time()
        #t1.transform.translation.x=trans1[0]
        #t1.transform.translation.x=trans1[1]
        #t1.transform.translation.x=trans1[2]
        t1.transform.rotation.x = rot1[0]
        t1.transform.rotation.y = rot1[1]
        t1.transform.rotation.z = rot1[2]
        t1.transform.rotation.w = rot1[3]
        x1t = tf2_geometry_msgs.do_transform_vector3(x, t1)
        v1t = tf2_geometry_msgs.do_transform_vector3(v1, t1)
        #print(x1t)
       
        t2 = TransformStamped()
        t2.header.frame_id = "/robot2/robot_footprint"
        ##q = tf.transformations.quaternion_from_euler(0, 0, math.radians(90))
        t2.transform.rotation.x = rot2[0]
        t2.transform.rotation.y = rot2[1]
        t2.transform.rotation.z = rot2[2]
        t2.transform.rotation.w = rot2[3]
        x2t = tf2_geometry_msgs.do_transform_vector3(x, t2)
        v2t = tf2_geometry_msgs.do_transform_vector3(v2, t2)
        #print(x2t)
        ##test_similar('rotation', vt.vector, 0, 1, 0)
        ###x1 = t1.inverseTimes(t2) * x;
        ###x2 = t1.inverseTimes(t2) * x;
        angle1 = np.arccos(limit_cos(np.dot([x1t.vector.x,x1t.vector.y],[v1t.vector.x,v1t.vector.y])))
        angle2 = np.arccos(limit_cos(np.dot([x2t.vector.x,x2t.vector.y],[v2t.vector.x,v2t.vector.y])))

        elapsed_time = rospy.get_time() - start_time
        inter_distance_dot = (inter_distance-inter_distance_prev)/elapsed_time
        angle1_dot= (angle1-angle1_prev)/elapsed_time
        angle2_dot= (angle2-angle2_prev)/elapsed_time
            #A = np.array([[-math.cos(), 0.0],
         #              [math.sin()/, -1.0],
         #              [math.sin()/, 0])
         #B = np.array([[-math.cos(), 0.0],
         #              [math.sin()/, -1.0],
         #              [-math.sin()/, 0])
         #u = np.array([[ux], [yawrate]])
         #V = np.array([[v], [yawrate]])
         #rates= np.dot(A,u) + np.dot(B,v)
         #u = np.dot(np.linalg.inv(A),(rates-np.dot(B,v))
         #u[0,0] = 0.7
         # print([1,0])
         #v[0,0] = 0.2
         # v[1,0]= nn.random(range)

        #print('elapsed time is ' + str(elapsed_time))
     
        #trans1_mat = tf.transformations.translation_matrix(trans1)
        #rot1_mat   = tf.transformations.quaternion_matrix(rot1)
        #mat1 = numpy.dot(trans1_mat, rot1_mat)      #

        #(trans2, rot2) = tf.lookupTransform(l4, l3, t)
        #trans2_mat = tf.transformations.translation_matrix(trans2)
        #rot2_mat    = tf.transformations.quaternion_matrix(rot2)
        #mat2 = numpy.dot(trans2_mat, rot2_mat)      #

        #mat3 = numpy.dot(mat1, mat2)
        #trans3 = tf.transformations.translation_from_matrix(mat3)
        #rot3 = tf.transformations.quaternion_from_matrix(mat3)      #

        #br = tf.TransformBroadcaster()
        #br.sendTransform(
        #  trans3,
        #  rot3,
        #  t,
        #  "target",
        #  "source");
        #print('inter-distance is ' + str(inter_distance))
        ##inter_angle_rpy =  euler_from_quaternion(rot)
        ##inter_angle = inter_angle_rpy[2]
        #print('inter-angle is ' + str(inter_angle))
       
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
    
   
