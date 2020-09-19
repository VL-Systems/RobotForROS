#!/usr/bin/env python
# coding: utf-8

import rospy
import tf2_ros
import tf2_geometry_msgs
import numpy as np

from fiducial_msgs.msg import FiducialTransformArray
from geometry_msgs.msg import PoseStamped

class aruco_reacher(object):
    def __init__(self):
       
       rospy.init_node("aruco_reacher")
       
       # Inner vars
       self.locked_goal = None
       
       ## params
       self.dr_threshold = rospy.get_param("~dr_threshold", 0.1)
       
       self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
       self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
       
       # Publishers 
       self.move_base_goal = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size = 1)
       
       # Subscribers
       rospy.Subscriber("fiducial_transforms", FiducialTransformArray, self.fiducial_cb)
       
    def fiducial_cb(self, msg):
        if len(msg.transforms) > 0:
            goal = PoseStamped()
            
            goal.header = msg.header
            goal.pose.position.x = msg.transforms[0].transform.translation.x
            goal.pose.position.y = msg.transforms[0].transform.translation.y
            goal.pose.position.z = msg.transforms[0].transform.translation.z
            
            goal.pose.orientation = msg.transforms[0].transform.rotation
            
            transform = self.tf_buffer.lookup_transform("map",
                                       goal.header.frame_id, #source frame
                                       rospy.Time(0), #get the tf at first available time
                                       rospy.Duration(1.0)) #wait for 1 second
            
            goal_transformed = tf2_geometry_msgs.do_transform_pose(goal, transform)
            
            if self.locked_goal is None:
                self.locked_goal = goal_transformed
                self.move_base_goal.publish(self.locked_goal)
                
            else:                
                dx = self.locked_goal.pose.position.x - goal_transformed.pose.position.x
                dy = self.locked_goal.pose.position.y - goal_transformed.pose.position.y
                
                dr = np.sqrt(dx**2 + dy**2)
                
                if dr > self.dr_threshold:
                    rospy.loginfo("[aruco_reacher] dr ({}) is higher then dr_threshold ({}), resend goal.".format(dr, self.dr_threshold))
                    self.locked_goal = goal_transformed
                    self.move_base_goal.publish(self.locked_goal)                    
       
    def run(self):
        rospy.spin()
        
        
if __name__ == '__main__' :
    ar = aruco_reacher()
    ar.run()
        
