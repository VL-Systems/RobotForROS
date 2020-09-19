#!/usr/bin/env python
# coding: utf-8

import rospy
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from tf.transformations import euler_from_quaternion

from fiducial_msgs.msg import FiducialTransformArray
from geometry_msgs.msg import PoseStamped
import move_base_msgs.msg
import mybot_2dnav.msg
import actionlib

class aruco_searcher(object):
    def __init__(self):
        rospy.init_node("aruco_searcher")
       
        # Inner vars
        self.locked_goal = None
       
        ## params
        self.dr_threshold = rospy.get_param("~dr_threshold", 0.1)
        self.dyaw_threshold = rospy.get_param("~dyaw_threshold", 0.1)
        self.angle_inc = rospy.get_param("~angle_inc", 0.1)
       
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # connect to rotation_server
        self.rotation_client = actionlib.SimpleActionClient('rotation_action_server', mybot_2dnav.msg.RotationAction)
        rospy.loginfo("[aruco_searcher] waiting for rotation action server...")
        self.rotation_client.wait_for_server()
        
        # connect to move_base
        self.move_client = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)
        rospy.loginfo("[aruco_searcher] waiting for move_base...")
        self.move_client.wait_for_server()
        
        # Timer
        rospy.Timer(rospy.Duration(1.), self.timer_cb)
        
        # Subscribers
        rospy.Subscriber("fiducial_transforms", FiducialTransformArray, self.fiducial_cb)
        
        
    def timer_cb(self, event):
        if self.locked_goal is None:
            if self.rotation_client.get_state() != 1:
            
                goal = mybot_2dnav.msg.RotationGoal()
                goal.angle = self.angle_inc
            
                self.rotation_client.send_goal(goal)
        
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
                self.sent_cmd_to_move_base(goal_transformed)
                rospy.loginfo("[aruco_searcher] goal sent.")                
                
            else:                
                dx = self.locked_goal.pose.position.x - goal_transformed.pose.position.x
                dy = self.locked_goal.pose.position.y - goal_transformed.pose.position.y
                
                dr = np.sqrt(dx**2 + dy**2)
                
                if dr > self.dr_threshold:
                    rospy.loginfo("[aruco_searcher] dr ({}) is higher then dr_threshold ({}), goal resent.".format(dr, self.dr_threshold))
                    self.sent_cmd_to_move_base(goal_transformed)
                    
                else:
                    quat_tf_1 = [self.locked_goal.pose.orientation.x, 
                               self.locked_goal.pose.orientation.y,
                               self.locked_goal.pose.orientation.z,
                               self.locked_goal.pose.orientation.w]
                    
                    quat_tf_2 = [goal_transformed.pose.orientation.x, 
                               goal_transformed.pose.orientation.y,
                               goal_transformed.pose.orientation.z,
                               goal_transformed.pose.orientation.w]
                    
                    yaw1 = euler_from_quaternion(quat_tf_1)[2]
                    yaw2 = euler_from_quaternion(quat_tf_2)[2]
                    
                    dyaw = abs(np.arctan2( np.sin(yaw1-yaw2), np.cos(yaw1-yaw2)))
                    
                    if dyaw > self.dyaw_threshold:
                        rospy.loginfo("[aruco_searcher] dyaw ({}) is higher then dyaw_threshold ({}), goal resent.".format(dyaw, self.dyaw_threshold))
                        self.sent_cmd_to_move_base(goal_transformed)
                    
    def sent_cmd_to_move_base(self, pose):
        if self.rotation_client.get_state() != 3:
            self.rotation_client.cancel_goal()
        
        self.locked_goal = pose 
        move_goal = move_base_msgs.msg.MoveBaseGoal()
        move_goal.target_pose = self.locked_goal
        self.move_client.send_goal(move_goal, done_cb = self.move_action_done_cb)
        
                    
    def move_action_done_cb(self, state, result):        
        rospy.loginfo("[aruco_searcher] move base action done with state [{}]".format(state))
        self.locked_goal = None
                    
    def run(self):
        rospy.spin()
        
        
if __name__ == '__main__' :
    as_ = aruco_searcher()
    as_.run()
                        
