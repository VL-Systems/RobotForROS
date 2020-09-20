#! /usr/bin/env python

import numpy as np

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped,Twist
import actionlib
import mybot_2dnav.msg

class RotationActionServer(object):
    
    def __init__(self, name):                
        
        # Parameters
        self.def_speed = rospy.get_param("~default_speed", 0.5)
        
        # Inner vars
        self.current_angle_proc = 0.0
        self.prev_time = None
        self.goal_active = False
        #self.sleep_rate = rospy.Rate(0.1)                
        
        # Publishers
        self.twist_pub = rospy.Publisher("cmd_vel", Twist, queue_size = 1 )

        # Start action server
        self._as = actionlib.SimpleActionServer(name,
                                                mybot_2dnav.msg.RotationAction,
                                                execute_cb=self.execute_cb,
                                                auto_start = False)
                
        # Subscribers        
        rospy.Subscriber("odom", Odometry, self.odom_cb )
        
        self._as.start()
        
    def execute_cb(self, goal):
        
        _feedback = mybot_2dnav.msg.RotationFeedback()
        _result = mybot_2dnav.msg.RotationResult()
        
        direction = np.sign(goal.angle)
        cmd = Twist()        
        cmd.angular.z = self.def_speed * direction     
        
        self.current_angle_proc = 0.0
        self.goal_active = True
        while(True):
            
            if self._as.is_preempt_requested():
                self._as.set_preempted()
                cmd = Twist()
                self.twist_pub.publish(cmd)
                self.goal_active = False
                return
            
            _feedback.angle = self.current_angle_proc
            if direction > 0:
                if self.current_angle_proc < goal.angle:
                    self.twist_pub.publish(cmd)                    
                else:                    
                    break
            else:
                if self.current_angle_proc > goal.angle:
                    self.twist_pub.publish(cmd)                                        
                else:
                    break
                    
            self._as.publish_feedback(_feedback)
            #self.sleep_rate.sleep()            
            
        cmd = Twist()
        self.twist_pub.publish(cmd)
        _result.angle = self.current_angle_proc
        self._as.set_succeeded(_result)         
        self.goal_active = False
                                
    def odom_cb(self, msg):
        if self.goal_active:
            if self.prev_time is None:
                self.prev_time = rospy.Time.now()
                return
            ct = rospy.Time.now()            
            dt = (ct - self.prev_time).to_sec()        
            self.prev_time = ct
            self.current_angle_proc += msg.twist.twist.angular.z * dt
        
            
    def run(self):
        rospy.spin()
        

if __name__ == '__main__':
    rospy.init_node('rotation_action')    
    ras = RotationActionServer(rospy.get_name())
    ras.run()
    
