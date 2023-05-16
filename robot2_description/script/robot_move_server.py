#!/usr/bin/env python3
import rospy
import tf
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Quaternion, Pose, Point
from std_msgs.msg import Float32
from your_package.srv import Location


class go_to_goal():
    def __init__(self):
        rospy.init_node('nav_test', anonymous=False)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("wait for the action server to come up")
        self.move_base.wait_for_server(rospy.Duration(5))
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'base_link'
        goal.target_pose.header.stamp = rospy.Time.now()
        # goal.target_pose.pose.position.x = 2.0 #3 meters
        goal.target_pose.pose.position.x = 0.5 #3 meters
        goal.target_pose.pose.position.y = 0.3 #3 meters
        
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, np.pi/2)
        goal.target_pose.pose.orientation = Quaternion(*q)

        #start moving
        self.move_base.send_goal(goal)
        #allow TurtleBot up to 60 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(60))
        if not success:
            self.move_base.cancel_goal()
            rospy.loginfo("The base failed to move forward %s meters for some reason",self.a)
        else:
            # We made it!
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Parking successed")

    def shutdown(self):
        stop_goal = MoveBaseGoal()
        self.move_base.send_goal(stop_goal)
        rospy.loginfo("Stop")

if __name__ == '__main__':
    go_to_goal()