#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
 
group_arm = moveit_commander.MoveGroupCommander("arm_group")
end_effector = moveit_commander.MoveGroupCommander("hand")

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory,queue_size=20)

end_effector.set_named_target("hand_open")
plan1 = end_effector.plan()
rospy.sleep(1)
end_effector.go(wait=True)

group_arm.set_named_target("hand_pickup")
plan2 = group_arm.plan()
rospy.sleep(1)
group_arm.go(wait=True)

end_effector.set_named_target("hand_close")
plan3 = end_effector.plan()
rospy.sleep(1)
end_effector.go(wait=True)

group_arm.set_named_target("pose_drop")
plan4 = group_arm.plan()
rospy.sleep(1)
group_arm.go(wait=True)

end_effector.set_named_target("hand_open")
plan5 = end_effector.plan()
rospy.sleep(1)
end_effector.go(wait=True)

moveit_commander.roscpp_shutdown()

def get_pose():
        t= Twist()
        o =Odometry()
        o.header.frame_id = 'odom'
        o.child_frame_id= 'base_link'
        init_pos_x = o.pose.pose.position.x
        init_pos_y = o.pose.pose.position.y
        print("initial positions: ",init_pos_x, init_pos_y)
        pub_twist = rospy.Publisher('/vehicle_assembly/cmd_vel', Twist, queue_size=10)
        
        rate = rospy.Rate(10) # 10hz
        
        Kp=0.2
        goal1_x_pos = -5.0
        goal1_y_pos = -5.0
        i=0
        j=0
        if(goal1_y_pos<0):
            y_vel = -1
        else:
            y_vel = 1
        if(goal1_x_pos<0):
            x_vel = -1
        else:
            x_vel = 1

        y1_dis= goal1_y_pos-init_pos_y
        x1_dis= goal1_x_pos-init_pos_x
        while(i!=(y1_dis+y_vel)):
            #print('\nupdated i:', i)
            t.linear.y = y_vel
            pub_twist.publish(t)
            i=i+y_vel
            #print("Odometry: values\n", o.pose.pose.position)
            rospy.sleep(1) 
        
        t.linear.y = 0
        pub_twist.publish(t)

        while(j!=(x1_dis)):
            #print('\nupdated j:', j)
            t.linear.x = x_vel
            pub_twist.publish(t)
            j=j+x_vel
            rospy.sleep(1)

        t.linear.x=0
        pub_twist.publish(t)

if __name__ == '__main__':
       try:
           get_pose()
       except rospy.ROSInterruptException:
           pass