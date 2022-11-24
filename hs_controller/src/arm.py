#!/usr/bin/env python


import rospy, sys
import moveit_commander
from control_msgs.msg import GripperCommand

class MoveItDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the ROS node
        rospy.init_node('moveit_demo', anonymous=True)
        
        GRIPPER_OPEN = [0.30]
        GRIPPER_CLOSED = [0.003]
        GRIPPER_NEUTRAL = [0.1]
 
        # Connect to the arm move group
        arm = moveit_commander.MoveGroupCommander('arm')
        
        # Connect to the gripper move group
        gripper = moveit_commander.MoveGroupCommander('gripper')
                
        # Get the name of the end-effector link
        end_effector_link = arm.get_end_effector_link()
        
        # Display the name of the end_effector link
        rospy.loginfo("The end effector link is: " + str(end_effector_link))
        
        # Set a small tolerance on joint angles
        arm.set_goal_joint_tolerance(0.001)
        gripper.set_goal_joint_tolerance(0.001)
        
        # Start the arm target in "resting" pose stored in the SRDF file
        arm.set_named_target('resting')
        
        # Plan a trajectory to the goal configuration
        traj = arm.plan()
         
        # Execute the planned trajectory
        arm.execute(traj)
        
        # Pause for a moment
        rospy.sleep(1)
         
        # Set the gripper target to neutal position using a joint value target
        gripper.set_joint_value_target(GRIPPER_NEUTRAL)
         
        # Plan and execute the gripper motion
        gripper.go()
        rospy.sleep(1)
        
        # Open the gripper as if letting something go
        gripper.set_joint_value_target(GRIPPER_OPEN)
        gripper.go()
        rospy.sleep(1)          
         
        # Set the arm target to the named "straight_out" pose stored in the SRDF file
        arm.set_named_target('working2')
         
        # Plan and execute the motion
        arm.go()
        rospy.sleep(1)

        # Open the gripper as if letting something go
        gripper.set_joint_value_target(GRIPPER_OPEN)
        gripper.go()
        rospy.sleep(1)
                  
        # Close the gripper as if picking something up
        gripper.set_joint_value_target(GRIPPER_CLOSED)
        gripper.go()
        rospy.sleep(5)
                 
        # Open the gripper as if letting something go
        gripper.set_joint_value_target(GRIPPER_OPEN)
        gripper.go()
        rospy.sleep(1)
         
        # Return the arm to the named "resting" pose stored in the SRDF file
        arm.set_named_target('resting')
        arm.go()
        rospy.sleep(1)
         
        # Return the gripper target to neutral position
        gripper.set_joint_value_target(GRIPPER_NEUTRAL)
        gripper.go()
        rospy.sleep(1)
        
        # Cleanly shut down MoveIt
        moveit_commander.roscpp_shutdown()
        
        # Exit the script
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItDemo()
    except rospy.ROSInterruptException:
        pass
    
