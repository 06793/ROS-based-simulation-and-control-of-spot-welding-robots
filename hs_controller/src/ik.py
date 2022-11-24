#!/usr/bin/env python


import rospy, sys
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from hs_controller.arm_utils import scale_trajectory_speed

class MoveItDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)
        
        rospy.init_node('moveit_demo', anonymous=True)
        
        GRIPPER_OPEN = [0.30]
        GRIPPER_CLOSED = [0.003]
        GRIPPER_NEUTRAL = [0.1]
                
        # Initialize the move group for the right arm
        arm = moveit_commander.MoveGroupCommander('arm')

        # Connect to the gripper move group
        gripper = moveit_commander.MoveGroupCommander('gripper')
           
        # Get the name of the end-effector link
        end_effector_link = arm.get_end_effector_link()
                        
        # Set the reference frame for pose targets
        reference_frame = 'base_link'
        
        # Set the right arm reference frame accordingly
        arm.set_pose_reference_frame(reference_frame)
                
        # Allow replanning to increase the odds of a solution
        arm.allow_replanning(True)
        
        # Allow some leeway in position (meters) and orientation (radians)
        arm.set_goal_position_tolerance(0.01)
        arm.set_goal_orientation_tolerance(0.05)
        gripper.set_goal_joint_tolerance(0.001)
        
        # Start the arm in the "resting" pose stored in the SRDF file
        arm.set_named_target('resting')
        arm.go()
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

        # Set the target pose.  This particular pose has the gripper oriented horizontally
        # 0.85 meters above the ground, 0.10 meters to the right and 0.20 meters ahead of 
        # the center of the robot base.
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = 1.3
        target_pose.pose.position.y = 0.6
        target_pose.pose.position.z = 1.3
        target_pose.pose.orientation.x = 0.119641717163
        target_pose.pose.orientation.y = -0.791079259511
        target_pose.pose.orientation.z = -0.58825415617
        target_pose.pose.orientation.w = 0.117628705827
        
        # Set the start state to the current state
        arm.set_start_state_to_current_state()
        
        # Set the goal pose of the end effector to the stored pose
        arm.set_pose_target(target_pose, end_effector_link)
        
        # Plan the trajectory to the goal
        traj = arm.plan()
        
        # Execute the planned trajectory
        arm.execute(traj)
    
        # Pause for a second
        rospy.sleep(1)

        i = 1
        while (i <= 5):

          # Close the gripper as if picking something up
          gripper.set_joint_value_target(GRIPPER_CLOSED)
          gripper.go()
          rospy.sleep(5)
                 
          # Open the gripper as if letting something go
          gripper.set_joint_value_target(GRIPPER_OPEN)
          gripper.go()
          rospy.sleep(1)
          
          # Shift the end-effector to the right 5cm
          arm.shift_pose_target(0, 0.20, end_effector_link)

          # Get back the planned trajectory
          traj = arm.plan()
        
          # Scale the trajectory speed by a factor of 0.25
          new_traj = scale_trajectory_speed(traj, 0.25)

          # Execute the new trajectory     
          arm.execute(new_traj)
          rospy.sleep(1)

          i=i+1
   
        # Finish up in the resting position  
        arm.set_named_target('resting')
        arm.go()
        rospy.sleep(1)

        # Open the gripper as if letting something go
        gripper.set_joint_value_target(GRIPPER_NEUTRAL)
        gripper.go()

        # Shut down MoveIt cleanly
        moveit_commander.roscpp_shutdown()
        
        # Exit MoveIt
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    MoveItDemo()

    
    
