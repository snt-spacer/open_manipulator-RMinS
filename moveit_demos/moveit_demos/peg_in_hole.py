#!/usr/bin/env python3
"""
DEMO Peg-in-Hole Task
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time
import math

# Standard ROS 2 messages imports
from geometry_msgs.msg import Pose, Point, Quaternion
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive

# MoveIt action interfaces
from moveit_msgs.action import MoveGroup

# Utility functions
from .utils import PegInHole 
    

def demo_solution(robot):
    """
    TODO: Implement your peg-in-hole solution HERE!
    Use the provided robot functions to complete the task assigned by the professor.
    """
    
    robot.get_logger().info("🚀 Starting peg-in-hole task...")

    # Example coordinates - MODIFY THESE!
    PEG_POSITION = [-0.1, -0.21, 0.01]      # Peg spawns here
    PEG_POS_DEG = [0, 0, 0, 0]         # Joint angles to pick up the peg, modify as needed
    FINAL_PEG_POS_DEG = [0, 19, 18, -2]  # Joint angles to place the peg in the hole, modify as needed
    HOLE_POSITION = [0.2, 0.1, 0.02]     # Hole is located here, modify as needed
    APPROACH_HEIGHT = 0.1                 # Safe height above objects
    
    # Clear any existing objects in the scene and detach any objects from the gripper
    robot.detach_all()


    # Step 1: Spawn the peg (cylinder)
    robot.get_logger().info("Step 1: Spawning the peg...")
    robot.spawn_cylinder("cylinder_1", PEG_POSITION[0], PEG_POSITION[1], PEG_POSITION[2])
    robot.wait(2)
    
    # Step 2: Move to approach position above peg
    robot.get_logger().info("Step 2: Moving above the peg...")
    robot.move_gripper_joints(PEG_POS_DEG[0], PEG_POS_DEG[1], PEG_POS_DEG[2], PEG_POS_DEG[3])
    robot.wait(1)
    
    # # Step 3: Move down to peg
    # robot.get_logger().info("Step 3: Moving down to the peg...")
    # robot.move(PEG_POSITION[0], PEG_POSITION[1], PEG_POSITION[2] + 0.03)
    # robot.wait(1)
    
    # Step 4: Attach peg to gripper
    robot.get_logger().info("Step 4: Attaching the peg...")
    robot.attach_to_gripper("cylinder_1")
    robot.wait(1)
    
    # Step 5: Lift peg
    robot.get_logger().info("Step 5: Lifting the peg...")
    robot.move_gripper_joints(FINAL_PEG_POS_DEG[0], FINAL_PEG_POS_DEG[1], FINAL_PEG_POS_DEG[2], FINAL_PEG_POS_DEG[3])
    robot.wait(1)
    
    # # Step 6: Move to approach position above hole
    # robot.get_logger().info("Step 6: Moving above the hole...")
    # robot.move_gripper_to(HOLE_POSITION[0], HOLE_POSITION[1], APPROACH_HEIGHT)
    # robot.wait(1)
    
    # # Step 7: Move down to insert peg in hole
    # robot.get_logger().info("Step 7: Inserting the peg into the hole...")
    # robot.move_gripper_to(HOLE_POSITION[0], HOLE_POSITION[1], HOLE_POSITION[2] + 0.02)
    # robot.wait(1)
    
    # # Step 8: Detach peg to drop it in hole
    # robot.get_logger().info("Step 8: Detaching the peg...")
    # robot.detach_from_gripper("cylinder_1")
    # robot.wait(1)
    
    # # Step 9: Move back to initial position
    # robot.get_logger().info("Step 9: Moving back to safe position...")
    # robot.move_gripper_to(HOLE_POSITION[0], HOLE_POSITION[1], APPROACH_HEIGHT)
    
    robot.get_logger().info("🎉 Peg-in-hole task completed!")

def main():
    rclpy.init()
    
    try:
        robot = PegInHole()
        robot.wait(3)
        
        demo_solution(robot)
        
        robot.get_logger().info("✅ All tasks completed successfully!")
        
    except Exception as e:
        robot.get_logger().error(f"💥 Error in demo solution: {e}")
    
    finally:
        # Cleanup
        robot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()