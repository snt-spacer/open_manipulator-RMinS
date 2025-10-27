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
from control_msgs.action import GripperCommand
from control_msgs.msg import GripperCommand as GripperCommandMsg

class PegInHole(Node):
    def __init__(self):
        super().__init__('peg_in_hole')
        
        self.get_logger().info("🎓 Peg-in-Hole Demo Ready!")
        
        self.collision_publisher = self.create_publisher(CollisionObject, '/collision_object', 10)
        self.attached_publisher = self.create_publisher(AttachedCollisionObject, '/attached_collision_object', 10)
        self.gripper_action_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')
        
        self.move_action_client = ActionClient(self, MoveGroup, '/move_action')
        
        self.get_logger().info("Waiting for MoveIt action server...")
        self.move_action_client.wait_for_server()
        self.get_logger().info("MoveIt action server connected!")
        
        self.get_logger().info("Available functions:")
        self.get_logger().info(" - move_gripper_to(x, y, z)")
        self.get_logger().info(" - move_gripper_joints(joint_1, joint_2, joint_3, joint_4)")
        self.get_logger().info(" - open_gripper()")
        self.get_logger().info(" - close_gripper()")
        self.get_logger().info(" - set_gripper_position(position)")
        self.get_logger().info(" - spawn_cylinder(name, x, y, z)")
        self.get_logger().info(" - spawn_cube(name, x, y, z)")
        self.get_logger().info(" - attach_to_gripper(object_name)")
        self.get_logger().info(" - detach_from_gripper(object_name)")

    def wait(self, seconds):
        """Wait for specified seconds"""
        time.sleep(seconds)

    def move_gripper_to(self, x, y, z, roll=0.0, pitch=math.pi/2, yaw=0.0):
            """
            Move gripper to specified position and orientation
            Default orientation: gripper pointing down (pitch=90°)
            """
            try:
                # Create goal pose
                goal_pose = Pose()
                goal_pose.position.x = x
                goal_pose.position.y = y
                goal_pose.position.z = z
                
                # Convert Euler to quaternion (gripper down by default)
                cy = math.cos(yaw * 0.5)
                sy = math.sin(yaw * 0.5)
                cp = math.cos(pitch * 0.5)
                sp = math.sin(pitch * 0.5)
                cr = math.cos(roll * 0.5)
                sr = math.sin(roll * 0.5)
                
                goal_pose.orientation.w = cy * cp * cr + sy * sp * sr
                goal_pose.orientation.x = cy * cp * sr - sy * sp * cr
                goal_pose.orientation.y = sy * cp * sr + cy * sp * cr
                goal_pose.orientation.z = sy * cp * cr - cy * sp * sr
                
                # Create MoveIt goal
                goal_msg = MoveGroup.Goal()
                goal_msg.request.workspace_parameters.header.frame_id = "world"
                goal_msg.request.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses[0] = goal_pose
                goal_msg.request.group_name = "arm"
                goal_msg.request.num_planning_attempts = 5
                goal_msg.request.allowed_planning_time = 5.0
                goal_msg.request.max_velocity_scaling_factor = 0.5
                goal_msg.request.max_acceleration_scaling_factor = 0.5
                
                # Send goal
                future = self.move_action_client.send_goal_async(goal_msg)
                while not future.done():
                    rclpy.spin_once(self)
                
                self.get_logger().info(f"✅ Moved gripper to: ({x:.3f}, {y:.3f}, {z:.3f})")
                return True
                
            except Exception as e:
                self.get_logger().error(f"❌ Failed to move to: ({x:.3f}, {y:.3f}, {z:.3f}): {e}")
                return False

    def move_gripper_joints(self, joint1, joint2, joint3, joint4):
        """
        Move gripper using joint angles (degrees)
        
        Args:
            joint1, joint2, joint3, joint4: Joint angles in degrees
        """
        try:
            from moveit_msgs.action import MoveGroup
            from moveit_msgs.msg import Constraints, JointConstraint
            import math
            
            goal_msg = MoveGroup.Goal()
            goal_msg.request.group_name = "arm"
            goal_msg.request.num_planning_attempts = 5
            goal_msg.request.allowed_planning_time = 5.0
            
            joint_values_rad = [math.radians(joint1), math.radians(joint2), 
                            math.radians(joint3), math.radians(joint4)]
            
            constraints = Constraints()
            joint_names = ["joint1", "joint2", "joint3", "joint4"]
            
            for name, value in zip(joint_names, joint_values_rad):
                joint_constraint = JointConstraint()
                joint_constraint.joint_name = name
                joint_constraint.position = value
                joint_constraint.tolerance_above = 0.01
                joint_constraint.tolerance_below = 0.01
                joint_constraint.weight = 1.0
                constraints.joint_constraints.append(joint_constraint)
            
            goal_msg.request.goal_constraints.append(constraints)
            
            future = self.move_action_client.send_goal_async(goal_msg)
            while not future.done():
                rclpy.spin_once(self)
            
            self.get_logger().info(f"✅ Moved joints to: [{joint1:.1f}°, {joint2:.1f}°, {joint3:.1f}°, {joint4:.1f}°]")
            return True
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to move joints: {e}")
            return False
        
    def open_gripper(self):
        """Open gripper fully"""
        try:
            from control_msgs.msg import GripperCommand as GripperCommandMsg
            
            goal_msg = GripperCommand.Goal()
            goal_msg.command = GripperCommandMsg()
            goal_msg.command.position = 0.02    
            goal_msg.command.max_effort = 5.0  
            
            future = self.gripper_action_client.send_goal_async(goal_msg)
            while not future.done():
                rclpy.spin_once(self)
            
            self.get_logger().info("✅ Gripper opened")
            return True
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to open gripper: {e}")
            return False

    def close_gripper(self):
        """Close gripper fully"""
        try:
            
            goal_msg = GripperCommand.Goal()
            goal_msg.command = GripperCommandMsg()
            goal_msg.command.position = -0.011     
            goal_msg.command.max_effort = 10.0   
            
            future = self.gripper_action_client.send_goal_async(goal_msg)
            while not future.done():
                rclpy.spin_once(self)

            self.get_logger().info(f"✅ Gripper closed successfully")
            return True
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to close gripper: {e}")
            return False

    def set_gripper_position(self, position, max_effort=10.0):
        """
        Set gripper to a specific position.
        The range is -0.011 (closed) to 0.02 (open)
        
        Args:
            position: Target position (0.0 = closed, max_value = open)
            max_effort: Maximum effort to apply
        """
        try:
            from control_msgs.action import GripperCommand
            from control_msgs.msg import GripperCommand as GripperCommandMsg
            
            goal_msg = GripperCommand.Goal()
            goal_msg.command = GripperCommandMsg()
            goal_msg.command.position = float(position)
            goal_msg.command.max_effort = float(max_effort)

            if position < -0.011 or position > 0.02:
                raise ValueError("Position must be between -0.011 (closed) and 0.02 (open)")
            
            future = self.gripper_action_client.send_goal_async(goal_msg)
            while not future.done():
                rclpy.spin_once(self)
            
            self.get_logger().info(f"✅ Gripper set to position: {position}")
            return True
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to set gripper position: {e}")
            return False
        
    def spawn_cylinder(self, name, x, y, z, radius=0.012, height=0.05):
        """
        Spawn a cylinder in the planning scene
        Default: radius=0.012m, height=0.05m (fits in the hole)

        Warning: The cylinder only spawns in the MoveIt planning scene, do not panic if it does not appear in Gazebo!
        """
        try:
            collision_object = CollisionObject()
            collision_object.header.frame_id = "world"
            collision_object.header.stamp = self.get_clock().now().to_msg()
            collision_object.id = name
            
            cylinder = SolidPrimitive()
            cylinder.type = SolidPrimitive.CYLINDER
            cylinder.dimensions = [height, radius]
            
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z + height/2
            pose.orientation.w = 1.0
            
            collision_object.primitives.append(cylinder)
            collision_object.primitive_poses.append(pose)
            collision_object.operation = CollisionObject.ADD
            
            self.wait(1.0)
            self.collision_publisher.publish(collision_object)
            
            self.get_logger().info(f"✅ Spawned cylinder '{name}' at ({x:.3f}, {y:.3f}, {z:.3f})")
            return True
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to spawn cylinder: {e}")
            return False

    def spawn_cube(self, name, x, y, z, size=0.24):
        """
        Spawn a cube in the planning scene
        Default: size=0.24m (fits in the hole)
        """
        try:
            collision_object = CollisionObject()
            collision_object.header.frame_id = "world"
            collision_object.header.stamp = self.get_clock().now().to_msg()
            collision_object.id = name
            
            cube = SolidPrimitive()
            cube.type = SolidPrimitive.BOX
            cube.dimensions = [size, size, size]
            
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z + size/2
            pose.orientation.w = 1.0
            
            collision_object.primitives.append(cube)
            collision_object.primitive_poses.append(pose)
            collision_object.operation = CollisionObject.ADD
            
            self.wait(1.0)
            self.collision_publisher.publish(collision_object)
            
            self.get_logger().info(f"✅ Spawned cube '{name}' at ({x:.3f}, {y:.3f}, {z:.3f})")
            return True
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to spawn cube: {e}")
            return False

    def attach_to_gripper(self, object_name):
        """
        Attach an object to the gripper
        """
        try:
            attached_object = AttachedCollisionObject()
            attached_object.object.id = object_name
            attached_object.object.operation = CollisionObject.ADD
            attached_object.link_name = "link5"
            attached_object.touch_links = ["gripper_link", "left_finger", "right_finger"]
            
            self.wait(1.0)
            self.attached_publisher.publish(attached_object)
            
            self.get_logger().info(f"✅ Attached '{object_name}' to gripper")
            return True
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to attach object: {e}")
            return False

    def detach_from_gripper(self, object_name):
        """Detach an object from the gripper"""
        try:
            attached_object = AttachedCollisionObject()
            attached_object.object.id = object_name
            attached_object.object.operation = CollisionObject.REMOVE
            
            self.wait(1.0)
            self.attached_publisher.publish(attached_object)
            
            self.get_logger().info(f"✅ Detached '{object_name}' from gripper")
            return True
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to detach object: {e}")
            return False

    def detach_all(self):
        """Detach all objects from the gripper"""
        try:
            attached_object = AttachedCollisionObject()
            attached_object.object.operation = CollisionObject.REMOVE
            
            self.wait(1.0)
            self.attached_publisher.publish(attached_object)
            
            self.get_logger().info("✅ Detached all objects from gripper")
            return True
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to detach all objects: {e}")
            return False