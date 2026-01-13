#!/usr/bin/env python3
"""
Execute waypoints - PRODUCTION READY
Handles collision (abort/go_home) and configuration errors (skip) separately
"""

import sys
import json
import math
import time
import signal
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy. action import ActionClient

from geometry_msgs.msg import Pose, Point
from shape_msgs.msg import Mesh as ShapeMesh, MeshTriangle
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory

from moveit_msgs.msg import (
    CollisionObject, PlanningScene, MotionPlanRequest, 
    Constraints, JointConstraint, PlanningOptions, RobotState
)
from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan, GetPositionIK
from moveit_msgs.action import MoveGroup
import struct

# ==================== CONFIGURATION ====================
ON_FAILURE = "abort"  # For REAL collisions:  "abort" or "go_home"
GROUP_NAME = "jaka_zu5"
JOINT_ORDER = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

# Configuration validation threshold
MAX_JOINT_JUMP_DEG = 120.0  # Reject IK solutions with >120° joint jumps

# Global emergency stop
EMERGENCY_STOP = False


def read_stl_binary(filename):
    """Read binary STL file"""
    mesh = ShapeMesh()
    vertices, triangles = [], []
    
    with open(filename, 'rb') as f:
        f.read(80)
        num_triangles = struct.unpack('<I', f.read(4))[0]
        
        for _ in range(num_triangles):
            f.read(12)
            triangle = MeshTriangle()
            vertex_idx = []
            
            for i in range(3):
                x, y, z = struct.unpack('<fff', f.read(12))
                vertex = Point()
                vertex.x, vertex.y, vertex.z = float(x), float(y), float(z)
                vertex_idx.append(len(vertices))
                vertices.append(vertex)
            
            triangle.vertex_indices = vertex_idx
            triangles.append(triangle)
            f.read(2)
    
    mesh.vertices = vertices
    mesh. triangles = triangles
    return mesh


class WaypointExecutor(Node):
    def __init__(self):
        super().__init__('waypoint_executor')
        
        self.move_group_client = ActionClient(self, MoveGroup, 'move_action')
        if not self.move_group_client.wait_for_server(timeout_sec=15.0):
            self.move_group_client = ActionClient(self, MoveGroup, 'move_group')
            if not self.move_group_client.wait_for_server(timeout_sec=15.0):
                raise RuntimeError("MoveGroup action not available")
        
        self. trajectory_client = ActionClient(self, FollowJointTrajectory, 
                                             '/jaka_zu5_controller/follow_joint_trajectory')
        self.scene_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.plan_service = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.ik_service = self. create_client(GetPositionIK, '/compute_ik')
        
        self.joint_sub = self.create_subscription(JointState, '/joint_states', 
                                                  self.joint_callback, 10)
        self.current_joints = None
        self. joint_names = JOINT_ORDER
        self.current_goal_handle = None
        
    def joint_callback(self, msg):
        if len(msg.position) >= 6:
            self.current_joints = list(msg.position[:6])
    
    def load_collision_object(self):
        """Load STL collision object"""
        try:
            stl_path = "/home/wakanda_forever/jaka_ws/src/jaka_ros2/src/jaka_description/meshes/jaka_zu5_meshes/Part1m.STL"
            mesh = read_stl_binary(stl_path)
            
            obj = CollisionObject()
            obj.header.frame_id = "Link_0"
            obj.id = "part1m"
            obj.meshes = [mesh]
            
            pose = Pose()
            pose.position.x = -0.4
            pose.position.y = -0.8
            pose.position.z = 0.0
            pose. orientation.w = 1.0
            obj.mesh_poses = [pose]
            obj.operation = CollisionObject.ADD
            
            scene = PlanningScene()
            scene.world. collision_objects = [obj]
            scene.is_diff = True
            
            if self.scene_client.wait_for_service(timeout_sec=2.0):
                req = ApplyPlanningScene.Request()
                req.scene = scene
                future = self.scene_client.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
                print("✅ STL collision object loaded")
                return True
            return False
        except Exception as e: 
            print(f"⚠️  STL load failed: {e}")
            return False
    
    def compute_ik(self, target_pose:  Pose) -> Tuple[Optional[List[float]], str]:
        """
        Compute IK with collision checking AND configuration validation
        
        Returns:
            (joint_values, status) where status is:
            - "success": Valid IK solution
            - "collision": IK failed due to collision
            - "config_flip": IK succeeded but configuration flipped
            - "failed": Other IK failure
        """
        if not self. ik_service.wait_for_service(timeout_sec=2.0):
            return None, "failed"
        
        from moveit_msgs.srv import GetPositionIK
        from geometry_msgs.msg import PoseStamped
        
        req = GetPositionIK. Request()
        req.ik_request.group_name = GROUP_NAME
        req.ik_request.avoid_collisions = True
        
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "world"
        pose_stamped. pose = target_pose
        req.ik_request.pose_stamped = pose_stamped
        
        req.ik_request.robot_state.joint_state.name = self.joint_names
        req.ik_request.robot_state.joint_state.position = self.current_joints if self.current_joints else [0.0] * 6
        
        try:
            future = self.ik_service. call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            response = future.result()
            
            if response.error_code. val == 1:  # SUCCESS
                joint_dict = dict(zip(response.solution.joint_state.name, 
                                    response.solution. joint_state.position))
                target_joints = [joint_dict. get(name, 0.0) for name in self.joint_names]
                
                # ✅ Check for configuration flip
                if not self.validate_configuration(target_joints):
                    return None, "config_flip"
                
                return target_joints, "success"
            
            elif response.error_code.val == -31:  # COLLISION
                return None, "collision"
            
            else:
                return None, "failed"
                
        except: 
            return None, "failed"
    
    def validate_configuration(self, target_joints: List[float]) -> bool:
        """Check if IK solution has acceptable joint displacements"""
        if self.current_joints is None:
            return True  # First waypoint, nothing to compare
        
        threshold_rad = math.radians(MAX_JOINT_JUMP_DEG)
        
        for i, (current, target) in enumerate(zip(self.current_joints, target_joints)):
            delta = target - current
            delta = ((delta + math.pi) % (2 * math.pi)) - math.pi
            delta_abs = abs(delta)
            
            if delta_abs > threshold_rad:
                self.get_logger().warn(
                    f"  ⚠️  Joint {i+1} jump:  {math.degrees(delta_abs):.1f}° "
                    f"(max:  {MAX_JOINT_JUMP_DEG}°)"
                )
                return False
        
        return True
    
    def plan_and_execute_joints(self, target_joints: List[float]) -> bool:
        """Plan and execute to joint target with emergency stop support"""
        global EMERGENCY_STOP
        
        req = MotionPlanRequest()
        req.group_name = GROUP_NAME
        req.num_planning_attempts = 15
        req.allowed_planning_time = 8.0
        req.planner_id = 'RRTConnectkConfigDefault'
        req.max_velocity_scaling_factor = 0.15
        req.max_acceleration_scaling_factor = 0.15
        
        req.start_state.joint_state.name = self.joint_names
        req.start_state. joint_state.position = self.current_joints if self.current_joints else [0.0] * 6
        
        goal_constraint = Constraints()
        for name, value in zip(self.joint_names, target_joints):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = value
            jc.tolerance_above = 0.00001
            jc.tolerance_below = 0.00001
            jc.weight = 1.0
            goal_constraint.joint_constraints.append(jc)
        req.goal_constraints = [goal_constraint]
        
        opts = PlanningOptions()
        opts.plan_only = False
        
        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = opts
        
        try:
            if EMERGENCY_STOP:
                return False
            
            send_future = self.move_group_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
            
            goal_handle = send_future.result()
            if not goal_handle or not goal_handle.accepted:
                return False
            
            self.current_goal_handle = goal_handle
            result_future = goal_handle.get_result_async()
            
            while not result_future.done():
                if EMERGENCY_STOP:
                    self.get_logger().warn("Emergency stop - canceling")
                    cancel_future = goal_handle.cancel_goal_async()
                    rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)
                    return False
                rclpy.spin_once(self, timeout_sec=0.1)
            
            result = result_future.result().result
            self.current_goal_handle = None
            return result. error_code.val == 1
            
        except: 
            self.current_goal_handle = None
            return False
    
    def plan_to_joint_target(self, target_joints: List[float]) -> Optional[JointTrajectory]:
        """Plan motion to joint target"""
        if not self.plan_service.wait_for_service(timeout_sec=2.0):
            return None
        
        from moveit_msgs.srv import GetMotionPlan
        req = GetMotionPlan.Request()
        req.motion_plan_request.workspace_parameters.header.frame_id = "world"
        req.motion_plan_request.workspace_parameters.header. stamp = self.get_clock().now().to_msg()
        
        req.motion_plan_request.start_state.joint_state.header.frame_id = "Link_0"
        req.motion_plan_request.start_state.joint_state.header.stamp = self.get_clock().now().to_msg()
        req.motion_plan_request.start_state.joint_state.name = self.joint_names
        req.motion_plan_request. start_state.joint_state.position = self.current_joints if self.current_joints else [0.0] * 6
        req.motion_plan_request.start_state.is_diff = False
        
        goal_constraint = Constraints()
        for name, value in zip(self.joint_names, target_joints):
            jc = JointConstraint()
            jc.joint_name = name
            jc. position = value
            jc. tolerance_above = 0.00001
            jc.tolerance_below = 0.00001
            jc.weight = 1.0
            goal_constraint. joint_constraints.append(jc)
        
        req.motion_plan_request.goal_constraints = [goal_constraint]
        req.motion_plan_request. group_name = "jaka_arm"
        req.motion_plan_request.num_planning_attempts = 10
        req.motion_plan_request.allowed_planning_time = 10.0
        req.motion_plan_request.max_velocity_scaling_factor = 0.08
        req.motion_plan_request.max_acceleration_scaling_factor = 0.08
        
        try:
            future = self.plan_service.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
            response = future.result()
            
            if response.motion_plan_response.error_code.val == 1:
                return response.motion_plan_response.trajectory. joint_trajectory
            return None
        except:
            return None
    
    def plan_to_pose(self, target_pose: Pose) -> Optional[JointTrajectory]:
        """Plan motion to pose"""
        target_joints, status = self.compute_ik(target_pose)
        if target_joints is None:
            return None
        return self.plan_to_joint_target(target_joints)
    
    def execute_trajectory(self, trajectory: JointTrajectory) -> bool:
        """Execute trajectory"""
        if not self.trajectory_client.wait_for_server(timeout_sec=5.0):
            return False
        
        goal = FollowJointTrajectory. Goal()
        goal.trajectory = trajectory
        
        send_future = self.trajectory_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
        
        goal_handle = send_future.result()
        if not goal_handle. accepted:
            return False
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
        
        return result_future.result().result.error_code == 0


def handle_failure(failure_type: str, mode: str, executor_node) -> bool:
    """
    Handle failure based on type
    
    Args:
        failure_type: "collision", "config_flip", or "failed"
        mode: "abort" or "go_home" (for real collisions)
    
    Returns:
        True to continue, False to stop
    """
    HOME_JOINTS = [math.radians(-90), math.radians(90), math.radians(-90), 
                   math.radians(90), math.radians(90), math.radians(0)]
    
    # ✅ Configuration flip:  Just skip, no movement
    if failure_type == "config_flip": 
        print(f"  ⏭️  SKIPPING (wrong orientation)")
        print(f"      Continuing to next waypoint...")
        return True  # Continue to next
    
    # ✅ Real collision or other failure:  Use configured mode
    if mode == "abort": 
        print(f"\n⚠️  ABORTING (collision detected)")
        print(f"   Robot stopped at current position")
        return False
    
    elif mode == "go_home": 
        print(f"\n🏠 Collision detected - Returning to HOME")
        success = executor_node.plan_and_execute_joints(HOME_JOINTS)
        if success: 
            print(f"  ✅ Robot at HOME position")
        else:
            print(f"  ❌ Failed to reach HOME")
        return False
    
    return False


def signal_handler(sig, frame):
    """Handle Ctrl+C"""
    global EMERGENCY_STOP
    EMERGENCY_STOP = True
    print("\n\n" + "="*70)
    print("🛑 EMERGENCY STOP (Ctrl+C)")
    print("="*70 + "\n")


def main():
    global EMERGENCY_STOP
    
    signal.signal(signal.SIGINT, signal_handler)
    rclpy.init()
    executor_node = WaypointExecutor()
    
    print()
    print("="*70)
    print("⚠️  EMERGENCY STOP:  Press Ctrl+C to stop immediately")
    print("="*70)
    print()
    
    # Wait for joint states
    print("Waiting for robot state...")
    timeout = 0
    while executor_node.current_joints is None and rclpy.ok() and not EMERGENCY_STOP:
        rclpy.spin_once(executor_node, timeout_sec=0.1)
        timeout += 1
        if timeout > 50:
            print("❌ Timeout - robot not connected? ")
            executor_node.destroy_node()
            rclpy.shutdown()
            return
    
    if EMERGENCY_STOP:
        print("🛑 Emergency stop during startup")
        executor_node.destroy_node()
        rclpy.shutdown()
        return
    
    print(f"Current joints: {[f'{j:.2f}' for j in executor_node.current_joints]}")
    
    # Load waypoints
    try:
        with open('waypoints.json', 'r') as f:
            waypoints = json.load(f)
        print(f"\n✅ Loaded {len(waypoints)} waypoints\n")
    except Exception as e:
        print(f"❌ Failed to load waypoints. json: {e}")
        executor_node.destroy_node()
        rclpy.shutdown()
        return
    
    # Load collision
    executor_node.load_collision_object()
    time.sleep(1)
    
    print("="*70)
    print(f"WAYPOINT EXECUTION")
    print(f"Collision mode: {ON_FAILURE. upper()}")
    print(f"Config validation:  ENABLED (max jump:  {MAX_JOINT_JUMP_DEG}°)")
    print("="*70)
    
    successful = 0
    skipped = 0
    failed = 0
    
    for i, wp in enumerate(waypoints, 1):
        if EMERGENCY_STOP:
            print("\n🛑 EMERGENCY STOP")
            break
        
        print(f"\n[{i}/{len(waypoints)}] Waypoint {i}")
        print(f"  Position: [{wp['position']['x']:.3f}, {wp['position']['y']:.3f}, {wp['position']['z']:.3f}]")
        
        target_pose = Pose()
        target_pose.position. x = wp['position']['x']
        target_pose.position. y = wp['position']['y']
        target_pose.position. z = wp['position']['z']
        target_pose.orientation. x = wp['orientation_quaternion']['x']
        target_pose.orientation.y = wp['orientation_quaternion']['y']
        target_pose.orientation.z = wp['orientation_quaternion']['z']
        target_pose. orientation.w = wp['orientation_quaternion']['w']
        
        print("  🔄 Computing IK...")
        target_joints, ik_status = executor_node. compute_ik(target_pose)
        
        # ✅ Handle different failure types
        if target_joints is None:
            if ik_status == "config_flip":
                print(f"  ⚠️  Configuration flip detected")
                skipped += 1
                if not handle_failure("config_flip", ON_FAILURE, executor_node):
                    break
                continue  # Skip to next waypoint
            
            elif ik_status == "collision":
                print(f"  ❌ IK FAILED (COLLISION)")
                failed += 1
                if not handle_failure("collision", ON_FAILURE, executor_node):
                    break
                continue
            
            else:
                print(f"  ❌ IK FAILED")
                failed += 1
                if not handle_failure("failed", ON_FAILURE, executor_node):
                    break
                continue
        
        print(f"  ✅ IK:  [{', '.join([f'{j:.3f}' for j in target_joints])}]")
        
        if EMERGENCY_STOP:
            print("\n🛑 EMERGENCY STOP")
            break
        
        print("  🔄 Planning and executing...")
        success = executor_node.plan_and_execute_joints(target_joints)
        
        if EMERGENCY_STOP:
            print("\n🛑 EMERGENCY STOP")
            break
        
        if success:
            print(f"  ✅ Waypoint {i} completed!")
            successful += 1
            time.sleep(0.5)
            rclpy.spin_once(executor_node, timeout_sec=0.5)
        else:
            print(f"  ❌ Execution failed")
            failed += 1
            if not handle_failure("failed", ON_FAILURE, executor_node):
                break
    
    # Summary
    print("\n" + "="*70)
    print("EXECUTION SUMMARY")
    print("="*70)
    print(f"✅ Successful:   {successful}/{len(waypoints)}")
    print(f"⏭️  Skipped:     {skipped}/{len(waypoints)} (wrong orientation)")
    print(f"❌ Failed:      {failed}/{len(waypoints)} (collision/unreachable)")
    if EMERGENCY_STOP:
        print(f"🛑 Emergency stopped")
    print("="*70 + "\n")
    
    executor_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()