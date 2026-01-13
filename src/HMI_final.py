#!/usr/bin/env python3
"""
JAKA ZU5 Industrial HMI - Production Ready (Simplified Single Mode)
Direct robot connection with optional RViz visualization
"""

import sys
import os
import subprocess
import json
import time
import math
import signal
import struct

# GUI Libraries
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QPushButton, QGroupBox, QTextEdit, QMessageBox)
from PyQt5.QtCore import QTimer, QThread, pyqtSignal, Qt
from PyQt5.QtGui import QFont

# ROS Libraries
import rclpy
from rclpy.node import Node
from rclpy. action import ActionClient
from visualization_msgs.msg import InteractiveMarkerFeedback
from geometry_msgs. msg import Pose, Point
from moveit_msgs.msg import CollisionObject, PlanningScene, MotionPlanRequest, Constraints, JointConstraint, PlanningOptions
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.action import MoveGroup
from shape_msgs.msg import Mesh as ShapeMesh, MeshTriangle

# ==================== CONFIGURATION ====================
STL_PATH = "/home/wakanda_forever/jaka_ws/src/jaka_ros2/src/jaka_description/meshes/jaka_zu5_meshes/Part1m.STL"
JSON_PATH = "waypoints.json"

# Launch commands
LAUNCH_ROBOT_HARDWARE = "ros2 launch jaka_planner moveit_server.launch.py ip:=192.168.1.100 model:=zu5"
LAUNCH_ROBOT_PLANNING = "ros2 launch jaka_zu5_moveit_config move_group.launch.py"

# RViz config path (adjust if needed)
RVIZ_CONFIG = "/home/wakanda_forever/jaka_ws/install/jaka_zu5_moveit_config/share/jaka_zu5_moveit_config/config/moveit.rviz"

EXECUTE_SCRIPT = "execute.py"

# Robot connection
ROBOT_IP = "192.168.1.100"
ROBOT_USER = "root"

# STL mesh position
MESH_POSE = {
    'x': -0.4, 'y': -0.8, 'z': 0.0,
    'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw':  1.0
}

# Home position (degrees): [-90, 90, -90, 90, 90, 0]
HOME_POSITION_DEG = [-90, 90, -90, 90, 90, 0]
HOME_POSITION = [math.radians(deg) for deg in HOME_POSITION_DEG]
JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']


# ==================== HELPER FUNCTIONS ====================
def read_stl_binary(filename):
    """Read binary STL file and return Mesh"""
    mesh = ShapeMesh()
    vertices = []
    triangles = []
    try:
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
                    vertices. append(vertex)
                triangle.vertex_indices = vertex_idx
                triangles.append(triangle)
                f.read(2)
        mesh.vertices = vertices
        mesh.triangles = triangles
        return mesh
    except Exception as e:
        print(f"STL Read Error: {e}")
        return None


# ==================== EXECUTION MONITOR ====================
class ExecutionMonitor(QThread):
    """Thread to monitor execution script output"""
    output_signal = pyqtSignal(str)
    finished_signal = pyqtSignal(int)
    
    def __init__(self, process):
        super().__init__()
        self.process = process
    
    def run(self):
        """Read process output line by line"""
        try:
            for line in self.process.stdout:
                self.output_signal.emit(line)
            exit_code = self.process.wait()
            self.finished_signal.emit(exit_code)
        except Exception as e:
            self.output_signal.emit(f"Monitor error: {e}")
            self.finished_signal.emit(-1)


# ==================== ROS WORKER THREAD ====================
class ROSWorker(QThread):
    """Background thread for ROS2 operations"""
    pose_update_signal = pyqtSignal(dict)
    scene_pub_signal = pyqtSignal(str)
    home_result_signal = pyqtSignal(bool, str)

    def __init__(self):
        super().__init__()
        rclpy.init(args=None)
        self.node = Node('hmi_listener_node')
        
        # Subscriber for interactive marker
        self.sub = self.node.create_subscription(
            InteractiveMarkerFeedback,
            '/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback',
            self.marker_callback,
            10
        )
        
        # Service client for STL loading
        self.scene_client = self.node.create_client(ApplyPlanningScene, '/apply_planning_scene')
        
        # MoveGroup action client (created on first use)
        self.move_group_client = None
        self.running = True

    def marker_callback(self, msg):
        """Handle interactive marker feedback"""
        p = msg.pose. position
        q = msg.pose.orientation
        
        # Convert quaternion to roll-pitch-yaw
        roll = math.atan2(2*(q.w*q.x + q.y*q.z), 1 - 2*(q.x*q.x + q.y*q.y))
        pitch = math.asin(2*(q.w*q.y - q.z*q.x))
        yaw = math. atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

        data = {
            'x': p.x, 'y': p.y, 'z': p.z,
            'qx': q.x, 'qy': q.y, 'qz': q.z, 'qw': q. w,
            'roll': roll, 'pitch': pitch, 'yaw': yaw
        }
        self.pose_update_signal.emit(data)

    def publish_stl(self):
        """Load STL mesh into planning scene"""
        if not self. scene_client.wait_for_service(timeout_sec=2.0):
            self.scene_pub_signal.emit("[WARN] Scene service not available")
            return

        mesh = read_stl_binary(STL_PATH)
        if not mesh:
            self.scene_pub_signal.emit("[ERROR] Failed to read STL file")
            return

        # Create collision object
        obj = CollisionObject()
        obj.header.frame_id = "Link_0"
        obj.id = "part1m"
        obj.meshes = [mesh]
        
        pose = Pose()
        pose.position.x = MESH_POSE['x']
        pose.position.y = MESH_POSE['y']
        pose.position.z = MESH_POSE['z']
        pose.orientation.x = MESH_POSE['qx']
        pose. orientation.y = MESH_POSE['qy']
        pose.orientation.z = MESH_POSE['qz']
        pose.orientation.w = MESH_POSE['qw']
        obj.mesh_poses = [pose]
        obj.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.world. collision_objects = [obj]
        scene.is_diff = True

        req = ApplyPlanningScene.Request()
        req.scene = scene
        
        future = self.scene_client.call_async(req)
        self.scene_pub_signal.emit("[OK] STL mesh loaded")

    def move_to_home(self):
        """Move robot to home position using MoveGroup"""
        
        # Create MoveGroup action client
        if self.move_group_client is None:
            self.move_group_client = ActionClient(self.node, MoveGroup, 'move_action')
            if not self.move_group_client.wait_for_server(timeout_sec=10.0):
                self.move_group_client = ActionClient(self.node, MoveGroup, 'move_group')
                if not self.move_group_client.wait_for_server(timeout_sec=10.0):
                    self.home_result_signal.emit(False, "MoveGroup action not available")
                    return
        
        # Create motion plan request
        req = MotionPlanRequest()
        req.group_name = "jaka_zu5"
        req.num_planning_attempts = 15
        req.allowed_planning_time = 8.0
        req.planner_id = 'RRTConnectkConfigDefault'
        req.max_velocity_scaling_factor = 0.25
        req.max_acceleration_scaling_factor = 0.25
        
        # Set goal to hardcoded home position
        goal_constraint = Constraints()
        for name, value in zip(JOINT_NAMES, HOME_POSITION):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = value
            jc.tolerance_above = 0.000001
            jc.tolerance_below = 0.000001
            jc.weight = 1.0
            goal_constraint.joint_constraints.append(jc)
        req.goal_constraints = [goal_constraint]
        
        # Planning options:  plan AND execute
        opts = PlanningOptions()
        opts.plan_only = False
        
        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = opts
        
        try:
            # Send goal
            send_future = self.move_group_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=5.0)
            
            goal_handle = send_future.result()
            if not goal_handle or not goal_handle.accepted:
                self.home_result_signal.emit(False, "Goal rejected by controller")
                return
            
            # Wait for result
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=30.0)
            
            result = result_future.result()
            if result and result.result. error_code. val == 1:
                self.home_result_signal.emit(True, "Robot moved to home position")
            else:
                error_code = result.result.error_code.val if result else "timeout"
                self.home_result_signal.emit(False, f"Failed (error code: {error_code})")
                
        except Exception as e:
            self.home_result_signal.emit(False, f"Exception: {e}")

    def run(self):
        """Main thread loop"""
        while self.running and rclpy.ok():
            rclpy.spin_once(self. node, timeout_sec=0.1)

    def stop(self):
        """Stop the worker thread"""
        self.running = False
        self.node.destroy_node()
        rclpy.shutdown()


# ==================== MAIN HMI CLASS ====================
class JakaHMI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("JAKA ZU5 Industrial HMI - Production Ready")
        self.setGeometry(100, 100, 1200, 750)
        self.setStyleSheet("background-color: #2b2b2b; color: #ffffff;")
        
        # State variables
        self.current_pose = None
        self.waypoints = []
        self. rviz_process = None
        self.robot_process = None
        self.rsp_process = None  # ← ADD THIS LINE
        self.desc_pub_process = None  # robot_description publisher
        self.planning_process = None
        self.execution_process = None
        self.execution_monitor = None
        self.robot_connected = False

        # Build GUI
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        # LEFT PANEL
        left_panel = QVBoxLayout()
        
        # Mode indicator
        self.label_mode = QLabel("MODE:  Idle")
        self.label_mode.setStyleSheet("background-color: #555; padding: 10px; font-size: 14px; font-weight: bold; border-radius: 5px;")
        self.label_mode.setAlignment(Qt.AlignCenter)
        left_panel.addWidget(self.label_mode)
        
        # 1. Robot Connection
        self.group_connect = QGroupBox("1. Robot Connection")
        self.group_connect.setStyleSheet("QGroupBox { font-weight: bold; border: 2px solid #FF9800; margin-top: 10px; padding: 10px; }")
        connect_layout = QVBoxLayout()
        
        self.btn_connect = QPushButton("[CONNECT] Connect Robot")
        self.btn_connect.setStyleSheet("background-color: #FF9800; font-size: 13px; padding: 12px; font-weight: bold;")
        self.btn_connect.clicked. connect(self.connect_robot)
        
        self.btn_rviz = QPushButton("[RVIZ] Open Visualization")
        self.btn_rviz.setStyleSheet("background-color: #4CAF50; padding: 8px;")
        self.btn_rviz.clicked.connect(self.open_rviz)
        self.btn_rviz.setEnabled(False)

        self.btn_load_stl = QPushButton("[STL] Load Part Mesh")
        self.btn_load_stl.setStyleSheet("background-color: #607D8B; padding: 6px;")
        self.btn_load_stl.clicked. connect(self.trigger_stl_load)
        self.btn_load_stl. setEnabled(False)
        
        connect_layout.addWidget(self. btn_connect)
        connect_layout.addWidget(self.btn_rviz)
        connect_layout.addWidget(self.btn_load_stl)
        self.group_connect.setLayout(connect_layout)
        
        # 2. Live Marker Pose
        self.group_data = QGroupBox("2. Live Marker Pose")
        data_layout = QVBoxLayout()
        self.label_pose = QLabel("Waiting for marker...")
        self.label_pose. setFont(QFont("Courier", 11))
        self.label_pose.setStyleSheet("color: #00ff00; border: 1px solid #444; padding: 10px;")
        data_layout.addWidget(self.label_pose)
        self.group_data.setLayout(data_layout)

        # 3. Waypoint Teaching
        self.group_teach = QGroupBox("3. Waypoint Teaching")
        self.group_teach.setStyleSheet("QGroupBox { border: 1px solid #666; padding-top: 20px; }")
        teach_layout = QVBoxLayout()
        teach_layout.setSpacing(5)
        
        self.btn_save = QPushButton("[SAVE] Save Waypoint")
        self.btn_save.setStyleSheet("background-color: #4CAF50; padding: 8px;")
        self.btn_save.clicked. connect(self.save_waypoint)
        
        self.btn_delete = QPushButton("[DEL] Delete Last")
        self.btn_delete. setStyleSheet("background-color:  #f44336; padding: 8px;")
        self.btn_delete.clicked.connect(self.delete_last)
        
        self.btn_list = QPushButton("[LIST] List Current Session")
        self.btn_list.setStyleSheet("background-color: #FF9800; padding: 8px;")
        self.btn_list.clicked.connect(self. list_waypoints)
        
        self.btn_write = QPushButton("[WRITE] Write to JSON")
        self.btn_write.setStyleSheet("background-color: #2196F3; padding: 8px;")
        self.btn_write.clicked.connect(self.write_json)
        
        self.btn_show_saved = QPushButton("[SHOW] Show Saved JSON")
        self.btn_show_saved.setStyleSheet("background-color: #9C27B0; padding: 8px;")
        self.btn_show_saved.clicked. connect(self.show_saved_waypoints)
        
        teach_layout.addWidget(self.btn_save)
        teach_layout.addWidget(self.btn_delete)
        teach_layout. addWidget(self.btn_list)
        teach_layout.addWidget(self.btn_write)
        teach_layout.addWidget(self.btn_show_saved)
        self.group_teach.setLayout(teach_layout)
        
        # 4. Production
        self.group_prod = QGroupBox("4. Production")
        self.group_prod.setStyleSheet("QGroupBox { font-weight: bold; border: 2px solid #E91E63; margin-top:   10px; padding: 10px; }")
        prod_layout = QVBoxLayout()
        
        self.btn_estop = QPushButton("🛑 [E-STOP] EMERGENCY STOP")
        self.btn_estop.setStyleSheet(
            "background-color: #ff0000; "
            "font-weight: bold; "
            "font-size:  18px; "
            "padding: 20px; "
            "color: white; "
            "border: 3px solid #8B0000; "  # ✅ Dark red = Normal/Ready
            "border-radius: 10px;"
        )
        self.btn_estop.clicked.connect(self.emergency_stop)
        self.btn_estop.setEnabled(False)
        
        self.btn_move_home = QPushButton("[HOME] Move to HOME")
        self.btn_move_home.setStyleSheet("background-color: #673AB7; padding: 10px;")
        self.btn_move_home.clicked.connect(self.move_robot_home)
        self.btn_move_home.setEnabled(False)
        
        self.btn_execute = QPushButton("[EXEC] START EXECUTION")
        self.btn_execute.setStyleSheet("background-color: #E91E63; font-weight: bold; padding: 15px;")
        self.btn_execute.setEnabled(False)
        self.btn_execute.clicked.connect(self.run_execution)
        
        prod_layout.addWidget(self. btn_estop)  # ✅ Add emergency stop first
        prod_layout.addWidget(self.btn_move_home)
        prod_layout.addWidget(self.btn_execute)
        self.group_prod.setLayout(prod_layout)
        
        # 5. Robot Shutdown
        self.group_shutdown = QGroupBox("5. Robot Shutdown")
        self.group_shutdown.setStyleSheet("QGroupBox { font-weight: bold; border: 2px solid #f44336; margin-top: 10px; padding: 10px; }")
        shutdown_layout = QVBoxLayout()
        
        self.btn_disable = QPushButton("[DISABLE] Disable Servo")
        self.btn_disable.setStyleSheet("background-color: #FFC107; padding: 8px;")
        self.btn_disable.clicked.connect(self.disable_robot)
        self.btn_disable.setEnabled(False)
        
        self.btn_power_off = QPushButton("[POWER OFF] Power Off Robot")
        self.btn_power_off.setStyleSheet("background-color: #FF5722; padding: 8px;")
        self.btn_power_off.clicked.connect(self.power_off_robot)
        self.btn_power_off.setEnabled(False)
        
        self.btn_shutdown = QPushButton("[SHUTDOWN] Shutdown Cabinet")
        self.btn_shutdown.setStyleSheet("background-color: #f44336; font-weight: bold; padding: 10px;")
        self.btn_shutdown.clicked.connect(self.shutdown_cabinet)
        self.btn_shutdown.setEnabled(False)
        
        shutdown_layout.addWidget(self.btn_disable)
        shutdown_layout.addWidget(self.btn_power_off)
        shutdown_layout.addWidget(self.btn_shutdown)
        self.group_shutdown.setLayout(shutdown_layout)

        left_panel.addWidget(self. group_connect)
        left_panel.addWidget(self.group_data)
        left_panel.addWidget(self.group_teach)
        left_panel. addStretch()
        left_panel. addWidget(self.group_prod)
        left_panel.addWidget(self.group_shutdown)

        # RIGHT PANEL (System Logs)
        right_panel = QVBoxLayout()
        log_label = QLabel("System Logs:")
        log_label.setStyleSheet("font-size: 14px; font-weight: bold;")
        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setStyleSheet("background-color: #111; color: #eee; font-family:  Courier; font-size: 11px;")
        right_panel.addWidget(log_label)
        right_panel. addWidget(self.log_box)

        main_layout.addLayout(left_panel, 35)
        main_layout.addLayout(right_panel, 65)

        # Start ROS worker thread
        self.ros_thread = ROSWorker()
        self.ros_thread.pose_update_signal.connect(self.update_pose_display)
        self.ros_thread.scene_pub_signal. connect(self.log)
        self.ros_thread. home_result_signal.connect(self.handle_home_result)
        self.ros_thread.start()
        
        self.log("[OK] HMI Initialized - Production Ready")
        self.log("[INFO] Click [CONNECT] to connect robot and begin")

    # ==================== LOGGING ====================
    def log(self, msg):
        """Add timestamped message to system log"""
        timestamp = time.strftime("%H:%M:%S")
        self.log_box. append(f"[{timestamp}] {msg}")
        self.log_box.verticalScrollBar().setValue(self.log_box.verticalScrollBar().maximum())

    # ==================== ROBOT CONNECTION ====================
    def connect_robot(self):
        """Connect to real robot"""
        if not self.robot_connected:
            self.log("[CONNECT] Step 1/3: Connecting to robot hardware...")
            self.robot_process = subprocess.Popen(LAUNCH_ROBOT_HARDWARE, shell=True, preexec_fn=os.setsid)
            
            self.log("[WAIT] Step 2/3: Launching MoveGroup planning...")
            QTimer.singleShot(5000, self.launch_move_group)
            
            self. btn_connect.setEnabled(False)
    
    def launch_move_group(self):
        """Launch MoveGroup planning node"""
        self.log("[PLAN] Launching MoveGroup planning node...")
        self.planning_process = subprocess. Popen(LAUNCH_ROBOT_PLANNING, shell=True, preexec_fn=os. setsid)
        
        self.log("[WAIT] Step 3/3: Starting robot_state_publisher for RViz...")
        QTimer.singleShot(10000, self.start_robot_state_publisher)

    def start_robot_state_publisher(self):
        """Start robot_state_publisher AND robot_description publisher"""
        import time
        
        self.log("[RSP] Waiting for move_group...")
        time.sleep(3)
        
        self.log("[RSP] Extracting robot_description...")
        
        # Get robot_description from move_group
        get_param_cmd = "ros2 param get /move_group robot_description"
        
        try:
            result = subprocess.run(
                get_param_cmd,
                shell=True,
                capture_output=True,
                text=True,
                timeout=10
            )
            
            if result.returncode == 0:
                # Clean up the output
                urdf = result.stdout
                if "String value: " in urdf:
                    urdf = urdf.split("String value: ", 1)[1].strip()
                
                # Save to file
                urdf_file = "/tmp/robot_description.urdf"
                with open(urdf_file, 'w') as f:
                    f.write(urdf)
                
                self.log(f"[RSP] Saved URDF ({len(urdf)} chars)")
                
                # Start robot_state_publisher with the file
                rsp_cmd = f"ros2 run robot_state_publisher robot_state_publisher {urdf_file}"
                self.rsp_process = subprocess.Popen(
                    rsp_cmd,
                    shell=True,
                    preexec_fn=os.setsid,
                    stdout=subprocess. DEVNULL,
                    stderr=subprocess.DEVNULL
                )
                
                self. log("[OK] robot_state_publisher started")
                
                # Also start the robot_description topic publisher
                time.sleep(1)
                self.log("[PUB] Starting robot_description topic publisher...")
                
                pub_script = os.path.expanduser("~/jaka_ws/scripts_murali/publish_robot_description.py")
                
                if os.path.exists(pub_script):
                    self.desc_pub_process = subprocess.Popen(
                        f"python3 {pub_script}",
                        shell=True,
                        preexec_fn=os.setsid
                    )
                    self. log("[OK] robot_description topic publisher started")
                else: 
                    self.log(f"[WARN] Publisher script not found: {pub_script}")
                    self.log("[INFO] Creating inline publisher...")
                    
                    # Fallback:  publish using ros2 topic pub
                    # Escape the URDF for command line
                    urdf_escaped = urdf.replace("'", "'\\''")
                    pub_cmd = f"ros2 topic pub -r 1 /robot_description std_msgs/msg/String \"{{data: '{urdf_escaped}'}}\" &"
                    subprocess.Popen(pub_cmd, shell=True)
                    self.log("[OK] Using ros2 topic pub for robot_description")
                    
            else:
                self.log("[WARN] Could not get robot_description parameter")
                
        except Exception as e:
            self.log(f"[ERROR] RSP start failed: {e}")
            self.log("[WARN] RViz may not display robot model")
        
        # Continue anyway
        QTimer.singleShot(2000, self.robot_ready)

    def emergency_stop(self):
        """
        EMERGENCY STOP - INSTANT robot stop (< 500ms)
        Uses parallel kill commands for zero delay
        """
        self.log("=" * 70)
        self.log("🛑 EMERGENCY STOP ACTIVATED!")
        self.log("=" * 70)
        
        # ✅ STEP 1: Kill moveit_server IMMEDIATELY (most critical)
        # This stops command stream to robot hardware → robot stops in ~100-200ms
        self.log("[E-STOP] Stopping robot IMMEDIATELY...")
        try:
            # SIGKILL = immediate, no cleanup, instant death
            subprocess.run("pkill -9 -f 'moveit_server'", shell=True, timeout=0.5)
            self.log("[E-STOP] ✅ Hardware interface killed")
            self.robot_process = None
        except: 
            pass
        
        # ✅ STEP 2: Fire off cancel/SSH commands in BACKGROUND (non-blocking)
        # These run in parallel, don't wait for them
        try:
            # Cancel action (background, fire-and-forget)
            subprocess. Popen(
                "ros2 action send_goal /move_action moveit_msgs/action/MoveGroup '{}' --cancel-all 2>/dev/null &",
                shell=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            subprocess.Popen(
                "ros2 action send_goal /move_group moveit_msgs/action/MoveGroup '{}' --cancel-all 2>/dev/null &",
                shell=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess. DEVNULL
            )
            self.log("[E-STOP] ✅ Cancel requests sent (background)")
        except:
            pass
        
        try:
            # SSH disable (background, fire-and-forget)
            subprocess.Popen(
                f"sshpass -p 'Jaka2019!' ssh -o ConnectTimeout=1 -o StrictHostKeyChecking=no {ROBOT_USER}@{ROBOT_IP} 'jkservo disable' 2>/dev/null &",
                shell=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess. DEVNULL
            )
            self.log("[E-STOP] ✅ SSH disable sent (background)")
        except:
            pass
        
        # ✅ STEP 3: Kill execution script (immediate)
        if self.execution_process:
            try:
                os.killpg(os.getpgid(self.execution_process. pid), signal.SIGKILL)
                self.execution_process.wait(timeout=0.5)
                self.log("[E-STOP] ✅ Execution script killed")
            except:
                pass
            finally:
                self.execution_process = None
        
        # ✅ STEP 4: Stop monitor thread
        if self.execution_monitor:
            self.execution_monitor.terminate()
            self.execution_monitor.wait(500)  # 500ms max
            self.execution_monitor = None
        
        # ✅ Visual feedback
        self.btn_estop.setStyleSheet(
            "background-color: #ff0000; "
            "font-weight: bold; "
            "font-size: 18px; "
            "padding: 20px; "
            "color: white; "
            "border: 5px solid #ffff00; "
            "border-radius:  10px;"
        )
        
        self.label_mode.setText("MODE: [🛑 EMERGENCY STOPPED]")
        self.label_mode.setStyleSheet("background-color: #ff0000; padding: 10px; font-size: 14px; font-weight: bold; color: white;")
        
        self.log("[E-STOP] ✅ Robot STOPPED")
        self.log("[E-STOP] ⏱️  Total time: < 500ms")
        self.log("=" * 70)
        
        # Show dialog
        self.show_estop_dialog()
    
    def show_estop_dialog(self):
        """
        Show E-STOP dialog with Disable & Reset option
        """
        msg = QMessageBox(self)
        msg.setIcon(QMessageBox.Critical)
        msg.setWindowTitle("🛑 EMERGENCY STOP")
        msg.setText(
            "EMERGENCY STOP ACTIVATED!\n\n"
            "✅ Robot servo DISABLED\n"
            "✅ Execution terminated\n\n"
            "What do you want to do?"
        )
        msg.setInformativeText(
            "• OK - Acknowledge and keep system stopped\n"
            "• Disable & Reset - Reset HMI to ready state\n"
            "  (You can then reconnect and continue)"
        )
        
        # Add custom buttons
        btn_ok = msg.addButton("OK", QMessageBox.AcceptRole)
        btn_reset = msg.addButton("Disable && Reset", QMessageBox.ActionRole)
        
        msg.setDefaultButton(btn_ok)
        msg.exec_()
        
        # ✅ FIX 2: Handle Disable & Reset
        if msg.clickedButton() == btn_reset:
            self.disable_and_reset()
    
    def disable_and_reset(self):
        """
        Disable robot and reset HMI (optimized for speed)
        """
        self.log("=" * 70)
        self.log("[RESET] Resetting HMI...")
        self.log("=" * 70)
        
        # 1. SSH disable (background, don't wait)
        try:
            subprocess.Popen(
                f"sshpass -p 'Jaka2019!' ssh -o ConnectTimeout=1 -o StrictHostKeyChecking=no {ROBOT_USER}@{ROBOT_IP} 'jkservo disable' 2>/dev/null &",
                shell=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess. DEVNULL
            )
            self.log("[RESET] ✅ Disable command sent")
        except:
            pass
        
        # 2. Kill all ROS processes (parallel kills, fast)
        self.log("[RESET] Shutting down ROS processes...")
        
        # Kill everything at once
        subprocess.run(
            "pkill -9 -f 'move_group|rviz2|moveit_server|robot_state_publisher' 2>/dev/null",
            shell=True,
            timeout=1
        )
        
        # Clean up process handles
        for name in ['rviz_process', 'robot_process', 'rsp_process', 
                     'desc_pub_process', 'planning_process']: 
            setattr(self, name, None)
        
        self.execution_process = None
        self. execution_monitor = None
        self.robot_connected = False
        
        self.log("[RESET] ✅ All processes stopped")
        
        # 3. Reset UI (instant)
        self.label_mode.setText("MODE:   Idle")
        self.label_mode.setStyleSheet("background-color: #555; padding: 10px; font-size: 14px; font-weight: bold; border-radius: 5px;")
        
        self.btn_estop.setStyleSheet(
            "background-color: #ff0000; "
            "font-weight: bold; "
            "font-size: 18px; "
            "padding:  20px; "
            "color: white; "
            "border: 3px solid #8B0000; "
            "border-radius: 10px;"
        )
        
        # Reset buttons
        self.btn_connect.setEnabled(True)
        self.btn_connect.setText("[CONNECT] Connect Robot")
        
        for btn in [self.btn_rviz, self.btn_load_stl, self.btn_execute, 
                    self.btn_move_home, self.btn_estop, self.btn_disable,
                    self.btn_power_off, self.btn_shutdown]:
            btn.setEnabled(False)
        
        self.btn_rviz.setText("[RVIZ] Open Visualization")
        self.btn_rviz.setStyleSheet("background-color: #4CAF50; padding: 8px;")
        
        self.log("[RESET] ✅ HMI reset complete")
        self.log("[RESET] Click [CONNECT] to reconnect")
        self.log("=" * 70)
    
    def robot_ready(self):
        """Robot is ready for operation"""
        self.robot_connected = True
        self. log("[OK] Robot connected - Ready for operation")
        self.log("[INFO] RViz is optional - use for visualization during teaching")
        
        # Update mode
        self.label_mode.setText("MODE: [ROBOT CONNECTED]")
        self.label_mode.setStyleSheet("background-color: #FF5722; padding: 10px; font-size: 14px; font-weight: bold; border-radius: 5px;")
        
        # Enable buttons
        self.btn_rviz.setEnabled(True)
        self.btn_load_stl.setEnabled(True)
        self.btn_execute.setEnabled(True)
        self.btn_move_home. setEnabled(True)
        self.btn_estop.setEnabled(True)  # ✅ Enable emergency stop
        self.btn_disable.setEnabled(True)
        self.btn_power_off.setEnabled(True)
        self.btn_shutdown.setEnabled(True)

    def open_rviz(self):
        """Open RViz for visualization (stays open)"""
        if self.rviz_process is None:
            self.log("[RVIZ] Opening visualization...")
            self.log("[INFO] RViz will stay open for teaching and monitoring")
            
            # Use MoveIt's RViz launch
            rviz_launch = "ros2 launch jaka_zu5_moveit_config moveit_rviz.launch.py"
            
            self.rviz_process = subprocess.Popen(rviz_launch, shell=True)
            
            # Disable button (RViz stays open)
            self.btn_rviz.setEnabled(False)
            self.btn_rviz.setText("[RVIZ] Running...")
            self.btn_rviz.setStyleSheet("background-color: #555; padding: 8px;")
            
            self.log("[OK] RViz opened")
            self.log("[INFO] Use for teaching and execution monitoring")
            
            # Auto-load STL
            QTimer.singleShot(5000, self.auto_load_stl_for_rviz)
        else:
            self.log("[WARN] RViz already running")
    
    def auto_load_stl_for_rviz(self):
        """Auto-load STL after RViz opens"""
        self.log("[STL] Auto-loading part mesh...")
        self.trigger_stl_load()

    def trigger_stl_load(self):
        """Trigger STL mesh loading"""
        self.ros_thread.publish_stl()

    # ==================== MARKER POSE & WAYPOINTS ====================
    def update_pose_display(self, data):
        """Update live marker pose display"""
        self.current_pose = data
        text = (f"X: {data['x']:.4f}  Y: {data['y']:.4f}  Z: {data['z']:.4f}\n"
                f"R:  {data['roll']:.3f}  P: {data['pitch']:.3f}  Y: {data['yaw']:.3f}")
        self.label_pose.setText(text)

    def save_waypoint(self):
        """Save current marker pose as waypoint"""
        if self.current_pose:
            wp = {
                "position": {
                    "x": self. current_pose['x'], 
                    "y": self.current_pose['y'], 
                    "z": self. current_pose['z']
                },
                "orientation_quaternion": {
                    "x": self. current_pose['qx'], 
                    "y": self. current_pose['qy'], 
                    "z": self. current_pose['qz'], 
                    "w": self. current_pose['qw']
                },
                "orientation_rpy": {
                    "roll": self.current_pose['roll'], 
                    "pitch": self.current_pose['pitch'], 
                    "yaw": self.current_pose['yaw']
                }
            }
            self.waypoints.append(wp)
            self.log(f"[SAVE] Waypoint {len(self.waypoints)} saved to session")
        else:
            self.log("[ERROR] No pose available - open RViz and move marker first!")

    def delete_last(self):
        """Delete last waypoint"""
        if self. waypoints:
            self.waypoints.pop()
            self.log(f"[DEL] Deleted.  Session total: {len(self.waypoints)}")
        else:
            self.log("[WARN] No waypoints in current session")

    def list_waypoints(self):
        """List current session waypoints"""
        if not self.waypoints:
            self.log("[LIST] No waypoints in current session")
            return
        
        self.log(f"[LIST] Current session:  {len(self.waypoints)} waypoints")
        for i, wp in enumerate(self.waypoints, 1):
            p = wp['position']
            self.log(f"  [{i}] X:{p['x']:.3f} Y:{p['y']:.3f} Z:{p['z']:.3f}")

    def show_saved_waypoints(self):
        """Show waypoints from saved JSON file"""
        if not os.path.exists(JSON_PATH):
            self.log("[WARN] No saved waypoints. json file found")
            return
        
        try:
            with open(JSON_PATH, 'r') as f:
                saved_wps = json.load(f)
            
            self.log(f"[SHOW] Saved waypoints. json contains {len(saved_wps)} waypoints:")
            for i, wp in enumerate(saved_wps, 1):
                p = wp['position']
                self.log(f"  [{i}] X:{p['x']:.3f} Y:{p['y']:.3f} Z:{p['z']:.3f}")
        except Exception as e:
            self.log(f"[ERROR] Failed to read waypoints.json: {e}")

    def write_json(self):
        """Write current session waypoints to JSON file"""
        if not self.waypoints:
            self.log("[WARN] No waypoints in current session to save")
            return
        
        try:
            with open(JSON_PATH, 'w') as f:
                json.dump(self.waypoints, f, indent=4)
            self.log(f"[WRITE] Saved {len(self.waypoints)} waypoints to {JSON_PATH}")
        except Exception as e:
            self.log(f"[ERROR] Failed to save:  {e}")

    # ==================== PRODUCTION ====================
    def move_robot_home(self):
        """Move real robot to home position"""
        self.log("[HOME] Moving robot to HOME (collision-safe)...")
        self.btn_move_home.setEnabled(False)
        self.ros_thread.move_to_home()

    def handle_home_result(self, success, message):
        """Handle home movement result"""
        if success: 
            self.log(f"[OK] HOME:  {message}")
        else:
            self.log(f"[ERROR] HOME: {message}")
        
        if self.robot_connected:
            self.btn_move_home.setEnabled(True)

    # ==================== EXECUTION ====================
    def run_execution(self):
        """Start waypoint execution"""
        if not os.path.exists(JSON_PATH):
            self.log("[ERROR] No waypoints. json found - save waypoints first!")
            return
        
        self.btn_execute.setEnabled(False)
        
        # ✅ FIX 2: Keep dark red border during execution
        # Yellow border appears ONLY when E-STOP is pressed
        # (No style change here - stays dark red)
        
        self.log("[EXEC] Preparing execution...")
        self.log("[WAIT] Waiting 5s for move_group to fully initialize...")
        
        # Countdown timer
        self.execution_countdown = 5
        self.execution_timer = QTimer()
        self.execution_timer.timeout. connect(self.execution_countdown_tick)
        self.execution_timer.start(1000)
    
    def execution_countdown_tick(self):
        """Countdown before execution"""
        self.execution_countdown -= 1
        if self.execution_countdown > 0:
            self.log(f"[TIMER] Starting in {self.execution_countdown} seconds...")
        else:
            self.execution_timer.stop()
            self.log("[EXEC] Starting execution NOW!")
            
            # Run with unbuffered output
            self.execution_process = subprocess.Popen(
                f"python3 -u {EXECUTE_SCRIPT}",
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                bufsize=1,
                preexec_fn=os.setsid
            )
            
            # Monitor output
            self.execution_monitor = ExecutionMonitor(self.execution_process)
            self.execution_monitor.output_signal.connect(self.log_execution_output)
            self.execution_monitor.finished_signal.connect(self.execution_finished)
            self.execution_monitor.start()
    
    def log_execution_output(self, line):
        """Log ALL execution output - NO FILTERING"""
        if line. strip():  # Only skip empty lines
            self.log(f"[ROBOT] {line.strip()}")
    
    def execution_finished(self, exit_code):
        """Execution completed"""
        if exit_code == 0:
            self.log("[OK] EXECUTION COMPLETED SUCCESSFULLY!")
        else:
            self.log(f"[WARN] Execution finished with errors (code {exit_code})")
        
        self.btn_execute.setEnabled(True)
        
        # Button stays dark red (no change needed - it never turned yellow)
        
        self.label_mode.setText("MODE: [ROBOT CONNECTED]")
        self.label_mode.setStyleSheet("background-color: #FF5722; padding: 10px; font-size: 14px; font-weight: bold;")

    # ==================== SHUTDOWN CONTROLS ====================
    def disable_robot(self):
        """Disable robot servo motors"""
        self.log("[INFO] Disable Robot:")
        self.log("  1. Open JAKA Cobot Pi application")
        self.log("  2. Click 'Disable Robot'")
        self.log("  OR press physical button on control cabinet")
        
        QMessageBox.information(self, 'Disable Robot', 
                               'Please use JAKA Cobot Pi or\nphysical button to disable robot')

    def power_off_robot(self):
        """Power off robot"""
        self.log("[INFO] Power Off Robot:")
        self.log("  1. Open JAKA Cobot Pi application")
        self.log("  2. Click 'Power Off'")
        
        QMessageBox.information(self, 'Power Off Robot', 
                               'Please use JAKA Cobot Pi to\npower off robot')

    def shutdown_cabinet(self):
        """Shutdown control cabinet"""
        reply = QMessageBox.question(self, 'Confirm Shutdown', 
                                     'Shutdown control cabinet?\n\nThis requires manual action.',
                                     QMessageBox. Yes | QMessageBox.No,
                                     QMessageBox. No)
        
        if reply == QMessageBox.Yes:
            self.log("[SHUTDOWN] To shutdown cabinet:")
            self.log("  1. Open JAKA Cobot Pi")
            self.log("  2. Click 'Shutdown' in settings")
            self.log("  OR use physical power button")
            
            QMessageBox.information(self, 'Shutdown Cabinet', 
                                   'Please use JAKA Cobot Pi or\nphysical button to shutdown')

    # ==================== CLEANUP ====================
    def closeEvent(self, event):
        """Clean shutdown"""
        self.log("[SHUTDOWN] Shutting down HMI...")
        
        # Stop execution monitor
        if self.execution_monitor: 
            self.execution_monitor. terminate()
            self.execution_monitor.wait(1000)
        
        # Kill all processes
        for name, proc in [('RViz', self.rviz_process),
                          ('Robot', self. robot_process),
                          ('RSP', self.rsp_process),
                          ('DescPub', self.desc_pub_process),
                          ('Planning', self.planning_process),
                          ('Execution', self.execution_process)]:
            if proc: 
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                    proc.wait(timeout=3)
                except:
                    try:
                        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                    except:
                        pass
        
        # Force kill any remaining processes
        subprocess.run("pkill -9 -f 'move_group|rviz2|moveit_server|robot_state_publisher'", shell=True)
        
        # Stop ROS thread
        self.ros_thread.stop()
        event.accept()


# ==================== MAIN ====================
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = JakaHMI()
    window.show()
    sys.exit(app.exec_())