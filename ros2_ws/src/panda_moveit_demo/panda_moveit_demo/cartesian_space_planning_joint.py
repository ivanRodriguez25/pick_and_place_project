import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints
from moveit_msgs.srv import GetPositionFK, GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from moveit_msgs.msg import PositionConstraint, OrientationConstraint
from moveit_msgs.srv import GetCartesianPath
import xml.etree.ElementTree as ET
from shape_msgs.msg import SolidPrimitive
class CartesianSpaceMotion(Node):  
    def __init__(self):
        super().__init__('joint_space_motion')
        
        self.srdf_data = None
        self.group_name = 'arm'     
        self.joint_names = []
        self.planning_scene_client = self.create_client(GetPlanningScene, 'get_planning_scene')
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
        self.create_subscription(String, '/robot_description_semantic', self.srdf_callback, qos_profile)
        self.move_action_goal = MoveGroup.Goal()
        self.fk_client = self.create_client(GetPositionFK, 'compute_fk')
        while not self.fk_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Waiting for the FK service...')
        while not self.planning_scene_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Waiting for the planning scene service...')

        self.__move_action_client = ActionClient(self, action_type=MoveGroup, action_name="move_action")
        self.joint_names_ready = False
        self.__cartesian_path_request = GetCartesianPath.Request()

    def srdf_callback(self, msg):
        self.srdf_data = msg.data
        self.joint_names = self.get_joint_names_from_group()

        if self.joint_names:
            self.get_logger().info(f'Retrieved joint names for group {self.group_name}: {self.joint_names}')
            self.joint_names_ready = True
        else:
            self.get_logger().error(f'Failed to retrieve joint names for group {self.group_name}.')
    
    def get_joint_names_from_group(self):
        if not self.srdf_data:
            self.get_logger().warn('SRDF data is not yet available.')
            return []
        srdf_root = ET.fromstring(self.srdf_data)
        for group in srdf_root.findall("group"):
            if group.get("name") == self.group_name:
                joint_names = [joint.get("name") for joint in group.findall("joint")]
                return joint_names

        self.get_logger().warn(f'Planning group {self.group_name} not found in SRDF.')
        return []
    
    def get_current_joint_values(self):
        """Retrieve the current joint values using the get_planning_scene service."""
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ROBOT_STATE

        future = self.planning_scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            current_joint_state = response.scene.robot_state.joint_state

            # Filter the joint positions based on the relevant joint names for the planning group
            joint_positions = []
            for joint_name in self.joint_names:
                if joint_name in current_joint_state.name:
                    index = current_joint_state.name.index(joint_name)
                    joint_positions.append(current_joint_state.position[index])

            self.get_logger().info(f'Current joint positions for group {self.group_name}: {joint_positions}')
            return joint_positions
        else:
            self.get_logger().error('Failed to retrieve the current joint state from the planning scene.')
            return None
    
    def call_fk_service(self, joint_positions, fk_link_names):
        if not self.joint_names:
            return None

        request = GetPositionFK.Request()
        request.fk_link_names = fk_link_names

        joint_state = JointState()
        joint_state.name = self.joint_names
        joint_state.position = joint_positions

        request.robot_state.joint_state = joint_state

        future = self.fk_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            for pose in response.pose_stamped:
                self.get_logger().info(f'Link: {pose.header.frame_id} Pose: {pose.pose}')
            return response.pose_stamped  
        else:
            self.get_logger().error('FK Computation Failed.')
            return None

    def feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        #self.get_logger().info('Result: {0}'.format(result.motion_done))
        if( result.error_code.val == 1 ):
            print("Plan and Execute Succeeded")
        else:
            print("Plan and Execute Failed")

    #Questo pianifica in joint space attenzione!
    def send_cartesian_motion_request(self, pconstraint, oconstraint):
        self.get_logger().info('Sending joint motion request...')
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'arm'  # Replace with your planning group name
        if not self.joint_names:
            print("None")
            return None
        
        constraints = Constraints()
        constraints.joint_constraints = []
        joint_names = self.get_joint_names_from_group()  # This should be replaced with actual joint names
        print("joint_names: ", joint_names)
        self.move_action_goal.request.group_name = 'arm'  # Replace with your planning group name
        self.move_action_goal.request.num_planning_attempts = 5
        self.move_action_goal.request.allowed_planning_time = float(5)
        self.move_action_goal.request.max_velocity_scaling_factor = 0.0

        self.move_action_goal.request.goal_constraints = [Constraints()]
        self.move_action_goal.request.goal_constraints[-1].position_constraints.append(pconstraint)
        self.move_action_goal.request.goal_constraints[-1].orientation_constraints.append(oconstraint)

        future = self.__move_action_client.send_goal_async(self.move_action_goal, feedback_callback=self.feedback_cb)
        future.add_done_callback(self.goal_response_callback)

        
    def run( self ):
        self.__move_action_client.wait_for_server()

        while not self.joint_names:
            rclpy.spin_once(self)

        current_joint_positions = self.get_current_joint_values()

        pose = Pose()
        if current_joint_positions:
            fk_link_names = ['link7']  # Replace with actual link names of your robot
            r = self.call_fk_service(current_joint_positions, fk_link_names)
            pose = r[0].pose

        pconstraint = PositionConstraint()
        pconstraint.header.frame_id = "link0"
        pconstraint.link_name = 'link7'

        pconstraint.constraint_region.primitive_poses.append(Pose())
        pconstraint.constraint_region.primitive_poses[0].position.x = float(pose.position.x)
        pconstraint.constraint_region.primitive_poses[0].position.y = float(pose.position.y)
        pconstraint.constraint_region.primitive_poses[0].position.z = float(pose.position.z + 0.2)
        pconstraint.constraint_region.primitives.append(SolidPrimitive())
        pconstraint.constraint_region.primitives[0].type = 2  # Sphere
        pconstraint.constraint_region.primitives[0].dimensions = [0.001]
        pconstraint.weight = 1.0        
        
        oconstraint = OrientationConstraint()
        oconstraint.header.frame_id = "link0"
        oconstraint.link_name = 'link7'
        oconstraint.orientation.w = float(pose.orientation.w)
        oconstraint.orientation.x = float(pose.orientation.x)
        oconstraint.orientation.y = float(pose.orientation.y)
        oconstraint.orientation.z = float(pose.orientation.z)
        oconstraint.absolute_x_axis_tolerance = 0.001
        oconstraint.absolute_y_axis_tolerance = 0.001
        oconstraint.absolute_z_axis_tolerance = 0.001
        oconstraint.parameterization = 1
        oconstraint.weight = 1.0        

        self.move_action_goal.request.goal_constraints = [Constraints()]
        self.move_action_goal.request.goal_constraints[-1].position_constraints.append(pconstraint)
        self.send_cartesian_motion_request( pconstraint, oconstraint )
               
        rclpy.spin(self)
        
def main(args=None):
    rclpy.init(args=args)
    fk_client = CartesianSpaceMotion()
    fk_client.run() 
    rclpy.shutdown()

if __name__ == '__main__':
    main()
