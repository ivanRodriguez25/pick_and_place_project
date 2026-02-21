import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint
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

        self.joint_names_ready = False
        self.__cartesian_path_request = GetCartesianPath.Request()
        self._plan_cartesian_path_service = self.create_client(srv_type=GetCartesianPath, srv_name="compute_cartesian_path")
        self._execute_trajectory_action_client = ActionClient(self, ExecuteTrajectory, "execute_trajectory")

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
    



    def send_cartesian_path_execution(self, execute_trajectory_goal):
        
        future = self._execute_trajectory_action_client.send_goal_async( execute_trajectory_goal, feedback_callback=self.feedback_cb) # QUIII(self.move_action_goal, feedback_callback=self.feedback_cb)
        future.add_done_callback(self.goal_response_callback)
    def run( self ):

        while not self.joint_names:
            rclpy.spin_once(self)


        current_joint_positions = self.get_current_joint_values()
        print("current_joint_positions: ", current_joint_positions)

        pose = Pose()
        if current_joint_positions:
            fk_link_names = ['link7']  # Replace with actual link names of your robot
            r = self.call_fk_service(current_joint_positions, fk_link_names)
            pose = r[0].pose
            print("reeeeeeeeeeeesult ", r[0].pose)
        
        
        if not self._plan_cartesian_path_service.service_is_ready():
            self._node.get_logger().warn(
                f"Service '{self._plan_cartesian_path_service.srv_name}' is not yet available. Better luck next time!"
            )
            return None
        target_pose = Pose()
        target_pose = pose

        target_pose.position.z = target_pose.position.z + 0.2
        #target_pose.orientation.w = 1.0
        #target_pose.orientation.x = 0.0
        #target_pose.orientation.y = 0.0
        #target_pose.orientation.z = 0.0
        
        stamp = self.get_clock().now().to_msg()
        self.__cartesian_path_request.header.stamp = stamp
        self.__cartesian_path_request.group_name = "arm"
        self.__cartesian_path_request.start_state.joint_state = JointState()
        self.__cartesian_path_request.start_state.joint_state.name = self.get_joint_names_from_group()
        self.__cartesian_path_request.start_state.joint_state.position = self.get_current_joint_values()
        self.__cartesian_path_request.max_step = 0.0025
        self.__cartesian_path_request.waypoints = [target_pose]
        print("target pose: " , target_pose)
        future = self._plan_cartesian_path_service.call_async(
            self.__cartesian_path_request
        )

        future = self._plan_cartesian_path_service.call_async(self.__cartesian_path_request)
        rclpy.spin_until_future_complete(self, future)
        execute_trajectory_goal = ExecuteTrajectory.Goal()

        if future.result() is not None:
            response = future.result()
            #            if MoveItErrorCodes.SUCCESS == res.error_code.val: TOADD!!
            print("Result: ", response.solution.joint_trajectory)
            execute_trajectory_goal.trajectory.joint_trajectory = response.solution.joint_trajectory

        else:
            self.get_logger().error('FK Computation Failed.')
        
        print("Done")


        if not self._execute_trajectory_action_client.server_is_ready():
            self._node.get_logger().warn(
                f"Action server '{self._execute_trajectory_action_client._action_name}' is not yet available. Better luck next time!"
            )
            return
        self.send_cartesian_path_execution( execute_trajectory_goal)
        #while ( self.joint_names_ready == False):
        #    rclpy.spin_once(self)
        #self.send_joint_motion_request(joint_positions)
        rclpy.spin(self)
        
def main(args=None):
    rclpy.init(args=args)
    fk_client = CartesianSpaceMotion()
    fk_client.run() 
    rclpy.shutdown()

if __name__ == '__main__':
    main()
