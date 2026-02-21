import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from moveit_msgs.srv import GetPositionFK, GetPlanningScene
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from moveit_msgs.msg import PlanningSceneComponents
import xml.etree.ElementTree as ET

class ComputeFKClient(Node):
    def __init__(self):
        super().__init__('compute_fk_client')

        self.srdf_data = None
        self.group_name = 'arm'     
        self.joint_names = []
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.create_subscription(String, '/robot_description_semantic', self.srdf_callback, qos_profile)

        # Create a client for the compute_fk service
        self.fk_client = self.create_client(GetPositionFK, 'compute_fk')
        
        # Create a client for the get_planning_scene service
        self.planning_scene_client = self.create_client(GetPlanningScene, 'get_planning_scene')

        # Wait for the services to become available
        while not self.fk_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Waiting for the FK service...')
        
        while not self.planning_scene_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Waiting for the planning scene service...')
        
        self.get_logger().info('FK and Planning Scene services are available.')

    def srdf_callback(self, msg):
        """Callback function to handle the SRDF data from the /robot_description_semantic topic."""
        self.srdf_data = msg.data
        self.joint_names = self.get_joint_names_from_group()
        if self.joint_names:
            self.get_logger().info(f'Retrieved joint names for group {self.group_name}: {self.joint_names}')
        else:
            self.get_logger().error(f'Failed to retrieve joint names for group {self.group_name}.')

    def get_joint_names_from_group(self):
        """Retrieve joint names associated with the specified planning group from the SRDF data."""
        if not self.srdf_data:
            self.get_logger().warn('SRDF data is not yet available.')
            return []

        srdf_root = ET.fromstring(self.srdf_data)
        print("self.srdf_data: ", self.srdf_data)

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
            self.get_logger().error('No joint names available. Ensure SRDF has been received and parsed.')
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
            self.get_logger().info('FK Computation Successful.')
            for pose in response.pose_stamped:
                self.get_logger().info(f'Link: {pose.header.frame_id} Pose: {pose.pose}')
            return response
        else:
            self.get_logger().error('FK Computation Failed.')
            return None

def main(args=None):
    rclpy.init(args=args)
    
    fk_client = ComputeFKClient()

    while not fk_client.joint_names:
        rclpy.spin_once(fk_client)

    current_joint_positions = fk_client.get_current_joint_values()
    
    if current_joint_positions:
        fk_link_names = ['hand']  # Replace with actual link names of your robot
        fk_client.call_fk_service(current_joint_positions, fk_link_names)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
