import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
import xml.etree.ElementTree as ET

class JointSpaceMotion(Node):  
    def __init__(self):
        super().__init__('joint_space_motion')
        self.group_name = 'arm'     
        self.joint_names = []
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
        self.create_subscription(String, '/robot_description_semantic', self.srdf_callback, qos_profile)
        self.__move_action_client = ActionClient(self, action_type=MoveGroup, action_name="move_action")
        self.joint_names_ready = False

    def srdf_callback(self, msg):
        self.joint_names = self.get_joint_names_from_group(msg.data)
        if self.joint_names:
            self.get_logger().info(f'Retrieved joint names for group {self.group_name}: {self.joint_names}')
            self.joint_names_ready = True
        else:
            self.get_logger().error(f'Failed to retrieve joint names for group {self.group_name}.')
    
    def get_joint_names_from_group(self, srdf_data):
        if not srdf_data:
            self.get_logger().warn('SRDF data is not yet available.')
            return []
        srdf_root = ET.fromstring(srdf_data)
        for group in srdf_root.findall("group"):
            if group.get("name") == self.group_name:
                self.joint_names = [joint.get("name") for joint in group.findall("joint")]
                return self.joint_names

        self.get_logger().warn(f'Planning group {self.group_name} not found in SRDF.')
        return []

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

    def send_joint_motion_request(self, joint_positions):
        self.get_logger().info('Sending joint motion request...')
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self.group_name
        
        if not self.joint_names:
            return None
        
        constraints = Constraints()
        constraints.joint_constraints = []

        i=0
        for joint_name in self.joint_names:
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = joint_positions[i]
            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)
            i = i+1

        goal_msg.request.goal_constraints = [constraints]        
        future = self.__move_action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_cb)
        future.add_done_callback(self.goal_response_callback)

    def run( self ):
        
        self.__move_action_client.wait_for_server()
        joint_positions = [0.0, -1.0, 0.0, -2.35, 0.0, 1.57, 0.78]  
        while ( self.joint_names_ready == False):
            rclpy.spin_once(self)
        self.send_joint_motion_request(joint_positions)
        rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args)
    fk_client = JointSpaceMotion()
    fk_client.run() 
    rclpy.shutdown()

if __name__ == '__main__':
    main()
