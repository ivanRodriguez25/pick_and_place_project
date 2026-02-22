import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
import time

import xml.etree.ElementTree as ET


class JointSpaceContinuousMovement(Node):
    def __init__(self):
        super().__init__('joint_space_continuous_movement')

        self.group_name = 'arm'

        # Get semantic information from the arm reading from topic /robot_description_semantic
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.create_subscription(String, '/robot_description_semantic', self.srdf_callback, qos_profile)

        self.__move_action_client = ActionClient(self, action_type=MoveGroup, action_name="move_action")
        self.joint_names_ready = False
        self.srdf_data = None

        pi = 3.1415
        self.joint_positions_list  = [
            #|   J1 |   J2 |   J3 |   J3 |   J5 |    J6 |   J7 |
            [    0.0,   0.0,   0.0, -pi/2,   0.0, 0.6*pi,  0.0],
            #[    0.0,   0.0,   0.0, -pi/2,   0.0, 0.6*pi,  0.0],
            [    0.0, -1.78,   0.0,  -2.0,   0.0,   1.57,  0.78],
        ]

        self.current_idx = 0

    def srdf_callback(self, msg):

        self.srdf_data = msg.data
        self.joint_names = self.get_joint_names_from_group()

        if self.joint_names:
            self.get_logger().info(f"Retrieved joint names for group {self.group_name}: {self.joint_names}")
            self.joint_names_ready = True
        else:
            self.get_logger().info(f"Failed to retrieve joint names for group {self.group_name}")

    def get_joint_names_from_group(self):

        if not self.srdf_data:
            self.get_logger().warn("SRDF data is not yet available.")
            return []

        srdf_root = ET.fromstring(self.srdf_data)

        for group in srdf_root.findall("group"):
            if group.get("name") == self.group_name:
                joint_names = [joint.get("name") for joint in group.findall("joint")]
                return joint_names

        self.get_logger().warn(f"Planning group {self.group_name} not found in SRDF.")
        return []
    
    def feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Feedback: {feedback}")

    def goal_response_callback(self, future):

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return
        self.get_logger().info("Goal accepted :)")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):

        result = future.result().result
        status = future.result().status

        if result.error_code.val == 1:
            self.get_logger().info("Plan and Execute Succeded")

        else:
            self.get_logger().info(f"Plan and Execute Failed")
            self.get_logger().info(f"  Result status: {status}")
            self.get_logger().info(f"  Error code: {result.error_code.val}")

        time.sleep(10)
        self.current_idx = (self.current_idx + 1) % len(self.joint_positions_list)
        self.send_joint_motion_request(self.joint_positions_list[self.current_idx])

    def _build_constraints(self, joint_positions):

        constraints = Constraints()
        constraints.joint_constraints = []

        for i, joint_name in enumerate(self.joint_names):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = joint_positions[i]

            # Tolerance
            joint_constraint.tolerance_above = 0.1
            joint_constraint.tolerance_below = 0.1
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)

        return constraints

    def send_joint_motion_request(self, joint_positions):
        
        self.get_logger().info("Sending joint motion request..,")
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self.group_name
        goal_msg.request.allowed_planning_time = 10.0
        goal_msg.request.max_velocity_scaling_factor = 0.1
        goal_msg.request.max_acceleration_scaling_factor = 0.1

        if not self.joint_names:
            return None

        constraints = self._build_constraints(joint_positions)
        goal_msg.request.goal_constraints = [constraints]

        future = self.__move_action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_cb)
        future.add_done_callback(self.goal_response_callback)

        time.sleep(5)

    def run(self):

        self.__move_action_client.wait_for_server()

        while (self.joint_names_ready == False):
            rclpy.spin_once(self)

        self.get_logger().info(f"Sending new request...")
        self.send_joint_motion_request(
            self.joint_positions_list[self.current_idx]
        )
        rclpy.spin(self)


def main(args=None):
    rclpy.init(args=args)
    fk_client = JointSpaceContinuousMovement()
    fk_client.run()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
