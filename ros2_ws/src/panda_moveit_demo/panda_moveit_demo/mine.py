import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.srv import GetPositionFK, GetPlanningScene
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    PlanningSceneComponents,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
)
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive

from sensor_msgs.msg import (
    JointState,
)
from moveit_msgs.msg import JointConstraint

import time


class ControllerNode(Node):

    def __init__(self):
        super().__init__('controller_node')

        self.__move_action_client = ActionClient(self, action_type=MoveGroup, action_name="move_action")
        self.move_action_goal = MoveGroup.Goal()

        # Client 1
        self.planning_scene_client = self.create_client(GetPlanningScene, "get_planning_scene")
        while not self.planning_scene_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info("Waiting for the planning scene service...")

        # Client 2
        self.fk_client = self.create_client(GetPositionFK, "compute_fk")
        while not self.fk_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info("Waiting for the FK service...")

        self.group_name = "arm"
        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]

        self.semi_extended = [
            0.0,     # joint1 base
            -0.8,    # joint2 hombro
            0.0,     # joint3
            -2.3,    # joint4 codo
            0.0,     # joint5
            1.6,     # joint6
            0.0      # joint7 muñeca
        ]

    def send_joint_goal(self, joint_positions):

        self.move_action_goal.request.group_name = self.group_name
        self.move_action_goal.request.num_planning_attempts = 3
        self.move_action_goal.request.allowed_planning_time = 5.0
        self.move_action_goal.request.max_velocity_scaling_factor = 0.2

        constraints = Constraints()

        for name, position in zip(self.joint_names, joint_positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(position)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        self.move_action_goal.request.goal_constraints = [constraints]

        future = self.__move_action_client.send_goal_async(
            self.move_action_goal,
            feedback_callback=self.feedback_cb
        )
        future.add_done_callback(self.goal_response_callback)

        
    def get_current_joint_values(self):
        """
        Retrive the current configuration of the robot.
        request to /joint_states topic
        only for group name
        get the part of the planning scene associated with the ROBOT_STATE
        """

        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ROBOT_STATE

        future = self.planning_scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            current_joint_state = response.scene.robot_state.joint_state

            joint_positions = []
            for joint_name in self.joint_names:
                if joint_name in current_joint_state.name:
                    index = current_joint_state.name.index(joint_name)
                    joint_positions.append(current_joint_state.position[index])

            self.get_logger().info(f"Curent joint positions for group {self.group_name}: {joint_positions}")
            return joint_positions
        
        else:
            self.get_logger().error("Failed to retrieve the current joint state from the planning scene.")
            return None

    def call_fk_service(self, joint_postitions, fk_link_names):
        """
        Initialize the GetPositionFK.Request object defining th edesired link for the FK calculation
        """

        if not self.joint_names:
            return None
        
        request = GetPositionFK.Request()
        request.fk_link_names = fk_link_names

        # Build joint state object
        joint_state = JointState()
        joint_state.name = self.joint_names
        joint_state.position = joint_postitions

        request.robot_state.joint_state = joint_state
        request.header.frame_id = "link0"

        # Send FK request
        future = self.fk_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            return response.pose_stamped

    def build_joint_lock_constraints(self):

        constraints = Constraints()

        # Bloquear joint3
        jc3 = JointConstraint()
        jc3.joint_name = "joint3"
        jc3.position = self.semi_extended[2]   # valor inicial
        jc3.tolerance_above = 0.001
        jc3.tolerance_below = 0.001
        jc3.weight = 1.0

        # Bloquear joint5
        jc5 = JointConstraint()
        jc5.joint_name = "joint5"
        jc5.position = self.semi_extended[2]
        jc5.tolerance_above = 0.001
        jc5.tolerance_below = 0.001
        jc5.weight = 1.0

        constraints.joint_constraints.append(jc3)
        #constraints.joint_constraints.append(jc5)

        return constraints
        
    def send_cartesian_motion_request(self, pconstraint, oconstraint):
        """
        Used to call move_group action
        the goal is pecified by defininng a constraint that contains both
        position and orientation constraints.

        """

        constraints = Constraints()
        constraints.joint_constraints = []
        self.move_action_goal.request.group_name = self.group_name
        self.move_action_goal.request.num_planning_attempts = 3
        self.move_action_goal.request.allowed_planning_time = float(5)
        self.move_action_goal.request.max_velocity_scaling_factor = 0.1

        self.move_action_goal.request.goal_constraints = [Constraints()]
        self.move_action_goal.request.goal_constraints[-1].position_constraints.append(pconstraint)
        self.move_action_goal.request.goal_constraints[-1].orientation_constraints.append(oconstraint)

        #self.move_action_goal.request.path_constraints = self.build_joint_lock_constraints()

        # Contact move_group action
        future = self.__move_action_client.send_goal_async(self.move_action_goal, feedback_callback=self.feedback_cb)
        future.add_done_callback(self.goal_response_callback)


    def feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Feedback: {feedback}")


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


    def build_position_constraints(self, pose, displacement):

        pconstraint = PositionConstraint()
        pconstraint.header.frame_id = "link0"
        pconstraint.link_name = "link7"

        # displacement of 20 cm toward z axis
        pconstraint.constraint_region.primitive_poses.append(Pose())
        pconstraint.constraint_region.primitive_poses[0].position.x = float(pose.position.x)
        pconstraint.constraint_region.primitive_poses[0].position.y = float(pose.position.y)
        pconstraint.constraint_region.primitive_poses[0].position.z = float(pose.position.z + displacement)

        pconstraint.constraint_region.primitives.append(SolidPrimitive())
        pconstraint.constraint_region.primitives[0].type = 2  # Sphere
        pconstraint.constraint_region.primitives[0].dimensions = [0.001]
        pconstraint.weight = 1.0

        return pconstraint
    
    def build_orientation_constraint(self, pose):

        oconstraint = OrientationConstraint()
        oconstraint.header.frame_id = "link0"
        oconstraint.link_name = "link7"

        oconstraint.orientation.w = float(pose.orientation.w)
        oconstraint.orientation.x = float(pose.orientation.x)
        oconstraint.orientation.y = float(pose.orientation.y)
        oconstraint.orientation.z = float(pose.orientation.z)

        oconstraint.absolute_x_axis_tolerance = 0.001
        oconstraint.absolute_y_axis_tolerance = 0.001
        oconstraint.absolute_z_axis_tolerance = 0.001

        oconstraint.parameterization = 1
        oconstraint.weight = 1.0
    
        return oconstraint

    def run(self):
        
        self.__move_action_client.wait_for_server()

        while not self.joint_names:
            rclpy.spin_once(self)

        self.send_joint_goal(self.semi_extended)
        self.get_logger().info("Sending semi-extended order")
        time.sleep(15)

        for d in [0.2, -0.2, 0.2, -0.2, 0.2, -0.2]:
            current_joint_positions = self.get_current_joint_values()

            pose = Pose()
            if current_joint_positions:
                fk_link_names = ['link7']
                r = self.call_fk_service(current_joint_positions, fk_link_names)
                pose = r[0].pose

                self.get_logger().info(f"pose: {pose}")

            pconstraint = self.build_position_constraints(pose, d)
            oconstraint = self.build_orientation_constraint(pose)

            self.move_action_goal.request.goal_constraints = [Constraints()]
            self.move_action_goal.request.goal_constraints[-1].position_constraints.append(pconstraint)
            self.send_cartesian_motion_request(pconstraint, oconstraint)

            time.sleep(20)
            #current_joint_positions = self.get_current_joint_values()


def main(args=None):
    rclpy.init(args=args)
    controller = ControllerNode()
    controller.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
