from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, Image
from unix_msgs.msg import GripperCommand
from unix_msgs.msg import JointCommand
from controller_manager_msgs.srv import ListControllers, SwitchController

from teleoperation.interfaces.logging_config import logger
from teleoperation.robot_config.robot_loader import robot_const_proxy

class ROSBridge:
    def __init__(self, node: Node, config):
        self.node = node
        self.config = config

        self.init_publishers()
        self.init_subscribers()
        self.init_services()

    def init_publishers(self):
        self.pubs = {
            "dual_arm":         self.node.create_publisher(Float64MultiArray,   '/dual_arm_forward_position_controller/commands', 10),
            "dual_arm_max_torq":self.node.create_publisher(JointCommand,        '/joint_command_controller/joint_command', 10),
            "left_gripper":     self.node.create_publisher(GripperCommand,      '/left_gripper/gripper_command', 10),
            "right_gripper":    self.node.create_publisher(GripperCommand,      '/right_gripper/gripper_command', 10),
            "hand":             self.node.create_publisher(JointState,          '/dexhand_controller/dexhand_command', 10),
            "torso":            self.node.create_publisher(JointState,          '/lifting_controller/commands', 10),
            "chassis":          self.node.create_publisher(Twist,               '/cmd_vel', 10),
            "head":             self.node.create_publisher(Float64MultiArray,   '/head_forward_position_controller/commands', 10),
            "dual_gripper":     self.node.create_publisher(Float64MultiArray,   '/gripper_forward_position_controller/commands', 10),
        }
        if robot_const_proxy.TorsoConfig.CONTROL_MODE=='pos':
            self.pubs['torso'] = self.node.create_publisher(Float64MultiArray,  '/lifting_forward_position_controller/commands', 10)

    def init_subscribers(self):
        self.subs = {
            'joint_states': self.node.create_subscription(JointState,       '/joint_states', lambda msg: None, 10),
            'left_gripper': self.node.create_subscription(JointState,       '/left_gripper/joint_states', lambda msg: None, 10),
            'right_gripper':self.node.create_subscription(JointState,       '/right_gripper/joint_states', lambda msg: None, 10),
            "hand":         self.node.create_subscription(JointState,       '/dexhand_controller/joint_states', lambda msg: None, 10),
            "torso":        self.node.create_subscription(JointState,       '/lifting_controller/joint_states',lambda msg: None, 10),
            "rgb":          self.node.create_subscription(Image,            '/camera/color/image_raw', lambda msg: None, 1),
            "depth":        self.node.create_subscription(Image,            '/camera/depth/image_raw', lambda msg: None, 1),
        }
        if robot_const_proxy.TorsoConfig.CONTROL_MODE == 'pos':
            self.subs['torso'] = self.node.create_subscription(JointState,  '/lifting_forward_position_controller/joint_states',lambda msg: None, 10)

    def init_services(self):
        call_bk_group_list = MutuallyExclusiveCallbackGroup()
        self.list_controllers_client = self.node.create_client(ListControllers, '/controller_manager/list_controllers', callback_group=call_bk_group_list)

        call_bk_group_switch = MutuallyExclusiveCallbackGroup()
        if self.config.robot.get('enable_bt'):
            from btcpp_ros2_interfaces.action import ExecuteTree
            from btcpp_ros2_interfaces.msg import NodeStatus
            self.bt_client = ActionClient(self.node, ExecuteTree, '/behavior_server', callback_group=call_bk_group_switch)
        else:
            self.switch_controller_client = self.node.create_client(SwitchController, '/controller_manager/switch_controller', callback_group=call_bk_group_switch)

    def publish(self, topic_key, msg):
        pub = self.pubs.get(topic_key)
        if pub:
            pub.publish(msg)
        else:
            logger.warning(f"[ROSBridge] Unknown publish topic: {topic_key}")

    def switch_controller(self):
        active_controllers = self.get_active_controllers()
        if not 'dual_arm_forward_position_controller' in active_controllers:
            logger.info("dual_arm_forward_position_controller is not active")
            controller_actived = self._switch_controller()
            logger.info(f'Switch to dual_arm_forward_position_controller is succeed: {controller_actived}')
        else:
            controller_actived = True
        return controller_actived

    def get_active_controllers(self):
        while not self.list_controllers_client.wait_for_service(timeout_sec=1.0):
            logger.warning('Waiting for /controller_manager/list_controllers service...')

        request = ListControllers.Request()
        future = self.list_controllers_client.call_async(request)
        self.node.executor.spin_until_future_complete(future, timeout_sec=1.0)

        if future.result():
            return [controller.name for controller in future.result().controller if controller.state == "active"]
        else:
            logger.error('Failed to get response from service.')
            return []

    def _switch_controller(self):
        if self.config.robot.get('enable_bt'):
            return self._send_bt_goal()
        else:
            return self._switch_controller_direct()

    def _switch_controller_direct(self):
        if not self.switch_controller_client.wait_for_service(timeout_sec=1.0):
            logger.warning('SwitchController service not available.')
            return False

        # https://docs.ros.org/en/iron/p/controller_manager_msgs/interfaces/srv/SwitchController.html
        request = SwitchController.Request()
        request.deactivate_controllers = ['vmp_controller']
        request.activate_controllers = ['dual_arm_forward_position_controller', 'head_forward_position_controller', 'lifting_forward_position_controller'] #joint_command_controller
        request.strictness = 1
        request.activate_asap = False
        request.timeout.sec = 0
        request.timeout.nanosec = 0

        future = self.switch_controller_client.call_async(request)
        self.node.executor.spin_until_future_complete(future, timeout_sec=1.0)
        # print(future.result())
        if future.result().ok==True:
            return True
        else:
            logger.error('Failed to switch to /controller_manager/switch_controller.')
            return False

    def _send_bt_goal(self):
        self.bt_client.wait_for_server(timeout_sec=1.0)
        # logger.info('Sending goal request...')
        goal_msg = ExecuteTree.Goal()
        goal_msg.target_tree = "TeleOpBT"
        future = self.bt_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.node.executor.spin_until_future_complete(future, timeout_sec=1.0)
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            logger.info('BT Goal rejected.')
            return False
        # logger.info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        self.node.executor.spin_until_future_complete(result_future, timeout_sec=1.0)

        # logger.info(f'Result: {result_future.result().result}')
        return result_future.result().result.node_status.status == NodeStatus.SUCCESS

    def feedback_callback(self, feedback_msg):
        logger.info(f'Feedback received: {feedback_msg.feedback}')






