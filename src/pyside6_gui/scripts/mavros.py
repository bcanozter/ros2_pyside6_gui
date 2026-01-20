from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from mavros_msgs.msg import State, SysStatus
from sensor_msgs.msg import NavSatFix
from rclpy.qos import qos_profile_default, qos_profile_sensor_data


class MAVROS:
    def __init__(self, node: Node, namespace: str):
        self.subscriptions = []
        self.node = node
        self.namespace = namespace
        self.callback_group = ReentrantCallbackGroup()

        self.create_subcriber_variables()
        self.create_subscribers()

    def create_subcriber_variables(self):
        self.state_sub = None
        self.state = State()

        self.sys_status_sub = None
        self.sys_status = SysStatus()

        self.global_position_sub = None
        self.global_position = NavSatFix()


    def create_subscribers(self):
        state_topic_name = f"{self.namespace}/state"
        if self.state_sub is None:
            self.state_sub = self.node.create_subscription(
                State,
                state_topic_name,
                self.state_cb,
                qos_profile_default,
                callback_group=self.callback_group,
            )
            self.subscriptions.append(self.state_sub)
        sys_status_topic_name = f"{self.namespace}/sys_status"
        if self.sys_status_sub is None:
            self.sys_status_sub = self.node.create_subscription(
                SysStatus,
                sys_status_topic_name,
                self.sys_status_cb,
                qos_profile_sensor_data,
                callback_group=self.callback_group,
            )
            self.subscriptions.append(self.sys_status_sub)
        global_position_topic_name = f"{self.namespace}/global_position/global"
        if self.global_position_sub is None:
            self.global_position_sub = self.node.create_subscription(
                NavSatFix,
                global_position_topic_name,
                self.global_position_cb,
                qos_profile_sensor_data,
                callback_group=self.callback_group,
            )
            self.subscriptions.append(self.global_position_sub)

    def state_cb(self, msg: State):
        self.state = msg

    def sys_status_cb(self, msg: SysStatus):
        self.sys_status = msg

    def global_position_cb(self, msg: NavSatFix):
        print(msg)
        self.global_position = msg

    def destroy(self):
        for sub in self.subscriptions:
            self.node.destroy_subscription(sub)
        for client in self.service_clients:
            self.node.destroy_client(client)
