from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from mavros_msgs.msg import State, SysStatus, VfrHud
from sensor_msgs.msg import NavSatFix
from rclpy.qos import qos_profile_default, qos_profile_sensor_data
from message_filters import Subscriber, ApproximateTimeSynchronizer
from PySide6.QtQml import QmlElement
from PySide6.QtCore import QObject, Signal
import time

QML_IMPORT_NAME = "MAVROS"
QML_IMPORT_MAJOR_VERSION = 1


@QmlElement
class MAVROS(QObject):
    positionChanged = Signal(float, float, float,int)

    def __init__(self, node: Node, namespace: str):
        super().__init__()
        self.subscriptions = []
        self.node = node
        self.namespace = namespace
        self.callback_group = ReentrantCallbackGroup()

        self.create_subcriber_variables()
        self.create_subscribers()
        queue_size = 10
        self.sync = ApproximateTimeSynchronizer([self.vfr_hud_sub, self.global_position_sub], queue_size, 0.5)
        self.sync.registerCallback(self.sync_gps_cb)
        self.last_gps_emit_time = 0.0

    def create_subcriber_variables(self):
        self.state_sub = None
        self.state = State()

        self.sys_status_sub = None
        self.sys_status = SysStatus()

        self.global_position_sub = None
        self.global_position = NavSatFix()

        self.vfr_hud_sub = None
        self.rel_alt = VfrHud()

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
            self.global_position_sub = Subscriber(
                self.node,
                NavSatFix,
                global_position_topic_name,
                qos_profile=qos_profile_sensor_data,
                callback_group=self.callback_group,
            )
            self.subscriptions.append(self.global_position_sub)

        vfr_hud_topic_name = f"{self.namespace}/vfr_hud"
        if self.vfr_hud_sub is None:
            self.vfr_hud_sub = Subscriber(
                self.node,
                VfrHud,
                vfr_hud_topic_name,
                qos_profile=qos_profile_sensor_data,
                callback_group=self.callback_group,
            )
            self.subscriptions.append(self.vfr_hud_sub)

    def state_cb(self, msg: State):
        self.state = msg

    def sys_status_cb(self, msg: SysStatus):
        self.sys_status = msg

    def sync_gps_cb(self, vfr_hud, global_position):
        seconds_since_epoch = time.time()
        if(seconds_since_epoch - self.last_gps_emit_time > 1):
            self.positionChanged.emit(global_position.latitude, global_position.longitude, vfr_hud.altitude,vfr_hud.heading)
            self.last_gps_emit_time = time.time()

    def destroy(self):
        for sub in self.subscriptions:
            self.node.destroy_subscription(sub)
        for client in self.service_clients:
            self.node.destroy_client(client)
