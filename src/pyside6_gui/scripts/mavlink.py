from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2
from mavros_msgs.msg import WaypointList
import time


class MAVLinkDevice:
    def __init__(
        self,
        device,
        baudrate: int = 115200,
        source_system: int = 255,
        retries: int = 3,
        dialect="ardupilotmega",
    ):
        self.master = None
        self.system_id = None
        self.component_id = None
        self.uptime = 0
        self.dialect = dialect
        self.device = device
        self.baudrate = baudrate
        self.source_system = source_system
        self.retries = retries

    # Maybe not a good idea?
    def check_mavlink_connection(func):
        def wrapper(self, *args, **kwargs):
            if self.master is None:
                raise Exception(f"[{self.device}] You must call create_mavlink_connection() first")
            else:
                return func(self, *args, **kwargs)

        return wrapper

    @check_mavlink_connection
    def wait_heartbeat(self):
        msg = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
        if msg is not None:
            self.print_hearbeat_info(msg)
            return msg

    def create_mavlink_connection(self) -> bool:
        try:
            print(f"Connecting to {self.device}")
            self.master = mavutil.mavlink_connection(
                self.device,
                baud=self.baudrate,
                source_system=self.source_system,
                retries=self.retries,
            )
            heartbeat = self.wait_heartbeat()
            if heartbeat is None:
                print(f"[{self.device}] no heartbeat received.")
                return False

            # explicitly set the dialect.
            self.set_dialect(self.dialect)
            self.system_id = self.master.target_system
            self.component_id = self.master.target_component
            self.print_device_info()
            return True

        except Exception as e:
            # https://github.com/ArduPilot/pymavlink/blob/master/mavutil.py#L1256
            # Exception is only raised when retries == 0 i.e. last retry
            print(e)
            return False

    def set_dialect(self, dialect="ardupilotmega"):
        try:
            mavutil.set_dialect(dialect)
        except Exception as e:
            print("Failed to set MAVLink device dialect.")
            print(e)

    def print_device_info(self):
        print("-" * 80)
        print(f"Uptime: {time.strftime('%HH:%MM:%SS', time.gmtime(self.get_uptime()))}")
        print(f"Dialect: {self.dialect}")
        print(f"Device: {self.device}")
        print(f"System ID: {self.system_id}")
        print(f"Component ID: {self.component_id}")
        print(f"Baudrate: {self.baudrate}")
        print("-" * 80)

    def print_hearbeat_info(self, msg):
        # https://mavlink.io/en/messages/common.html#HEARTBEAT
        print("-" * 80)
        print(f"Type: {msg.type}")
        print(f"Autopilot: {msg.autopilot}")
        print(f"Base Mode: {msg.base_mode}")
        print(f"Custom Mode: {msg.custom_mode}")
        print(f"System Status: {msg.system_status}")
        print(f"Mavlink Version: {msg.mavlink_version}")
        print("-" * 80)

    @check_mavlink_connection
    # https://mavlink.io/en/services/mission.html#download_mission
    def download_mission(self):
        waypoints = []
        try:
            self.master.mav.mission_request_list_send(
                self.master.target_system,
                self.master.target_component,
                mavlink2.MAV_MISSION_TYPE_MISSION,
            )
            msg = self.master.recv_match(type="MISSION_COUNT", blocking=True, timeout=5)
            if msg is None:
                print("Failed to receive mission count")
                return

            if isinstance(msg.count, int) is False:
                print("msg.count is not an integer")
                return

            for i in range(msg.count):
                self.master.mav.mission_request_int_send(
                    self.master.target_system,
                    self.master.target_component,
                    i,
                    mavlink2.MAV_MISSION_TYPE_MISSION,
                )

                msg = self.master.recv_match(type="MISSION_ITEM_INT", blocking=True, timeout=5)
                if msg is None:
                    print(f"Failed to receive waypoint {i}")
                    continue
                else:
                    waypoint = self.handle_mission_item(msg)
                    waypoints.append(waypoint)
                    print(f"Received waypoint {i}: {waypoint}")

            try:
                self.master.mav.mission_ack_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_MISSION_ACCEPTED,
                )
            except Exception as e:
                print(e)
            print(f"Read {len(waypoints)} waypoints")
            return waypoints

        except Exception as e:
            print(f"Error reading mission: {e}")
            return None

    def handle_mission_item(self, item):
        # https://mavlink.io/en/messages/common.html#MISSION_ITEM_INT
        waypoint = {
            "target_system": item.target_system,
            "target_component": item.target_component,
            "seq": item.seq,
            "frame": item.frame,
            "command": item.command,
            "current": item.current,
            "autocontinue": item.autocontinue,
            "param1": item.param1,
            "param2": item.param2,
            "param3": item.param3,
            "param4": item.param4,
            "x": item.x / 1e7,
            "y": item.y / 1e7,
            "z": item.z,
        }
        # https://github.com/ArduPilot/pymavlink/blob/master/mavutil.py#L54
        if mavutil.mavlink20() is True:
            waypoint["mission_type"] = item.mission_type
        return waypoint

    @check_mavlink_connection
    def mission_set_current(self, seq: int = 0) -> bool:
        try:
            # https://mavlink.io/en/messages/common.html#MISSION_SET_CURRENT
            self.master.waypoint_set_current_send(seq)
            print(f"[{self.device}] waiting for MISSION_CURRENT response")

            msg = self.master.recv_match(type="MISSION_CURRENT", blocking=True, timeout=5)
            if msg is not None:
                print(f"[{self.device}] mission_set_current wp {seq}")
                return True
            else:
                print(f"[{self.device}] mission_set_current timed out")
                return False

        except Exception as e:
            print(f"Error starting mission on {self.device}: {e}")
            return False

    @check_mavlink_connection
    def arm(self):
        self.send_command_long(mavlink2.MAV_CMD_COMPONENT_ARM_DISARM, 1)

    @check_mavlink_connection
    def disarm(self):
        self.send_command_long(mavlink2.MAV_CMD_COMPONENT_ARM_DISARM, 0)

    @check_mavlink_connection
    def mission_start(self):
        self.send_command_long(mavlink2.MAV_CMD_MISSION_START)

    @check_mavlink_connection
    def return_to_launch_cmd(self):
        ret = self.send_command_long(mavlink2.MAV_CMD_NAV_RETURN_TO_LAUNCH)
        print(f"Return to Launch command has been sent to {self.device}")
        if ret != 0:
            print(f"Return to Launch command result: {ret}")
        return ret

    @check_mavlink_connection
    def send_land_cmd(self, lat: float = 0, lon: float = 0):
        ret = self.send_command_long(mavlink2.MAV_CMD_NAV_LAND)
        print(f"Land command has been sent to {self.device}")
        if ret != 0:
            print(f"Land command result: {ret}")
        return ret

    @check_mavlink_connection
    def set_mode_cmd(self, mode, custom_mode, custom_submode):
        ret = self.send_command_long(mavlink2.MAV_CMD_DO_SET_MODE, mode, custom_mode, custom_submode)
        print(
            f"[{self.device}] Set mode command has been sent. Mode={mode}, custom_mode={custom_mode}, custom_submode={custom_submode}"
        )
        if ret != 0:
            print(
                f"[{self.device}] Set mode command result: {ret}. Mode={mode}, custom_mode={custom_mode}, custom_submode={custom_submode} "
            )
        return ret

    @check_mavlink_connection
    def request_message(self, message_id: int):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavlink2.MAV_CMD_REQUEST_MESSAGE,
            0,
            message_id,
            0,
            0,
            0,
            0,
            0,
            0,
        )

    @check_mavlink_connection
    def send_command_long(
        self,
        MAV_CMD,
        param1: float = 0.0,
        param2: float = 0.0,
        param3: float = 0.0,
        param4: float = 0.0,
        param5: float = 0.0,
        param6: float = 0.0,
        param7: float = 0.0,
    ) -> int:
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            MAV_CMD,
            0,
            param1,
            param2,
            param3,
            param4,
            param5,
            param6,
            param7,
        )
        ret = self.command_ack()
        return ret

    @check_mavlink_connection
    def clear_mission(self) -> bool:
        self.master.mav.mission_clear_all_send(
            self.master.target_system,
            self.master.target_component,
        )
        ret = self.mission_ack()
        if ret != mavlink2.MAV_MISSION_ACCEPTED:
            print(f"[{self.device}] Error clearing mission, ACK type: {ret}")
            return False

        print(f"[{self.device}] mission cleared.")
        return True

    # https://mavlink.io/en/services/mission.html#uploading_mission
    @check_mavlink_connection
    def upload_mission(self, waypointlist: WaypointList) -> bool:
        if (waypointlist is None) or (not waypointlist):
            print("No waypoints provided, ignoring..")
            return False
        current_seq = waypointlist.current_seq
        waypoints = waypointlist.waypoints
        try:
            ret = self.clear_mission()
            if ret is False:
                return False

            count = len(waypoints)
            print(f"Uploading mission with {count} waypoints")

            self.master.mav.mission_count_send(
                self.master.target_system,
                self.master.target_component,
                count,
                mavlink2.MAV_MISSION_TYPE_MISSION,
            )

            #! HUGE assumption: waypoints is ordered by seq already.
            # if we receive out of order waypoints, we need to search the list and order it.
            for seq in range(count):
                msg = self.master.recv_match(
                    type=["MISSION_REQUEST_INT", "MISSION_REQUEST"],
                    blocking=True,
                    timeout=5,
                )
                if msg is None:
                    print("Timed out waiting for MISSION_REQUEST_INT / MISSION_REQUEST")
                    return False

                wp = waypoints[seq]
                print(f"Sending MISSION_ITEM_INT for seq {seq}")
                mission_type = mavlink2.MAV_MISSION_TYPE_MISSION

                self.master.mav.mission_item_int_send(
                    self.master.target_system,
                    self.master.target_component,
                    seq,
                    wp.frame,
                    wp.command,
                    wp.is_current,
                    wp.autocontinue,
                    wp.param1,
                    wp.param2,
                    wp.param3,
                    wp.param4,
                    int(wp.x_lat * 1e7),
                    int(wp.y_long * 1e7),
                    float(wp.z_alt),
                    mission_type,
                )

            ret = self.mission_ack()
            if ret == mavlink2.MAV_MISSION_ACCEPTED:
                print("Mission upload successful")
                return True
            else:
                print(f"Mission upload failed, ACK type: {ret}")
                return False

        except Exception as e:
            print(f"Error during mission upload: {e}")
            return False

    @check_mavlink_connection
    def mission_ack(self) -> int:
        # https://mavlink.io/en/messages/common.html#MISSION_ACK
        ack = self.master.recv_match(type="MISSION_ACK", blocking=True, timeout=5)
        ret = -1
        if ack is not None:
            ret = ack.type
        return ret

    @check_mavlink_connection
    def command_ack(self) -> int:
        ack = self.master.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
        ret = -1
        if ack is not None:
            ret = ack.result
        return ret

    @check_mavlink_connection
    def get_uptime(self):
        return int(self.master.uptime)

    def disconnect(self):
        if self.master is not None:
            try:
                self.master.close()
                print(f"Disconnected from MAVLink device {self.device}")
            except Exception as e:
                print(f"Error during MAVLink disconnect: {e}")
            finally:
                self.master = None
        else:
            print(f"MAVLink Device Already disconnected {self.device}")
