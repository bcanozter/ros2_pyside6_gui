import sys
from pathlib import Path
import threading
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_default

from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.impl.rcutils_logger import RcutilsLogger

# PySide6
from PySide6.QtQml import QQmlApplicationEngine
from PySide6.QtGui import QGuiApplication
from PySide6.QtCore import QCoreApplication, QTimer

from .manager import OSMManager, CustomTextureData  # noqa: F401
from .mavros import MAVROS
#


class GuiNode(Node):
    logger: RcutilsLogger

    def __init__(self):
        super().__init__("pyside6_gui_node")
        self._set_logger(self.get_logger())
        self.init_ros_parameters()
        self._set_logger_level(self.log_level)

        self.setup()
        self._gui_thread = threading.Thread(target=self._gui_setup, daemon=True)
        self._gui_thread.start()

    def _gui_setup(self):
        self.app = QGuiApplication(sys.argv)
        self.engine = QQmlApplicationEngine()
        self.mavros_instance = MAVROS(self, "")
        self.engine.rootContext().setContextProperty("mavrosHandler", self.mavros_instance)
        self.engine.addImportPath(Path(__file__).parent)
        self.engine.loadFromModule("OSMBuildings", "Main")
        if not self.engine.rootObjects():
            self.logger.error("Failed to load QML root objects.")
            QTimer.singleShot(0, self.app.quit)
            self.exit_code = -1

        self.exit_code = QCoreApplication.exec()

    @classmethod
    def _set_logger(cls, logger):
        cls.logger = logger

    @classmethod
    def _set_logger_level(cls, log_level: str):
        log_level = log_level.lower()  # Ensure log level is lowercase
        if log_level == "debug":
            cls.logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)
        elif log_level == "info":
            cls.logger.set_level(rclpy.logging.LoggingSeverity.INFO)
        elif log_level == "warn":
            cls.logger.set_level(rclpy.logging.LoggingSeverity.WARN)
        elif log_level == "error":
            cls.logger.set_level(rclpy.logging.LoggingSeverity.ERROR)
        elif log_level == "fatal":
            cls.logger.set_level(rclpy.logging.LoggingSeverity.FATAL)
        else:
            cls.logger.warn(f"Invalid log level: {log_level}. Defaulting to 'info'")
            cls.logger.set_level(rclpy.logging.LoggingSeverity.INFO)

    def init_ros_parameters(self):
        self.declare_parameter(
            name="log_level",
            value="info",
            descriptor=ParameterDescriptor(
                type=rclpy.Parameter.Type.STRING,
                description="Logging level for the node. Options are: debug, info, warn, error, fatal",
                read_only=True,
            ),
        )
        self.log_level = self.get_parameter("log_level").get_parameter_value().string_value

    def setup_service_clients(self):
        # TODO
        pass

    def parameters_callback(self, parameters: list[rclpy.Parameter]) -> SetParametersResult:
        #TODO
        result = SetParametersResult()
        result.successful = True
        return result

    def setup(self):
        """Sets up subscribers, publishers, etc. to configure the node"""
        # callback for dynamic parameter configuration
        self.add_on_set_parameters_callback(self.parameters_callback)
        self.setup_service_clients()

    def destroy(self) -> None:
        super().destroy_node()
        self.app.quit()
        self._gui_thread.join(timeout=1.0)


def main():
    rclpy.init()
    node = GuiNode()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
