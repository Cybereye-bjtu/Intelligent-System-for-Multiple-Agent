import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


class FleetSafetyManager(Node):
    """Persist the fleet emergency-stop state; startup is always fail-closed."""

    def __init__(self):
        super().__init__("fleet_safety_manager")
        self.declare_parameter("default_emergency_stop", True)
        self.stopped = bool(self.get_parameter("default_emergency_stop").value)
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(Bool, "/swarm/emergency_stop", qos)
        self.create_service(SetBool, "/swarm/set_emergency_stop", self._set_stop)
        self.create_timer(0.2, self._publish)
        self._publish()

    def _set_stop(self, request, response):
        self.stopped = bool(request.data)
        self._publish()
        response.success = True
        response.message = "fleet emergency stop active" if self.stopped else "fleet emergency stop released"
        self.get_logger().warning(response.message)
        return response

    def _publish(self):
        message = Bool()
        message.data = self.stopped
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = FleetSafetyManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
