import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener


def approach_goal(robot_x, robot_y, target_x, target_y, standoff):
    dx = float(target_x) - float(robot_x)
    dy = float(target_y) - float(robot_y)
    distance = math.hypot(dx, dy)
    if distance <= float(standoff):
        raise ValueError("shared target is inside the standoff distance")
    travel = distance - float(standoff)
    scale = travel / distance
    return (
        float(robot_x) + dx * scale,
        float(robot_y) + dy * scale,
        math.atan2(dy, dx),
    )


class DualTargetCoordinator(Node):
    """Trigger both detectors and share a site-frame target after one fails."""

    def __init__(self):
        super().__init__("dual_target_coordinator")
        self.declare_parameter("robot_names", ["hyzx001", "jetson003"])
        self.declare_parameter(
            "base_frames", ["hyzx001/base_footprint", "jetson003/base_link"]
        )
        self.declare_parameter("global_frame", "site_map")
        self.declare_parameter("detection_timeout", 120.0)
        self.declare_parameter("standoff_distance", 0.30)
        self.robots = [
            str(value).strip("/")
            for value in self.get_parameter("robot_names").value
        ]
        frames = [
            str(value).strip("/")
            for value in self.get_parameter("base_frames").value
        ]
        if len(self.robots) != 2 or len(frames) != 2:
            raise ValueError("dual target coordination requires exactly two robots")
        self.frames = dict(zip(self.robots, frames))
        self.global_frame = str(self.get_parameter("global_frame").value)
        self.detector_clients = {
            robot: self.create_client(
                Trigger, f"/{robot}/target_navigator/detect_and_go"
            )
            for robot in self.robots
        }
        self.goal_publishers = {
            robot: self.create_publisher(PoseStamped, f"/{robot}/goal_pose", 10)
            for robot in self.robots
        }
        self.shared_target_publisher = self.create_publisher(
            PoseStamped, "/swarm/shared_target", 10
        )
        self.status_publisher = self.create_publisher(
            String, "/swarm/target_mission/status", 10
        )
        self._subscriptions = []
        for robot in self.robots:
            self._subscriptions.append(
                self.create_subscription(
                    PoseStamped,
                    f"/{robot}/target_pose",
                    lambda message, name=robot: self._target(name, message),
                    10,
                )
            )
            self._subscriptions.append(
                self.create_subscription(
                    String,
                    f"/{robot}/target_navigation/status",
                    lambda message, name=robot: self._detector_status(name, message),
                    10,
                )
            )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.active = False
        self.started_at = None
        self.deadline = None
        self.targets = {}
        self.failures = {}
        self.create_service(Trigger, "~/detect_and_plan", self._trigger)
        self.create_timer(0.2, self._evaluate, clock=self.steady_clock)

    def _publish_status(self, text):
        self.status_publisher.publish(String(data=text))
        self.get_logger().info(text)

    def _trigger(self, request, response):
        del request
        if self.active:
            response.success = False
            response.message = "dual target mission is already running"
            return response
        unavailable = [
            robot for robot, client in self.detector_clients.items()
            if not client.service_is_ready()
        ]
        if unavailable:
            response.success = False
            response.message = "detector services unavailable: " + ",".join(unavailable)
            return response
        self.active = True
        self.targets.clear()
        self.failures.clear()
        self.started_at = self.get_clock().now()
        self.deadline = self.steady_clock.now() + Duration(
            seconds=float(self.get_parameter("detection_timeout").value)
        )
        for robot, client in self.detector_clients.items():
            future = client.call_async(Trigger.Request())
            future.add_done_callback(
                lambda result, name=robot: self._trigger_response(name, result)
            )
        self._publish_status("dual detection started")
        response.success = True
        response.message = "both target detectors started"
        return response

    def _trigger_response(self, robot, future):
        try:
            result = future.result()
            if not result.success:
                self.failures[robot] = result.message
        except Exception as error:  # rclpy client failures are runtime errors
            self.failures[robot] = str(error)

    def _target(self, robot, message):
        if not self.active or message.header.frame_id != self.global_frame:
            return
        stamp = Time.from_msg(message.header.stamp)
        if stamp.nanoseconds <= 0 or stamp < self.started_at:
            return
        self.targets[robot] = message
        self.failures.pop(robot, None)

    def _detector_status(self, robot, message):
        if self.active and str(message.data).startswith("failed:"):
            self.failures[robot] = str(message.data)

    def _share(self, source, recipient):
        target = self.targets[source]
        transform = self.tf_buffer.lookup_transform(
            self.global_frame,
            self.frames[recipient],
            Time(),
            timeout=Duration(seconds=1.0),
        )
        position = transform.transform.translation
        goal_x, goal_y, yaw = approach_goal(
            position.x,
            position.y,
            target.pose.position.x,
            target.pose.position.y,
            float(self.get_parameter("standoff_distance").value),
        )
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self.global_frame
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        self.goal_publishers[recipient].publish(goal)
        shared = PoseStamped()
        shared.header = goal.header
        shared.pose = target.pose
        self.shared_target_publisher.publish(shared)
        self._finish(
            f"{source} detected the target; coordinates shared with {recipient}"
        )

    def _finish(self, text):
        self._publish_status(text)
        self.active = False

    def _evaluate(self):
        if not self.active:
            return
        if len(self.targets) == 2:
            self._finish("both robots detected the target and planned independently")
            return
        timed_out = self.steady_clock.now() >= self.deadline
        finished = all(
            robot in self.targets or robot in self.failures for robot in self.robots
        )
        if not timed_out and not finished:
            return
        if len(self.targets) == 1:
            source = next(iter(self.targets))
            recipient = next(robot for robot in self.robots if robot != source)
            try:
                self._share(source, recipient)
            except (TransformException, ValueError) as error:
                self._finish(f"target sharing failed: {error}")
            return
        self._finish("dual target mission failed: neither robot detected the target")


def main(args=None):
    rclpy.init(args=args)
    node = DualTargetCoordinator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
