import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .algorithms import (
    astar_grid,
    grid_to_world,
    nearest_traversable,
    resample_polyline,
    simplify_path,
    world_to_grid,
)


class AStarPlanner(Node):
    def __init__(self):
        super().__init__("astar_planner")
        defaults = {
            "costmap_topic": "/esdf_map",
            "goal_topic": "/goal_pose",
            "path_topic": "/global_path",
            "cancel_topic": "/navigation/cancel",
            "map_frame": "map",
            "robot_frame": "base_link",
            "planning_frequency": 2.0,
            "lethal_cost": 96,
            "allow_diagonal": True,
            "clearance_weight": 2.5,
            "turn_weight": 0.1,
            "goal_tolerance": 0.25,
            "max_expansions": 500000,
            "path_resample_spacing": 0.10,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.robot_frame = str(self.get_parameter("robot_frame").value)
        self.lethal_cost = int(self.get_parameter("lethal_cost").value)
        self.allow_diagonal = bool(
            self.get_parameter("allow_diagonal").value
        )
        self.clearance_weight = float(
            self.get_parameter("clearance_weight").value
        )
        self.turn_weight = float(self.get_parameter("turn_weight").value)
        self.max_expansions = int(
            self.get_parameter("max_expansions").value
        )
        self.costmap = None
        self.cost_array = None
        self.goal = None
        self.map_generation = 0
        self.planned_generation = -1

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        path_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.path_publisher = self.create_publisher(
            Path, str(self.get_parameter("path_topic").value), path_qos
        )
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("costmap_topic").value),
            self.map_callback,
            map_qos,
        )
        self.goal_subscription = self.create_subscription(
            PoseStamped,
            str(self.get_parameter("goal_topic").value),
            self.goal_callback,
            10,
        )
        self.cancel_subscription = self.create_subscription(
            String,
            str(self.get_parameter("cancel_topic").value),
            self.cancel_callback,
            10,
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=20.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        frequency = max(
            0.1, float(self.get_parameter("planning_frequency").value)
        )
        self.timer = self.create_timer(1.0 / frequency, self.plan_if_needed)

    def map_callback(self, message: OccupancyGrid):
        expected = int(message.info.width) * int(message.info.height)
        if expected == 0 or len(message.data) != expected:
            return
        self.costmap = message
        self.cost_array = np.asarray(message.data, dtype=np.int16).reshape(
            int(message.info.height), int(message.info.width)
        )
        self.map_generation += 1

    def goal_callback(self, message: PoseStamped):
        frame = message.header.frame_id or self.map_frame
        if frame != self.map_frame:
            self.get_logger().error(
                f"Goal frame must be {self.map_frame}, received {frame}"
            )
            return
        self.goal = message
        self.planned_generation = -1
        self.get_logger().info(
            "New goal: (%.2f, %.2f)"
            % (message.pose.position.x, message.pose.position.y)
        )

    def cancel_callback(self, message: String):
        """Clear both planner intent and the controller's latched path."""
        self.goal = None
        self.planned_generation = self.map_generation
        self._publish_empty()
        detail = message.data.strip() or "fleet stop"
        self.get_logger().warning(f"Navigation cancelled: {detail}")

    def _robot_position(self):
        transform = self.tf_buffer.lookup_transform(
            self.map_frame,
            self.robot_frame,
            Time(),
            timeout=Duration(seconds=0.15),
        )
        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
        )

    def _publish_empty(self):
        message = Path()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.map_frame
        self.path_publisher.publish(message)

    def plan_if_needed(self):
        if (
            self.goal is None
            or self.costmap is None
            or self.planned_generation == self.map_generation
        ):
            return
        try:
            robot_x, robot_y = self._robot_position()
        except TransformException as error:
            self.get_logger().warning(
                f"Waiting for {self.map_frame}->{self.robot_frame}: {error}",
                throttle_duration_sec=2.0,
            )
            return

        info = self.costmap.info
        # Isaac/SLAM maps use an axis-aligned origin. Refuse a rotated map
        # rather than silently planning in the wrong coordinates.
        if abs(info.origin.orientation.z) > 1e-6:
            self.get_logger().error("Rotated OccupancyGrid origins are unsupported")
            return
        start = world_to_grid(
            robot_x,
            robot_y,
            info.origin.position.x,
            info.origin.position.y,
            info.resolution,
        )
        goal = world_to_grid(
            self.goal.pose.position.x,
            self.goal.pose.position.y,
            info.origin.position.x,
            info.origin.position.y,
            info.resolution,
        )
        path_cells = astar_grid(
            self.cost_array,
            start,
            goal,
            lethal_cost=self.lethal_cost,
            allow_diagonal=self.allow_diagonal,
            clearance_weight=self.clearance_weight,
            turn_weight=self.turn_weight,
            max_expansions=self.max_expansions,
        )
        self.planned_generation = self.map_generation
        if not path_cells:
            projected_start = nearest_traversable(
                self.cost_array, start, self.lethal_cost
            )
            projected_goal = nearest_traversable(
                self.cost_array, goal, self.lethal_cost
            )

            def cell_cost(cell):
                if cell is None:
                    return "outside/no traversable cell"
                x, y = cell
                return str(int(self.cost_array[y, x]))

            self.get_logger().warning(
                "A* found no collision-free path: "
                f"start={start}->{projected_start} "
                f"cost={cell_cost(projected_start)}, "
                f"goal={goal}->{projected_goal} "
                f"cost={cell_cost(projected_goal)}, "
                f"map={info.width}x{info.height}"
            )
            self._publish_empty()
            return
        path_cells = simplify_path(
            path_cells, self.cost_array, self.lethal_cost
        )
        points = [
            grid_to_world(
                x,
                y,
                info.origin.position.x,
                info.origin.position.y,
                info.resolution,
            )
            for x, y in path_cells
        ]
        points = resample_polyline(
            points,
            float(self.get_parameter("path_resample_spacing").value),
        )
        output = Path()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = self.map_frame
        for index, (x, y) in enumerate(points):
            pose = PoseStamped()
            pose.header = output.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            if len(points) > 1:
                other = points[min(index + 1, len(points) - 1)]
                if index == len(points) - 1:
                    other = points[index - 1]
                    yaw = math.atan2(y - other[1], x - other[0])
                else:
                    yaw = math.atan2(other[1] - y, other[0] - x)
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
            else:
                pose.pose.orientation.w = 1.0
            output.poses.append(pose)
        self.path_publisher.publish(output)
        self.get_logger().info(
            f"Published global path with {len(output.poses)} waypoints"
        )


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
