import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SpawnEntity
from projectile_msgs.srv import Fire
from ament_index_python.packages import get_package_share_directory
from ros_gz_interfaces.msg import EntityWrench
import os


class GzInterface(Node):

    def __init__(self):
        super().__init__('gz_interface')

        # ROS2 parameters
        self.declare_parameter('world_name', 'world_test')

        self.world_name = self.get_parameter('world_name').value


        # Service server (NEW)
        self.fire_service = self.create_service(
            Fire,
            '/fire',
            self.fire_callback
        )

        # Gazebo spawn client
        self.spawn_client = self.create_client(
            SpawnEntity,
            f'/world/{self.world_name}/create'
        )

        # Wrench publisher
        self.wrench_publisher = self.create_publisher(
            EntityWrench,
            f'/world/{self.world_name}/wrench',
            10
        )

        self.pending_projectiles = {}

    # ---------------- FIRE SERVICE ----------------
    def fire_callback(self, request, response):

        self.get_logger().info("Fire request received")

        package_path = get_package_share_directory('projectile_gz')

        sdf_path = os.path.join(
            package_path,
            'models',
            'projectile_sphere.sdf'
        )

        name = f"projectile_{self.get_clock().now().nanoseconds}"

        spawn_request = SpawnEntity.Request()

        spawn_request.entity_factory.sdf_filename = sdf_path
        spawn_request.entity_factory.name = name

        spawn_request.entity_factory.pose.position = request.position

        future = self.spawn_client.call_async(spawn_request)

        self.pending_projectiles[future] = {
            "name": name,
            "direction": request.direction,
            "force": request.force
        }

        future.add_done_callback(self.spawn_response_callback)

        response.success = True
        response.projectile_name = name
        return response

    # ---------------- SPAWN CALLBACK ----------------
    def spawn_response_callback(self, future):

        result = future.result()
        context = self.pending_projectiles.pop(future, None)

        if not result.success or context is None:
            self.get_logger().error("Spawn failed")
            return

        name = context["name"]
        direction = context["direction"]
        force = context["force"]

        self.get_logger().info(f"Spawn OK: {name}")

        wrench_msg = EntityWrench()
        wrench_msg.entity.name = name
        wrench_msg.entity.type = 2  # MODEL

        wrench_msg.wrench.force.x = direction.x * force
        wrench_msg.wrench.force.y = direction.y * force
        wrench_msg.wrench.force.z = direction.z * force

        self.wrench_publisher.publish(wrench_msg)

        self.get_logger().info(
            f"Projectile {name} fired"
        )


def main(args=None):
    rclpy.init(args=args)

    node = GzInterface()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()