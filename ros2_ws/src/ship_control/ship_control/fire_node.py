"""
FIRE NODE
Package    : ship_control
Service    : /fire  (ship_msgs/srv/Fire)
Spawns a projectile at camera world position and applies force in camera direction.
"""

import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, Point
from tf2_ros import Buffer, TransformListener
from ros_gz_interfaces.srv import SpawnEntity
from ros_gz_interfaces.msg import EntityWrench, Entity
from ship_msgs.srv import Fire
from ament_index_python.packages import get_package_share_directory


import time  # 🌟 1. 補上 time 模組！
import subprocess  # 🌟 1. 補上這個模組，讓我們可以直接呼叫系統終端機
# 🌟 2. 補上 DeleteEntity！
from ros_gz_interfaces.srv import SpawnEntity, DeleteEntity

class FireNode(Node):

    def __init__(self):
        super().__init__('fire_node')

        self.declare_parameter('world_name', 'gimbal_world')
        self.declare_parameter('projectile_package', 'ship_simulation')
        self.declare_parameter('projectile_sdf', 'models/bullet/projectile_sphere.sdf')

        self.world_name = self.get_parameter('world_name').value
        self.projectile_path = os.path.join(
            get_package_share_directory(self.get_parameter('projectile_package').value),
            self.get_parameter('projectile_sdf').value
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.fire_service = self.create_service(Fire, '/fire', self.fire_callback)

        self.spawn_client = self.create_client(SpawnEntity, f'/world/{self.world_name}/create')
        if not self.spawn_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warning('SpawnEntity service not available')

        self.wrench_publisher = self.create_publisher(EntityWrench, f'/world/{self.world_name}/wrench', 10)
        self.pending_projectiles = {}

        # ==========================================
        # 🌟 核心新增：刪除實體的 Client 與自動清理計時器
        # ==========================================
        self.delete_client = self.create_client(DeleteEntity, f'/world/{self.world_name}/remove')
        self.active_projectiles = []  # 用來記錄已經發射出去的子彈
        # ==========================================
        # 🌟 核心新增：彈匣機制與刪除 Client
        # ==========================================
        self.delete_client = self.create_client(DeleteEntity, f'/world/{self.world_name}/remove')
        
        self.max_ammo = 3             # 🔫 彈匣容量：世界最多只允許 3 顆子彈同時存在！
        self.active_projectiles = []  # 記錄發射出去的子彈名稱 (單純存字串即可)
        
        # 🗑️ 把原本的 cleanup_timer 刪掉或註解掉！我們不需要計時器了！
        # self.cleanup_timer = self.create_timer(1.0, self.cleanup_projectiles)
        #self.cleanup_timer = self.create_timer(1.0, self.cleanup_projectiles) # 每秒檢查一次


        self.get_logger().info('✅ FireNode ready! Service: /fire')


    def fire_callback(self, request, response):
        name = self.shoot_projectile(request.position, request.direction, request.force)
        response.success = name is not None
        response.projectile_name = name or ''
        return response


    def shoot_projectile(self, position: Point, direction: Vector3, force: float):
        if not os.path.exists(self.projectile_path):
            self.get_logger().error(f'SDF not found: {self.projectile_path}')
            return None

        name = f'projectile_{self.get_clock().now().nanoseconds}'
        req = SpawnEntity.Request()
        req.entity_factory.sdf_filename = self.projectile_path
        req.entity_factory.name = name
        
        # 我們將在這裡計算出「真正的」相機方向
        real_direction = Vector3()
        real_direction.x = direction.x
        real_direction.y = direction.y
        real_direction.z = direction.z

        try:
            from rclpy.duration import Duration
            import math
            
            # 取得相機座標
            transform = self.tf_buffer.lookup_transform(
                'gimbal_world', 
                'gimbal_wam_v/gimbal/camera_link',
                rclpy.time.Time(nanoseconds=0),
                timeout=Duration(seconds=0.1)
            )
            
            cam_x = transform.transform.translation.x
            cam_y = transform.transform.translation.y
            cam_z = transform.transform.translation.z
            
            # ==========================================
            # 🎯 核心修正：直接從相機的 Quaternion 算出它的前方
            # 在 ROS 的標準預設中，X 軸是前方。
            # 如果還是不對，你可以嘗試改成相機標準的 Z 軸前方。
            # ==========================================
            q = transform.transform.rotation
            
            # 假設相機的前方是 X 軸 (標準 ROS 坐標系)
            cam_forward_x = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            cam_forward_y = 2.0 * (q.x * q.y + q.w * q.z)
            cam_forward_z = 2.0 * (q.x * q.z - q.w * q.y)
            
            # 更新真正的發射方向
            real_direction.x = float(cam_forward_x)
            real_direction.y = float(cam_forward_y)
            real_direction.z = float(cam_forward_z)
            
            spawn_distance = 1
            height_offset = 0.2
            
            req.entity_factory.pose.position = Point(
                x=cam_x + (real_direction.x * spawn_distance),
                y=cam_y + (real_direction.y * spawn_distance),
                z=cam_z + (real_direction.z * spawn_distance) + height_offset
            )
            
            self.get_logger().info(f"🎯 發射點：X={cam_x:.1f}, Y={cam_y:.1f}, Z={cam_z:.1f} | 方向：[{real_direction.x:.2f}, {real_direction.y:.2f}, {real_direction.z:.2f}]")
            
        except Exception as e:
            self.get_logger().warn(f"⚠️ TF2 查詢失敗，退回使用船體預備座標發射: {e}")
            req.entity_factory.pose.position = position

        # 設定初始姿態防呆
        req.entity_factory.pose.orientation.w = 1.0
        req.entity_factory.pose.orientation.x = 0.0
        req.entity_factory.pose.orientation.y = 0.0
        req.entity_factory.pose.orientation.z = 0.0

        future = self.spawn_client.call_async(req)
        # 🌟 注意：這裡要傳入我們剛算好的 real_direction，確保受力方向跟鏡頭一致
        self.pending_projectiles[future] = {'name': name, 'direction': real_direction, 'force': force}
        future.add_done_callback(self.spawn_response_callback)
        return name


    def spawn_response_callback(self, future):
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f'Spawn failed: {exc}')
            return

        context = self.pending_projectiles.pop(future, None)
        if context is None or not result.success:
            self.get_logger().error('Spawn failed')
            return

        wrench_msg = EntityWrench()
        wrench_msg.entity.name = context['name']
        wrench_msg.entity.type = Entity.MODEL
        wrench_msg.wrench.force.x = context['direction'].x * context['force']
        wrench_msg.wrench.force.y = context['direction'].y * context['force']
        wrench_msg.wrench.force.z = context['direction'].z * context['force']
        self.wrench_publisher.publish(wrench_msg)
        self.get_logger().info(f'Projectile {context["name"]} fired')

        
        # ==========================================
        # 🌟 彈匣管理：把新子彈裝入彈匣
        # ==========================================
        self.active_projectiles.append(context['name'])

        # 只要世界上的子彈數量超過彈匣容量 (3 顆)，就立刻把最前面 (最舊) 的子彈刪掉！
        while len(self.active_projectiles) > self.max_ammo:
            oldest_name = self.active_projectiles.pop(0) # 拿出最舊的
            self.remove_oldest_projectile(oldest_name)   # 毀滅它


    def remove_oldest_projectile(self, projectile_name):
        """呼叫系統指令強制刪除指定的子彈"""
        remove_cmd = (
            f"gz service -s /world/{self.world_name}/remove "
            f"--reqtype gz.msgs.Entity --reptype gz.msgs.Boolean "
            f"--req 'name: \"{projectile_name}\", type: MODEL'"
        )
        subprocess.Popen(remove_cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        # self.get_logger().info(f"🗑️ 彈匣已滿，強制回收舊子彈: {projectile_name}")



def main(args=None):
    rclpy.init(args=args)
    node = FireNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()