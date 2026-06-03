#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from ship_msgs.srv import Fire

class HitTester(Node):
    def __init__(self):
        super().__init__('hit_tester')
        
        # 🌟 拔除連字號，並確保使用的是 person_link
        self.hit_sub = self.create_subscription(
            Contacts, 
            '/world/gimbal_world/model/wamv/link/person_link/sensor/person_contact/contact',
            self.hit_callback, 
            10
        )
        
        # 2. 建立開火服務的 Client
        self.fire_client = self.create_client(Fire, '/fire')
        
        # 3. 設定定時器，每 2 秒自動開火一次
        self.timer = self.create_timer(2.0, self.auto_fire)
        
        self.get_logger().info("🎯 [測試儀器就緒] 痛覺監聽中！每 2 秒自動射擊...")

    def hit_callback(self, msg):
        # 🌟 加上 's'
        self.get_logger().info(f"⚡ 收到觸覺訊號！共 {len(msg.contacts)} 個接觸點")
        
        # 🌟 加上 's'
        for contact in msg.contacts:
            col1 = contact.collision1.name
            col2 = contact.collision2.name
            
            self.get_logger().info(f"   🥊 碰撞物 A: {col1}")
            self.get_logger().info(f"   🥊 碰撞物 B: {col2}")
            
            if 'projectile' in col1 or 'projectile' in col2:
                self.get_logger().info("\n========================================")
                self.get_logger().info(" 💥💥💥 命中確認！感測器完美運作中！ 💥💥💥")
                self.get_logger().info("========================================\n")
                break

    def auto_fire(self):
        if self.fire_client.wait_for_service(timeout_sec=0.1):
            req = Fire.Request()
            req.force = 15000.0
            self.fire_client.call_async(req)
            self.get_logger().info("🔥 發射測試彈...")
        else:
            self.get_logger().warn("等待 /fire 服務中，請確認 fire_node 已啟動。")

def main(args=None):
    rclpy.init(args=args)
    node = HitTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()