import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3
import matplotlib.pyplot as plt
from collections import deque
import time


class PlotDebugNode(Node):

    def __init__(self):
        super().__init__('plot_debug_node')

        self.max_len = 100

        # Unified buffers
        self.time_data = deque(maxlen=self.max_len)
        self.pixel_x = deque(maxlen=self.max_len)
        self.pixel_y = deque(maxlen=self.max_len)
        self.angle_x = deque(maxlen=self.max_len)
        self.angle_y = deque(maxlen=self.max_len)

        # Latest values (key fix)
        self.latest_pixel = None
        self.latest_angle = None
        
        # Timestamps
        self.last_pixel_time = None
        self.last_angle_time = None

        # Subscribers
        self.create_subscription(
            Point,
            '/target/pixel_center',
            self.pixel_callback,
            10
        )

        self.create_subscription(
            Vector3,
            '/gimbal/angle_command',
            self.angle_callback,
            10
        )

        # Plot setup
        plt.ion()
        self.fig, self.ax = plt.subplots(2, 1, figsize=(8, 6))

        self.get_logger().info("📊 Plot Debug Node started")


    def pixel_callback(self, msg):
        center_x = 640 / 2
        center_y = 480 / 2
        
        
        self.latest_pixel = (msg.x - center_x, msg.y - center_y)
        
        #self.last_pixel_time = self.get_clock().now()
        
        #self.pixel_x.append(msg.x - center_x)
        #self.pixel_y.append(msg.y - center_y)
        
        self.try_update()


    def angle_callback(self, msg):
        self.latest_angle = (msg.x, msg.y)
        #self.last_angle_time = self.get_clock().now()
        
        #self.angle_x.append(msg.x)
        #self.angle_y.append(msg.y)
        
        self.try_update()


    def try_update(self):
        # Only update when both exist
        if self.latest_pixel is None or self.latest_angle is None:
            return

        t = time.time()

        # Append synchronized data
        self.time_data.append(t)
        self.pixel_x.append(self.latest_pixel[0])
        self.pixel_y.append(self.latest_pixel[1])
        self.angle_x.append(self.latest_angle[0])
        self.angle_y.append(self.latest_angle[1])

        self.update_plot()
        
    def try_update_plot(self):
        if self.last_pixel_time is None or self.last_angle_time is None:
            return

        dt = abs((self.last_pixel_time - self.last_angle_time).nanoseconds) / 1e9

        if dt < 0.05:   # 50ms tolerance
            self.update_plot()


    def update_plot(self):
        if len(self.time_data) < 2:
            return

        t0 = self.time_data[0]
        t = [ti - t0 for ti in self.time_data]

        self.ax[0].cla()
        self.ax[1].cla()

        # Pixel plot
        self.ax[0].plot(t, self.pixel_x, label='Pixel X')
        self.ax[0].plot(t, self.pixel_y, label='Pixel Y')
        self.ax[0].set_title("Pixel Position")
        self.ax[0].set_ylabel("Pixel Offset (centered)")
        self.ax[0].set_ylim(-350, 350)
        self.ax[0].legend()
        self.ax[0].grid()

        # Angle plot
        self.ax[1].plot(t, self.angle_x, label='Angle X (Pan)')
        self.ax[1].plot(t, self.angle_y, label='Angle Y (Tilt)')
        self.ax[1].set_title("Gimbal Angles")
        self.ax[1].set_ylabel("Angle (degrees)")
        self.ax[1].set_ylim(-40, 40)
        self.ax[1].legend()
        self.ax[1].grid()

        plt.pause(0.01)


def main(args=None):
    rclpy.init(args=args)
    node = PlotDebugNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()