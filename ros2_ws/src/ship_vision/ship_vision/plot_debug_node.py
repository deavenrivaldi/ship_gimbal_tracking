import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3

import matplotlib
matplotlib.use('TkAgg')  # Use TkAgg backend for interactive plotting
import matplotlib.pyplot as plt

from collections import deque
import time


class PlotDebugNode(Node):
    def __init__(self):
            super().__init__('plot_debug_node')

            self.max_len = 100
            self.time_data = deque(maxlen=self.max_len)
            self.pixel_x = deque(maxlen=self.max_len)
            self.pixel_y = deque(maxlen=self.max_len)
            self.angle_x = deque(maxlen=self.max_len)
            self.angle_y = deque(maxlen=self.max_len)

            self.latest_pixel = None
            self.latest_angle = None

            # Subscribers
            self.create_subscription(Point, '/target/pixel_center', self.pixel_callback, 10)
            self.create_subscription(Vector3, '/gimbal/angle_command', self.angle_callback, 10)

            # Plot setup - Move this into a dedicated timer or main loop
            plt.ion()
            self.fig, self.ax = plt.subplots(2, 1, figsize=(8, 6))
            self.t0 = time.time()

            # CREATE A PLOT TIMER (Run at 10Hz - much more stable)
            self.plot_timer = self.create_timer(0.1, self.update_plot)

            self.get_logger().info("📊 Plot Debug Node started")

    def pixel_callback(self, msg):
        center_x, center_y = 640 / 2, 480 / 2
        self.latest_pixel = (msg.x - center_x, -(msg.y - center_y))

    def angle_callback(self, msg):
        self.latest_angle = (msg.x, -msg.y)

    def update_plot(self):
        # 1. Sync check
        if self.latest_pixel is None or self.latest_angle is None:
            return

        # 2. Update deques in the timer, not the callback
        self.time_data.append(time.time() - self.t0)
        self.pixel_x.append(self.latest_pixel[0])
        self.pixel_y.append(self.latest_pixel[1])
        self.angle_x.append(self.latest_angle[0])
        self.angle_y.append(self.latest_angle[1])

        if len(self.time_data) < 2:
            return

        # 3. Efficient Plotting
        self.ax[0].cla()
        self.ax[1].cla()

        # Plotting logic
        self.ax[0].plot(list(self.time_data), list(self.pixel_x), label='Pixel X', color='blue')
        self.ax[0].plot(list(self.time_data), list(self.pixel_y), label='Pixel Y', color='green')
        self.ax[0].set_ylim(-350, 350)
        self.ax[0].legend(loc='upper right')
        self.ax[0].grid(True)

        self.ax[1].plot(list(self.time_data), list(self.angle_x), label='Pan', color='red')
        self.ax[1].plot(list(self.time_data), list(self.angle_y), label='Tilt', color='yellow')
        self.ax[1].set_ylim(-40, 40)
        self.ax[1].legend(loc='upper right')
        self.ax[1].grid(True)

        # Draw the plot
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()



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