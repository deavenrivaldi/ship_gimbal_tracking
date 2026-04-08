"""
YOLO DETECTION NODE
Package : ship_vision
Subscribes : /ship_camera/image_raw
Publishes  : /target/pixel_center  (geometry_msgs/Point)
             /debug/image           (sensor_msgs/Image)
"""
import sys
print("PYTHON:", sys.executable)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import torch
import numpy as np

# ------- CONFIGURATION -------
MODEL_NAME     = "yolov8n.pt"
CONFIDENCE     = 0.5
TARGET_OBJECTS = ["person"]
SKIP_FRAMES    = 3

# Must match SDF <image> block exactly
CAMERA_WIDTH  = 640
CAMERA_HEIGHT = 480

# Debug display size — purely visual, does not affect detection coordinates
DISPLAY_WIDTH = 960


class YoloDetectionNode(Node):

    def __init__(self):
        super().__init__('yolo_detection_node')

        # --- GPU setup ---
        if torch.cuda.is_available():
            self.get_logger().info(
                f'🚀 GPU detected: {torch.cuda.get_device_name(0)} — using CUDA')
            self.device = 'cuda'
        else:
            self.get_logger().warn('⚠️  No GPU found — falling back to CPU')
            self.device = 'cpu'

        self.get_logger().info('🔄 Loading YOLOv8 model...')
        self.model = YOLO(MODEL_NAME)
        self.model.to(self.device)   # move model to GPU if available

        self.bridge        = CvBridge()
        self.frame_count   = 0
        self.latest_boxes  = []
        self.latest_locked = None

        # Subscriber — raw camera from Gazebo
        self.sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publisher 1 — target pixel center → pixel_to_angle node
        self.pub_target = self.create_publisher(
            Point,
            '/target/pixel_center',
            10
        )

        # Publisher 2 — annotated debug image → Foxglove
        self.pub_debug = self.create_publisher(
            Image,
            '/debug/image',
            10
        )

        self.get_logger().info(
            f'✅ YoloDetectionNode ready! '
            f'[device={self.device}] '
            f'[targets={TARGET_OBJECTS}]'
        )


    def image_callback(self, msg):
        self.frame_count += 1

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        # Run YOLO on raw resolution every N frames
        if self.frame_count % SKIP_FRAMES == 0:
            self.latest_boxes  = self.run_detection(frame)
            self.latest_locked = self.pick_locked_target(self.latest_boxes)

        # Publish target pixel center
        if self.latest_locked:
            cx = (self.latest_locked['x1'] + self.latest_locked['x2']) // 2
            cy = (self.latest_locked['y1'] + self.latest_locked['y2']) // 2
            pt = Point(x=float(cx), y=float(cy), z=0.0)
            self.pub_target.publish(pt)
            self.get_logger().info(
                f'🎯 Target pixel: ({cx}, {cy})',
                throttle_duration_sec=1.0
            )

        # Draw + publish debug image
        annotated = self.draw_detections(frame.copy(),
                                         self.latest_boxes,
                                         self.latest_locked)
        display = self.resize_for_display(annotated, DISPLAY_WIDTH)

        try:
            debug_msg        = self.bridge.cv2_to_imgmsg(display, encoding='bgr8')
            debug_msg.header = msg.header
            self.pub_debug.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f'debug publish error: {e}')


    def resize_for_display(self, frame, display_width):
        """Resize for visual output only — does not affect detection."""
        h, w = frame.shape[:2]
        scale = display_width / w
        return cv2.resize(frame, (display_width, int(h * scale)))


    def run_detection(self, frame):
        results = self.model(
            frame,
            conf=CONFIDENCE,
            device=self.device,
            verbose=False
        )
        result  = results[0]
        targets = [t.lower() for t in TARGET_OBJECTS]
        matched = []

        for box in result.boxes:
            class_id = int(box.cls[0])
            label    = result.names[class_id].lower()
            if label in targets:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                matched.append({
                    'x1': x1, 'y1': y1,
                    'x2': x2, 'y2': y2,
                    'confidence': float(box.conf[0]),
                    'label': result.names[class_id]
                })
        return matched


    def pick_locked_target(self, matched_boxes):
        """Lock onto largest box — assumed to be closest target."""
        if not matched_boxes:
            return None
        return max(matched_boxes,
                   key=lambda b: (b['x2'] - b['x1']) * (b['y2'] - b['y1']))


    def draw_detections(self, frame, matched_boxes, locked_target):
        frame_h, frame_w = frame.shape[:2]
        cx = frame_w // 2
        cy = frame_h // 2

        # All matched boxes
        for box in matched_boxes:
            x1, y1, x2, y2 = box['x1'], box['y1'], box['x2'], box['y2']
            conf  = box['confidence']
            label = box['label']

            color = (0, 255, 0)   if conf >= 0.75 else \
                    (0, 165, 255) if conf >= 0.50 else \
                    (0, 0, 255)

            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            text = f"{label} {conf:.0%}"
            (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(frame, (x1, y1 - th - 10), (x1 + tw, y1), color, -1)
            cv2.putText(frame, text, (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

        # Locked target highlight
        if locked_target:
            x1, y1 = locked_target['x1'], locked_target['y1']
            x2, y2 = locked_target['x2'], locked_target['y2']
            tcx = (x1 + x2) // 2
            tcy = (y1 + y2) // 2

            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 0), 3)
            cv2.circle(frame, (tcx, tcy), 5, (255, 255, 0), -1)
            cv2.line(frame, (cx, cy), (tcx, tcy), (255, 255, 0), 1, cv2.LINE_AA)

            offset_x = tcx - cx
            offset_y = tcy - cy
            h_text = f"{'RIGHT' if offset_x > 0 else 'LEFT'} {abs(offset_x)}px"
            v_text = f"{'DOWN'  if offset_y > 0 else 'UP'}   {abs(offset_y)}px"
            cv2.putText(frame, h_text, (10, frame_h - 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0), 2)
            cv2.putText(frame, v_text, (10, frame_h - 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0), 2)

        # Crosshair
        cv2.line(frame,   (cx - 20, cy), (cx + 20, cy), (255, 255, 255), 2)
        cv2.line(frame,   (cx, cy - 20), (cx, cy + 20), (255, 255, 255), 2)
        cv2.circle(frame, (cx, cy), 20,  (255, 255, 255), 2)

        return frame


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()