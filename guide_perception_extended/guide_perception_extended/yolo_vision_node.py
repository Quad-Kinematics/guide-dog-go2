#!/usr/bin/env python3
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO


class YoloVisionNode(Node):
    """Detects objects in each camera frame and publishes them on /yolo_targets."""

    def __init__(self):
        super().__init__('yolo_vision_node')

        # Parameters: settings you can change without editing the code.
        self.declare_parameter('camera_topic', '/camera/color/image_raw')
        self.declare_parameter('output_topic', '/yolo_targets')
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('process_every_n_frames', 3)

        # Read those settings and keep them on the node.
        self.camera_topic = self.get_parameter('camera_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.model_path = self.get_parameter('model_path').value
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        self.process_every_n_frames = max(1, int(self.get_parameter('process_every_n_frames').value))

        # Tools we reuse on every frame.
        self.bridge = CvBridge()
        self._frame_count = 0

        # Load the YOLO model once at startup (downloaded automatically on first run).
        self.get_logger().info(f'Loading YOLO model: {self.model_path} ...')
        self.model = YOLO(self.model_path)
        self.get_logger().info('YOLO model ready.')

        # Camera delivery rules: keep only the newest frame, lossy (matches the camera).
        camera_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Out-slot: publish detections. In-slot: receive camera frames.
        self.targets_pub = self.create_publisher(String, self.output_topic, 10)
        self.image_sub = self.create_subscription(
            Image, self.camera_topic, self.image_callback, camera_qos
        )

        self.get_logger().info(
            f'yolo_vision_node ready: camera_topic={self.camera_topic}, '
            f'output_topic={self.output_topic}'
        )

    def image_callback(self, msg):
        self._frame_count += 1
        if self._frame_count % self.process_every_n_frames != 0:
            return

        # Turn the ROS image message into an OpenCV image the model can read.
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as error:
            self.get_logger().warn(f'Could not read the camera image: {error}')
            return

        # Run the YOLO model on the image.
        results = self.model.predict(frame, conf=self.confidence_threshold, verbose=False)

        # Go through each detected object and put it into a simple list.
        detections = []
        for box in results[0].boxes:
            class_id = int(box.cls[0])
            label = self.model.names[class_id]
            confidence = float(box.conf[0])
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            one_detection = {
                'label': label,
                'confidence': round(confidence, 3),
                'bbox': [x1, y1, x2, y2],
            }
            detections.append(one_detection)

        # Package the list as JSON text and publish it on /yolo_targets.
        message = String()
        message.data = json.dumps({'detections': detections})
        self.targets_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = YoloVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
