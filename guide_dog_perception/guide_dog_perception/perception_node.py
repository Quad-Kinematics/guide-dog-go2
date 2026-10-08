import os

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from guide_dog_interfaces.msg import DetectedFace
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import onnxruntime as ort

CONF_THRESHOLD = 0.50  # 50% confidence threshold
PERSON_CLASS = 0       # Class 0 in the COCO dataset is 'Person'


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        self.bridge = CvBridge()

        # Default: the ONNX copy installed with this package, so the node runs
        # from any directory. Missing file = error instead of falling back to
        # anything that downloads weights.
        model_path = self.declare_parameter(
            'model_path',
            os.path.join(get_package_share_directory('guide_dog_perception'),
                         'models', 'yolov8n_384x640.onnx')).value
        if not model_path.endswith('.onnx'):
            raise ValueError(f"model_path must be an ONNX export: {model_path}")
        if not os.path.isfile(model_path):
            raise FileNotFoundError(f"YOLO model not found: {model_path}")

        # Inference runs on the CPU (no CUDA on the Jetson). ONNX Runtime takes
        # ~140 ms/frame with 2 threads while Nav2 runs and leaves the other
        # cores to Nav2 (torch/ultralytics took 0.7-1.4 s/frame).
        num_threads = self.declare_parameter('num_threads', 2).value

        self.get_logger().info(f"Loading YOLOv8 Nano model from {model_path}...")
        opts = ort.SessionOptions()
        if num_threads > 0:
            opts.intra_op_num_threads = num_threads
        opts.inter_op_num_threads = 1
        self.session = ort.InferenceSession(
            model_path, opts, providers=['CPUExecutionProvider'])
        self.input_name = self.session.get_inputs()[0].name
        # Fixed input size (H, W), set when the model was exported
        self.input_hw = tuple(self.session.get_inputs()[0].shape[2:])

        # Depth 1: inference is slower than the 30 fps camera, so always take
        # the newest frame. A deeper queue makes ALIGN steer on stale offsets.
        self.image_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            1
        )

        self.face_pub = self.create_publisher(
            DetectedFace,
            '/detected_face',
            10
        )

        self.get_logger().info("Perception Node online. Waiting for frames...")

    def detect_person(self, image):
        """Return (confidence, box center x) of the most confident person, or None."""
        # Letterbox to the model input, like ultralytics does
        in_h, in_w = self.input_hw
        h, w = image.shape[:2]
        scale = min(in_h / h, in_w / w)
        new_h, new_w = round(h * scale), round(w * scale)
        top, left = (in_h - new_h) // 2, (in_w - new_w) // 2
        padded = cv2.copyMakeBorder(
            cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR),
            top, in_h - new_h - top, left, in_w - new_w - left,
            cv2.BORDER_CONSTANT, value=(114, 114, 114))
        blob = cv2.dnn.blobFromImage(padded, 1.0 / 255.0, swapRB=True)

        # Output (1, 84, N): rows 0-3 box cx, cy, w, h in input pixels, then
        # one score row per COCO class. The best person box is the same with
        # or without NMS, so it is skipped.
        pred = self.session.run(None, {self.input_name: blob})[0][0]
        scores = pred[4 + PERSON_CLASS]
        best = int(np.argmax(scores))
        if scores[best] <= CONF_THRESHOLD:
            return None
        return float(scores[best]), (float(pred[0, best]) - left) / scale

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        image_center_x = cv_image.shape[1] / 2.0

        person = self.detect_person(cv_image)
        if person is None:
            return
        best_conf, box_center_x = person

        # Normalized offset (-1.0 to 1.0), positive = right of center
        offset_x = (box_center_x - image_center_x) / image_center_x

        # Publish the data to the FSM
        msg = DetectedFace()
        # The image's capture stamp: ALIGN turns by where the robot pointed
        # when the frame was taken, not when this message arrives
        msg.header = data.header
        msg.name = "Person"  # For V1, we just target any human
        msg.confidence = best_conf
        msg.center_offset_x = float(offset_x)
        msg.center_offset_y = 0.0

        self.face_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
