import csv
from datetime import datetime
import os
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from guide_dog_interfaces.msg import DetectedFace
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import onnxruntime as ort

CONF_THRESHOLD = 0.80  # 80% confidence threshold
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

        # Per-frame log for debugging missed detections: the best person score
        # even below CONF_THRESHOLD, box, inference time, sharpness (blur) and
        # brightness, one row per processed frame, in the ROS log directory.
        # debug_image_sec > 0 also saves an annotated 640x360 JPEG that often
        # (pictures of people: off by default, delete them after use).
        self.frame_log = None
        self.image_dir = None
        self.debug_image_sec = float(self.declare_parameter('debug_image_sec', 0.0).value)
        self.last_image_save = 0.0
        if self.declare_parameter('frame_log', True).value:
            log_root = os.environ.get('ROS_LOG_DIR') or os.path.join(
                os.environ.get('ROS_HOME') or os.path.expanduser('~/.ros'), 'log')
            log_dir = os.path.join(
                log_root, 'perception_' + datetime.now().strftime('%Y-%m-%d-%H-%M-%S'))
            os.makedirs(log_dir, exist_ok=True)
            self.frame_log_file = open(os.path.join(log_dir, 'frames.csv'), 'w', newline='')
            self.frame_log = csv.writer(self.frame_log_file)
            self.frame_log.writerow([
                'recv_time', 'frame_stamp', 'infer_ms', 'score', 'published',
                'cx', 'cy', 'w', 'h', 'top', 'bottom', 'sharpness', 'brightness'])
            if self.debug_image_sec > 0.0:
                self.image_dir = os.path.join(log_dir, 'images')
                os.makedirs(self.image_dir, exist_ok=True)
            self.get_logger().info(
                f"Logging every frame to {log_dir}"
                + (f" (+ an image every {self.debug_image_sec:.1f} s)" if self.image_dir else ''))

        self.face_pub = self.create_publisher(
            DetectedFace,
            '/detected_face',
            10
        )

        self.get_logger().info("Perception Node online. Waiting for frames...")

    def detect_person(self, image):
        """Return (score, cx, cy, w, h) of the most confident person box, any score."""
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
        return (float(scores[best]),
                (float(pred[0, best]) - left) / scale, (float(pred[1, best]) - top) / scale,
                float(pred[2, best]) / scale, float(pred[3, best]) / scale)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        image_center_x = cv_image.shape[1] / 2.0

        recv_time = self.get_clock().now().nanoseconds * 1e-9
        t0 = time.monotonic()
        best_conf, box_center_x, box_cy, box_w, box_h = self.detect_person(cv_image)
        infer_ms = (time.monotonic() - t0) * 1000.0
        published = best_conf > CONF_THRESHOLD
        if published:
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

        # After publishing, so the log adds no delay to /detected_face
        if self.frame_log is not None:
            self.log_frame(data, cv_image, recv_time, infer_ms, published,
                           (best_conf, box_center_x, box_cy, box_w, box_h))

    def log_frame(self, data, image, recv_time, infer_ms, published, box):
        score, cx, cy, w, h = box
        small = cv2.resize(image, (640, 360), interpolation=cv2.INTER_AREA)
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        # Variance of the Laplacian: drops when the image is smeared (turning)
        sharpness = cv2.Laplacian(gray, cv2.CV_64F).var()
        stamp = data.header.stamp.sec + data.header.stamp.nanosec * 1e-9
        top, bottom = max(cy - h / 2, 0.0), min(cy + h / 2, image.shape[0] - 1.0)
        self.frame_log.writerow([
            f'{recv_time:.3f}', f'{stamp:.3f}', f'{infer_ms:.0f}', f'{score:.3f}', int(published),
            f'{cx:.0f}', f'{cy:.0f}', f'{w:.0f}', f'{h:.0f}', f'{top:.0f}', f'{bottom:.0f}',
            f'{sharpness:.0f}', f'{gray.mean():.0f}'])
        self.frame_log_file.flush()

        if self.image_dir and recv_time - self.last_image_save >= self.debug_image_sec:
            self.last_image_save = recv_time
            k = 640.0 / image.shape[1]
            p0 = (int((cx - w / 2) * k), int(top * k))
            p1 = (int((cx + w / 2) * k), int(bottom * k))
            color = (0, 255, 0) if published else (0, 0, 255)
            cv2.rectangle(small, p0, p1, color, 2)
            cv2.putText(small, f'{score:.2f} sharp {sharpness:.0f}', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            cv2.imwrite(os.path.join(self.image_dir, f'{recv_time:.3f}_{score:.2f}.jpg'), small)


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
