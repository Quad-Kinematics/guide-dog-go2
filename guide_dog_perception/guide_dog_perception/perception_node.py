import os

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from guide_dog_interfaces.msg import DetectedFace
from cv_bridge import CvBridge, CvBridgeError
import cv2
import torch
from ultralytics import YOLO


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        self.bridge = CvBridge()

        # Default: the copy installed with this package, so the node runs from
        # any directory. Missing file = error: YOLO() would otherwise try to
        # download weights from the internet.
        model_path = self.declare_parameter(
            'model_path',
            os.path.join(get_package_share_directory('guide_dog_perception'),
                         'models', 'yolov8n.pt')).value
        if not os.path.isfile(model_path):
            raise FileNotFoundError(f"YOLO model not found: {model_path}")

        # Inference runs on the CPU (no CUDA torch on the Jetson). 4 threads
        # take ~210 ms/frame vs ~185 ms with all 8, and leave half the cores
        # to Nav2 so the controller keeps its rate.
        num_threads = self.declare_parameter('num_threads', 4).value
        if num_threads > 0:
            torch.set_num_threads(num_threads)

        self.get_logger().info(f"Loading YOLOv8 Nano model from {model_path}...")
        self.model = YOLO(model_path)

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

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        height, width, channels = cv_image.shape
        image_center_x = width / 2.0

        results = self.model(cv_image, verbose=False)

        person_found = False
        best_conf = 0.0
        best_box = None

        # Parse the YOLO results
        for result in results:
            boxes = result.boxes
            for box in boxes:
                # Class 0 in the COCO dataset is 'Person'
                if int(box.cls[0]) == 0:
                    conf = float(box.conf[0])

                    # If there are multiple people, lock onto the most confident one
                    if conf > best_conf and conf > 0.50:  # 50% confidence threshold
                        best_conf = conf
                        # Returns [x_min, y_min, x_max, y_max]
                        best_box = box.xyxy[0].tolist()
                        person_found = True

        if person_found and best_box is not None:
            x_min, y_min, x_max, y_max = best_box

            # 1. Calculate the center of the bounding box
            box_center_x = (x_min + x_max) / 2.0

            # 2. Calculate the normalized offset (-1.0 to 1.0)
            offset_x = (box_center_x - image_center_x) / image_center_x

            # 3. Publish the data to the FSM
            msg = DetectedFace()
            msg.name = "Person"  # For V1, we just target any human
            msg.confidence = best_conf
            msg.center_offset_x = float(offset_x)
            msg.center_offset_y = 0.0

            self.face_pub.publish(msg)

            # 4. Draw visual debugging markers
            # cv2.rectangle(cv_image, (int(x_min), int(y_min)),
            #              (int(x_max), int(y_max)), (0, 255, 0), 2)
            # cv2.circle(cv_image, (int(box_center_x), int(
            #   (y_min + y_max)/2)), 5, (0, 0, 255), -1)

        # Show the live feed (Press 'q' inside the window to close it, or Ctrl+C in terminal)
        # cv2.imshow("Guide Dog Vision", cv_image)
        # cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
