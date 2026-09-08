import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from guide_dog_interfaces.msg import DetectedFace
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ultralytics import YOLO
import pickle
import os
import random


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        self.bridge = CvBridge()

        self.get_logger().info("Loading YOLOv8 Nano model...")
        self.model = YOLO('yolov8n.pt')

        self.image_sub = self.create_subscription(
            Image,
            '/d455/image',
            self.image_callback,
            10
        )

        self.face_pub = self.create_publisher(
            DetectedFace,
            '/detected_face',
            10
        )

        self.face_db = self.load_face_db(
            # 'config/face_db.pkl',
            '/home/janith/dog_v1_ws/src/guide_dog_perception/config/face_db.pkl'
        )

        self.get_logger().info("Perception Node online. Waiting for frames...")

    def load_face_db(self, path):
        if not os.path.exists(path):
            self.get_logger().warn(
                f'Face database not found at {path}. Face recognition will not match'
                ' individuals.'
            )
            return {}
        try:
            with open(path, 'rb') as f:
                db = pickle.load(f)
            self.get_logger().info(
                f'Loaded face DB from {path} with {len(db)} person(s).'
            )
            return db
        except Exception as e:
            self.get_logger().error(f'Failed to load face database: {e}')
        return {}

    # Selects a random key (person's name) from the loaded face_db dictionary and returns it with a hardcoded confidence score of 1.0
    def recognize_face(self):
        if not self.face_db:
            return "Unknown", 0.0

        # Pick a random key (person's name) from the loaded pickle database
        random_name = random.choice(list(self.face_db.keys()))

        # Return the random name with a dummy confidence score
        return random_name, 1.0

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

            # Calculate the center of the bounding box
            box_center_x = (x_min + x_max) / 2.0

            # Calculate the normalized offset (-1.0 to 1.0)
            offset_x = (box_center_x - image_center_x) / image_center_x

            recognized_name, match_score = self.recognize_face(
            )

            # Publish the data to the FSM
            msg = DetectedFace()
            msg.name = recognized_name
            msg.face_confidence = match_score
            msg.person_confidence = best_conf
            msg.center_offset_x = float(offset_x)
            msg.center_offset_y = 0.0

            self.face_pub.publish(msg)

            # Draw visual debugging markers
            cv2.rectangle(cv_image, (int(x_min), int(y_min)),
                          (int(x_max), int(y_max)), (0, 255, 0), 2)
            cv2.circle(cv_image, (int(box_center_x), int(
                (y_min + y_max)/2)), 5, (0, 0, 255), -1)

            display_text = f"{recognized_name} ({match_score:.2f})"

            cv2.putText(
                cv_image,
                display_text,
                (int(x_min), max(20, int(y_min) - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )

        # Show the live feed (Press 'q' inside the window to close it, or Ctrl+C in terminal)
        cv2.imshow("Guide Dog Vision", cv_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
