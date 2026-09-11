import csv
import os
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from guide_dog_interfaces.msg import DetectedFace
from yasmin_msgs.msg import StateMachine
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy


class SimLogger(Node):
    def __init__(self):
        super().__init__('sim_logger')

        # 1. Odometry Subscriber
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)

        # 2. Detected Person Subscriber
        self.person_sub = self.create_subscription(
            DetectedFace,
            '/detected_person',
            self.person_callback,
            10)

        # 3. Speech Log Subscriber
        self.speech_sub = self.create_subscription(
            String,
            '/spoken_text',
            self.speech_callback,
            10)

        yasmin_qos = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        # 2. Update your subscriber to use this new QoS profile
        self.state_sub = self.create_subscription(
            StateMachine,
            '/fsm_viewer',
            self.state_callback,
            yasmin_qos)

        self.last_state = "UNKNOWN"

        self.current_x = 0.0
        self.current_y = 0.0
        self.last_logged_person = None
        self.last_person_log_time = 0.0

        # Create output directory
        save_dir = os.path.expanduser('~/dog_v1_ws/sim_results/csv')
        os.makedirs(save_dir, exist_ok=True)

        filename = os.path.join(save_dir, f'run_{int(time.time())}.csv')
        self.file = open(filename, mode='w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(
            ['timestamp', 'x', 'y', 'event_type', 'event_label'])

        self.get_logger().info(
            f'Logging trajectory and audio events to {filename}...')

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        timestamp = self.get_clock().now().nanoseconds / 1e9
        self.writer.writerow(
            [timestamp, self.current_x, self.current_y, 'path', 'none'])

    def state_callback(self, msg):
        if not msg.states:
            return

        active_state_id = -1
        active_state_name = "UNKNOWN"

        # 1. Find the root FSM state to get the ID of the currently active state
        for state in msg.states:
            if state.is_fsm and state.current_state != -1:
                active_state_id = state.current_state
                break

        # 2. Match that ID to the state's actual name (e.g., ID 7 -> "IDLE")
        if active_state_id != -1:
            for state in msg.states:
                if state.id == active_state_id:
                    active_state_name = state.name
                    break

        # 3. Log it if it's a new transition
        if active_state_name != self.last_state and active_state_name != "UNKNOWN":
            current_time = self.get_clock().now().nanoseconds / 1e9
            self.last_state = active_state_name

            self.writer.writerow(
                [current_time, self.current_x, self.current_y, 'state', self.last_state])
            self.get_logger().info(f"FSM Transitioned to: '{self.last_state}'")

    def person_callback(self, msg):
        current_time = self.get_clock().now().nanoseconds / 1e9
        if msg.person_confidence < 0.5 or not msg.name:
            return

        if (msg.name == self.last_logged_person) and (current_time - self.last_person_log_time < 2.0):
            return

        self.last_logged_person = msg.name
        self.last_person_log_time = current_time

        self.writer.writerow(
            [current_time, self.current_x, self.current_y, 'person', msg.name])
        self.get_logger().info(
            f"Logged detection: '{msg.name}' at ({self.current_x:.2f}, {self.current_y:.2f})")

    def speech_callback(self, msg):
        current_time = self.get_clock().now().nanoseconds / 1e9
        # Escape quotes/commas cleanly via standard csv writer
        self.writer.writerow(
            [current_time, self.current_x, self.current_y, 'speech', msg.data])
        self.get_logger().info(f"Logged speech prompt: '{msg.data}'")


def main(args=None):
    rclpy.init(args=args)
    logger_node = SimLogger()

    try:
        rclpy.spin(logger_node)
    except KeyboardInterrupt:
        pass
    finally:
        logger_node.file.close()
        logger_node.destroy_node()
        rclpy.shutdown()
        print("Log file saved successfully.")


if __name__ == '__main__':
    main()
