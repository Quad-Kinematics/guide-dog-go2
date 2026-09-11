import rclpy
from rclpy.node import Node
from guide_dog_interfaces.srv import Speak
from std_msgs.msg import String
import pyttsx3


class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_node')

        self.engine = pyttsx3.init()
        self.engine.setProperty('rate', 200)

        # Service for speech synthesis
        self.srv = self.create_service(
            Speak,
            '/speak',
            self.speak_callback
        )

        # Publisher to broadcast spoken text for logging/diagnostics
        self.speech_pub = self.create_publisher(String, '/spoken_text', 10)

        self.get_logger().info("Audio package online. TTS Service ready on '/speak'")

    def speak_callback(self, request, response):
        self.get_logger().info(f"Received request to speak: '{request.text}'")

        # Broadcast for logger
        msg = String()
        msg.data = request.text
        self.speech_pub.publish(msg)

        try:
            self.engine.say(request.text)
            self.engine.runAndWait()
            response.success = True
        except Exception as e:
            self.get_logger().error(f"TTS Engine failed: {e}")
            response.success = False

        return response


def main(args=None):
    rclpy.init(args=args)
    tts_node = TTSNode()

    try:
        rclpy.spin(tts_node)
    except KeyboardInterrupt:
        pass
    finally:
        tts_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
