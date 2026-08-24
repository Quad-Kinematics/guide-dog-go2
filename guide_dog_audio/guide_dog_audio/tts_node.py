import rclpy
from rclpy.node import Node
from guide_dog_interfaces.srv import Speak
import pyttsx3


class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_node')

        self.engine = pyttsx3.init()

        self.engine.setProperty('rate', 190)

        self.srv = self.create_service(
            Speak,
            '/speak',
            self.speak_callback
        )
        self.get_logger().info("Audio package online. TTS Service ready on '/speak'")

    def speak_callback(self, request, response):
        self.get_logger().info(f"Received request to speak: '{request.text}'")

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
