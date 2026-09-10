#!/usr/bin/env python3
import json
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import requests
    REQUESTS_AVAILABLE = True
except ImportError:
    REQUESTS_AVAILABLE = False


GEMINI_URL = (
    'https://generativelanguage.googleapis.com/v1beta/models/{model}:generateContent'
)

API_KEY_ENV_VAR = 'GEMINI_API_KEY'

SYSTEM_PROMPT = (
    'You are the voice of an office guide dog robot built on a Unitree Go2. '
    'You greet people in the office lobby and then lead them where they need '
    'to go. Write exactly one short spoken greeting, at most 20 words. '
    'It will be read aloud by a speaker, so use plain words with no emoji, '
    'no markdown, no stage directions and no quotation marks. '
    'Be warm and brief.'
)


class GreetingNode(Node):
    """Listens for recognised faces and publishes a greeting sentence."""

    def __init__(self):
        super().__init__('greeting_node')

        # --- Parameters -------------------------------------------------
        self.declare_parameter('input_topic', '/recognized_person')
        self.declare_parameter('output_topic', '/speech_out')
        self.declare_parameter('greet_cooldown_seconds', 300.0)
        self.declare_parameter('use_llm', True)
        # Leave api_key empty to read the key from the GEMINI_API_KEY
        # environment variable instead, which keeps it out of git.
        self.declare_parameter('api_key', '')
        self.declare_parameter('model', 'gemini-2.5-flash')
        self.declare_parameter('max_tokens', 200)
        self.declare_parameter('timeout_seconds', 8.0)
        self.declare_parameter('robot_name', 'Oxy')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.greet_cooldown_seconds = float(
            self.get_parameter('greet_cooldown_seconds').value
        )
        self.use_llm = bool(self.get_parameter('use_llm').value)
        self.api_key = str(self.get_parameter('api_key').value).strip()
        self.model = self.get_parameter('model').value
        self.max_tokens = int(self.get_parameter('max_tokens').value)
        self.timeout_seconds = float(self.get_parameter('timeout_seconds').value)
        self.robot_name = self.get_parameter('robot_name').value

        self. last_greeted_at = {}

        self.gemini_ready = False
        self.setup_gemini()

        # --- ROS publishers and subscribers -----------------------------
        self.speech_publisher = self.create_publisher(String, output_topic, 10)
        self.recognized_subscription = self.create_subscription(
            String,
            input_topic,
            self.on_recognized_person,
            10,
        )

        self.get_logger().info('greeting_node started.')
        self.get_logger().info(f'  listening on : {input_topic}')
        self.get_logger().info(f'  publishing on: {output_topic}')
        self.get_logger().info(f'  cooldown     : {self.greet_cooldown_seconds} s')

    def setup_gemini(self):
        """Work out whether we can call Gemini, or use built-in greetings."""
        if not self.use_llm:
            self.get_logger().info('use_llm is false - using built-in greetings.')
            return

        if not REQUESTS_AVAILABLE:
            self.get_logger().warn(
                'The requests package is not installed - using built-in '
                'greetings. Install it with: pip install requests'
            )
            return

        # The api_key parameter wins. If it is empty we look in the
        # environment, which is the safer place - nothing ends up in git.
        if self.api_key == '':
            self.api_key = os.environ.get(API_KEY_ENV_VAR, '')

        if self.api_key == '':
            self.get_logger().warn(
                f'No Gemini API key found - using built-in greetings. Set the '
                f'api_key parameter, or export {API_KEY_ENV_VAR}.'
            )
            return

        self.gemini_ready = True
        self.get_logger().info(f'Gemini ready (model: {self.model}).')

    def on_recognized_person(self, msg):
        """Called for every frame face_recognition_node has processed."""
        # Step 1: the message is a JSON string, so turn it into a dictionary.
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as error:
            self.get_logger().warn(f'Could not read JSON on input topic: {error}')
            return

        # Step 2: pull out the name. face_recognition_node sets this to null
        # when it sees no face, or sees a face it does not recognise. It has
        # already applied its own similarity threshold, so a name being here
        # at all means it is confident enough.
        name = data.get('name')
        if name is None or name == '':
            return

        # Step 3: the cooldown. Have we greeted this person recently?
        # Without this check we would greet them on every single frame.
        now = time.time()
        if name in self.last_greeted_at:
            seconds_since_greeting = now - self.last_greeted_at[name]
            if seconds_since_greeting < self.greet_cooldown_seconds:
                return

        # Step 4: record that we are greeting them now. Do this BEFORE the
        # API call, because that call can take seconds, and more frames will
        # arrive while we wait.
        self.last_greeted_at[name] = now

        # Step 5: write the greeting. Try Gemini first, fall back if needed.
        greeting = None
        if self.gemini_ready:
            greeting = self.ask_gemini_for_greeting(name)

        if greeting is None:
            greeting = self.build_fallback_greeting(name)

        # Step 6: send it out for the speaker node to read.
        self.publish_greeting(greeting)

    def ask_gemini_for_greeting(self, name):
        """Ask Gemini for one greeting sentence, or return None if it fails."""
        request_text = (
            f'A person named {name} has just walked up to you in the office '
            f'lobby. Greet them by name.'
        )

        url = GEMINI_URL.format(model=self.model)

        # Gemini takes the key in a header, not in the URL. Putting it in the
        # URL would leak it into logs and browser history.
        headers = {
            'x-goog-api-key': self.api_key,
            'Content-Type': 'application/json',
        }

        body = {
            'system_instruction': {
                'parts': [{'text': SYSTEM_PROMPT}],
            },
            'contents': [
                {'parts': [{'text': request_text}]},
            ],
            'generationConfig': {
                'maxOutputTokens': self.max_tokens,
                # Gemini 2.5 models think before answering, and those thinking
                # tokens are taken out of maxOutputTokens. For a one-line
                # greeting that thinking is not worth the delay, and if it ate
                # the whole budget we would get an empty reply back. Setting
                # the budget to 0 turns it off.
                'thinkingConfig': {'thinkingBudget': 0},
            },
        }

        try:
            response = requests.post(
                url,
                headers=headers,
                json=body,
                timeout=self.timeout_seconds,
            )
            # Turns a 401, 404 or 500 into an exception we catch below.
            response.raise_for_status()
            reply = response.json()
        except Exception as error:
            # Catching everything on purpose. Whatever went wrong - no
            # network, bad key, wrong model name, rate limit, timeout - the
            # robot still needs to say something, and build_fallback_greeting
            # handles that.
            self.get_logger().warn(f'Gemini request failed: {error}')
            return None

        # Step through the reply carefully. Gemini can legitimately return no
        # candidates at all, for example if a safety filter blocked the reply,
        # so we never assume the pieces are there.
        candidates = reply.get('candidates')
        if not candidates:
            self.get_logger().warn(
                f'Gemini returned no candidates - using a built-in greeting. '
                f'Full reply: {reply}'
            )
            return None

        content = candidates[0].get('content')
        if content is None:
            self.get_logger().warn(
                f'Gemini reply had no content (finishReason: '
                f'{candidates[0].get("finishReason")}) - using a built-in greeting.'
            )
            return None

        parts = content.get('parts')
        if not parts:
            self.get_logger().warn(
                'Gemini reply had no text parts - using a built-in greeting.'
            )
            return None

        greeting = parts[0].get('text', '').strip()
        if greeting == '':
            self.get_logger().warn(
                'Gemini returned empty text - using a built-in greeting.'
            )
            return None

        return greeting

    def build_fallback_greeting(self, name):
        """The built-in greeting, used whenever Gemini is unavailable."""
        return f'Hello {name}. I am {self.robot_name}. Follow me.'

    def publish_greeting(self, greeting):
        """Send the greeting text out on /speech_out."""
        msg = String()
        msg.data = greeting
        self.speech_publisher.publish(msg)

        self.get_logger().info(f'Saying: {greeting}')


def main(args=None):
    rclpy.init(args=args)

    node = GreetingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
