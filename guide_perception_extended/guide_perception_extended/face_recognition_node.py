#!/usr/bin/env python3

import json
import os
import pickle
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from insightface.app import FaceAnalysis
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String


class FaceRecognitionNode(Node):
    """Detects the single most relevant face in each frame and identifies it."""

    def __init__(self):
        # This name is what shows up when you run "ros2 node list" - launch
        # files and the params yaml both expect it to stay "face_recognition_node".
        super().__init__('face_recognition_node')

        # These parameter names (the strings on the left) are also fixed - they
        # have to match face_recognition_params.yaml exactly. Only the variable
        # names on the right (self.something) are ours to name however we like.
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/recognized_person')
        self.declare_parameter('debug_image_topic', '/face_recognition/debug_image')
        self.declare_parameter('face_db_path', os.path.expanduser('~/oxy_face_db.pkl'))
        self.declare_parameter('detector_ctx_id', -1)
        self.declare_parameter('process_every_n_frames', 3)
        self.declare_parameter('similarity_threshold', 0.45)
        self.declare_parameter('publish_debug_image', True)

        self.cameraTopic = self.get_parameter('camera_topic').value
        self.outputTopic = self.get_parameter('output_topic').value
        self.debugImageTopic = self.get_parameter('debug_image_topic').value
        self.faceDatabaseFilePath = os.path.expanduser(self.get_parameter('face_db_path').value)
        self.detectorDeviceId = int(self.get_parameter('detector_ctx_id').value)
        self.processEveryNthFrame = max(1, int(self.get_parameter('process_every_n_frames').value))
        self.matchThreshold = float(self.get_parameter('similarity_threshold').value)
        self.shouldPublishDebugImage = bool(self.get_parameter('publish_debug_image').value)

        self.imageConverter = CvBridge()
        self.frameCounter = 0
        self.knownFaces = self.loadKnownFaces(self.faceDatabaseFilePath)

        self.get_logger().info('Loading InsightFace buffalo_l model (ctx_id=%d)...' % self.detectorDeviceId)
        self.faceDetector = FaceAnalysis(name='buffalo_l')
        self.faceDetector.prepare(ctx_id=self.detectorDeviceId)
        self.get_logger().info('InsightFace model ready.')

        # BEST_EFFORT + depth 1 = always look at the newest camera frame and
        # never queue up old ones. A guide dog should react to what the camera
        # sees right now, not catch up on frames from a second ago.
        cameraQos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.recognizedPersonPublisher = self.create_publisher(String, self.outputTopic, 10)
        self.debugImagePublisher = None
        if self.shouldPublishDebugImage:
            self.debugImagePublisher = self.create_publisher(Image, self.debugImageTopic, 10)

        self.imageSubscriber = self.create_subscription(
            Image, self.cameraTopic, self.onNewCameraImage, cameraQos
        )

        self.get_logger().info(
            f'face_recognition_node ready: camera_topic={self.cameraTopic}, '
            f'output_topic={self.outputTopic}, face_db entries={len(self.knownFaces)}'
        )

    def loadKnownFaces(self, path):
        """Load the {name: face embedding} pickle file. Warn and continue if it's missing."""
        if not os.path.exists(path):
            self.get_logger().warn(
                f'Face DB not found at {path}. Continuing with an empty database - '
                'all faces will be reported as unknown. Run enroll_faces to build one.'
            )
            return {}
        try:
            with open(path, 'rb') as fileHandle:
                savedFaces = pickle.load(fileHandle)
            if not isinstance(savedFaces, dict):
                raise ValueError('face DB pickle did not contain a dict')
            return savedFaces
        except Exception as error:
            self.get_logger().warn(f'Failed to load face DB from {path}: {error}. Continuing with empty DB.')
            return {}

    def pickBestFace(self, faces, frameShape):
        """If several faces are in view, pick the one a person would call 'the' face:
        the one that is biggest and closest to the middle of the frame."""
        if not faces:
            return None

        frameHeight, frameWidth = frameShape[:2]
        centerX = frameWidth / 2.0
        centerY = frameHeight / 2.0

        bestFace = None
        bestScore = -float('inf')
        for face in faces:
            left, top, right, bottom = face.bbox
            faceWidth = max(0.0, right - left)
            faceHeight = max(0.0, bottom - top)
            faceArea = faceWidth * faceHeight

            faceCenterX = (left + right) / 2.0
            faceCenterY = (top + bottom) / 2.0
            distanceFromMiddle = ((faceCenterX - centerX) ** 2 + (faceCenterY - centerY) ** 2) ** 0.5

            # Bigger faces score higher. Faces further from the middle of the
            # frame score lower. Whichever face has the highest score wins.
            score = faceArea - distanceFromMiddle
            if score > bestScore:
                bestScore = score
                bestFace = face
        return bestFace

    def matchFaceToKnownName(self, faceEmbedding):
        """Compare one detected face against everyone in the face database.
        Returns (name, similarity) for the closest match, or (None, similarity)
        if nobody was a close enough match."""
        if not self.knownFaces:
            return None, 0.0

        embeddingLength = np.linalg.norm(faceEmbedding)
        if embeddingLength == 0:
            return None, 0.0
        # Shrink the embedding down to length 1. Once both embeddings have
        # length 1, a plain dot product gives cosine similarity - a score from
        # -1 (opposite) to 1 (identical) that ignores how bright/close the
        # photo was, which is what we want to compare on.
        normalizedEmbedding = faceEmbedding / embeddingLength

        bestMatchName = None
        bestSimilarity = -1.0
        for name, savedEmbedding in self.knownFaces.items():
            similarity = float(np.dot(normalizedEmbedding, savedEmbedding))
            if similarity > bestSimilarity:
                bestSimilarity = similarity
                bestMatchName = name

        if bestSimilarity >= self.matchThreshold:
            return bestMatchName, bestSimilarity
        return None, bestSimilarity

    def drawDebugPicture(self, cameraFrame, faceBox, personName, matchConfidence):
        """Draw a box and a name label over the detected face, for humans watching
        the debug image topic. Never used for anything the robot decides on."""
        pictureWithBox = cameraFrame.copy()
        if faceBox is not None:
            left = int(faceBox[0])
            top = int(faceBox[1])
            right = int(faceBox[2])
            bottom = int(faceBox[3])

            boxColor = (0, 200, 0) if personName else (0, 0, 200)
            cv2.rectangle(pictureWithBox, (left, top), (right, bottom), boxColor, 2)

            labelText = f'{personName or "unknown"} ({matchConfidence:.2f})'
            labelY = max(0, top - 10)
            cv2.putText(
                pictureWithBox, labelText, (left, labelY),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, boxColor, 2, cv2.LINE_AA
            )
        return pictureWithBox

    def onNewCameraImage(self, cameraImage: Image):
        self.frameCounter += 1
        if self.frameCounter % self.processEveryNthFrame != 0:
            # Skip this frame - only run the (expensive) face model every Nth frame.
            return

        try:
            cameraFrame = self.imageConverter.imgmsg_to_cv2(cameraImage, desired_encoding='bgr8')
        except Exception as error:
            self.get_logger().warn(f'Failed to convert incoming image: {error}')
            return

        try:
            detectedFaces = self.faceDetector.get(cameraFrame)
        except Exception as error:
            self.get_logger().warn(f'InsightFace inference failed: {error}')
            return

        mainFace = self.pickBestFace(detectedFaces, cameraFrame.shape)

        personName = None
        matchConfidence = 0.0
        faceBox = None

        if mainFace is not None:
            faceBox = []
            for coordinate in mainFace.bbox:
                faceBox.append(float(coordinate))
            personName, matchConfidence = self.matchFaceToKnownName(mainFace.embedding)

        # These dict keys ('name', 'face_detected', ...) are read by other
        # nodes (e.g. guide_cognition) - keep them exactly as they are.
        resultData = {
            'timestamp': time.time(),
            'face_detected': mainFace is not None,
            'name': personName,
            'confidence': round(float(matchConfidence), 4),
            'bbox': faceBox,
        }

        outputMessage = String()
        outputMessage.data = json.dumps(resultData)
        self.recognizedPersonPublisher.publish(outputMessage)

        if self.shouldPublishDebugImage and self.debugImagePublisher is not None:
            pictureWithBox = self.drawDebugPicture(cameraFrame, faceBox, personName, matchConfidence)
            try:
                debugImageMessage = self.imageConverter.cv2_to_imgmsg(pictureWithBox, encoding='bgr8')
                debugImageMessage.header = cameraImage.header
                self.debugImagePublisher.publish(debugImageMessage)
            except Exception as error:
                self.get_logger().warn(f'Failed to publish debug image: {error}')


def main(args=None):
    rclpy.init(args=args)
    faceRecognitionNode = FaceRecognitionNode()
    try:
        rclpy.spin(faceRecognitionNode)
    finally:
        faceRecognitionNode.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
