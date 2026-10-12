"""ROS 2 adapter around the platform-independent OpenCV detector.

The callback converts ``sensor_msgs/Image`` to BGR, publishes frame-level
observations, applies temporal confirmation, and publishes confirmed events.
Motor, steering, brake, and laser commands are deliberately outside this node.
"""

from __future__ import annotations

from pathlib import Path

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, RegionOfInterest

from ugv_stage_interfaces.msg import StageSign

from .config import DetectorConfig
from .detector import SignDetection, StageSignDetector, annotate_frame
from .templates import TemplateRepository
from .temporal import TemporalConfirmer


class SignDetectorNode(Node):
    """Subscribe to a ROS camera topic and publish stage-sign messages."""
    def __init__(self) -> None:
        super().__init__("stage_sign_detector")
        self.declare_parameter("config_path", "")
        self.declare_parameter("templates_directory", "")
        self.declare_parameter("font_path", "")
        self.declare_parameter("classifier_mode", "")
        self.declare_parameter("model_path", "")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("publish_debug_image", True)

        config_path = self.get_parameter("config_path").value
        templates_directory = self.get_parameter("templates_directory").value
        font_path = self.get_parameter("font_path").value
        classifier_mode = self.get_parameter("classifier_mode").value
        model_path = self.get_parameter("model_path").value
        image_topic = self.get_parameter("image_topic").value
        self.publish_debug = bool(self.get_parameter("publish_debug_image").value)

        config = DetectorConfig.from_yaml(config_path) if config_path else DetectorConfig()
        if classifier_mode:
            config.classifier.mode = classifier_mode
        if model_path:
            config.classifier.model_path = str(Path(model_path).expanduser().resolve())
        config.validate()

        templates = None
        if config.classifier.mode == "template":
            templates = TemplateRepository.load_or_generate(
                config.templates,
                directory=templates_directory or None,
                font_path=font_path or None,
            )
        self.detector = StageSignDetector(config, templates)
        self.confirmer = TemporalConfirmer(config.temporal)
        self.bridge = CvBridge()

        # Camera traffic uses best-effort sensor QoS to avoid stale queues.
        # Stage events use reliable delivery because losing one can prevent a
        # required mission behavior from starting.
        event_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.observation_publisher = self.create_publisher(
            StageSign,
            "/perception/stage_sign/observation",
            event_qos,
        )
        self.confirmed_publisher = self.create_publisher(
            StageSign,
            "/perception/stage_sign/confirmed",
            event_qos,
        )
        self.debug_publisher = (
            self.create_publisher(Image, "/perception/stage_sign/debug_image", qos_profile_sensor_data)
            if self.publish_debug
            else None
        )
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"Listening for camera frames on {image_topic}; "
            f"classifier={config.classifier.mode}"
        )

    def _image_callback(self, image_message: Image) -> None:
        """Process exactly one image; callback errors do not stop the ROS node."""
        try:
            frame = self.bridge.imgmsg_to_cv2(image_message, desired_encoding="bgr8")
            detections = self.detector.detect(frame)
        except Exception as exc:  # ROS callback boundary
            self.get_logger().error(f"Stage-sign frame processing failed: {exc}")
            return

        # Raw observations are useful for dashboards and tuning, but only the
        # confirmed topic is allowed to feed the mission stage manager.
        for detection in detections:
            self.observation_publisher.publish(self._to_message(detection, image_message))

        best = detections[0] if detections else None
        confirmed = self.confirmer.update(best)
        if confirmed is not None:
            self.confirmed_publisher.publish(self._to_message(confirmed, image_message))
            self.get_logger().info(
                f"Confirmed stage sign {confirmed.sign_id} "
                f"(confidence={confirmed.confidence:.3f})"
            )

        if self.debug_publisher is not None:
            displayed = [confirmed] if confirmed is not None else detections
            status = (
                f"confirmed sign {confirmed.sign_id}"
                if confirmed is not None
                else "searching for stage signs"
            )
            debug_frame = annotate_frame(frame, displayed, status)
            debug_message = self.bridge.cv2_to_imgmsg(debug_frame, encoding="bgr8")
            debug_message.header = image_message.header
            self.debug_publisher.publish(debug_message)

    @staticmethod
    def _to_message(detection: SignDetection, source: Image) -> StageSign:
        message = StageSign()
        message.header = source.header
        message.sign_id = detection.sign_id
        message.confidence = detection.confidence
        box = detection.bounding_box
        message.bounding_box = RegionOfInterest(
            x_offset=max(0, box.x),
            y_offset=max(0, box.y),
            width=max(0, box.width),
            height=max(0, box.height),
            do_rectify=False,
        )
        message.confirmed = detection.confirmed
        return message


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SignDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
