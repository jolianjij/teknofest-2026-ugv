#!/usr/bin/env python3
"""
Detects the competition's stage road signs (Figure 2 in the spec: 60cm
diameter, red ring, white fill, black Arial Black digit) on the front
camera and publishes which stage the vehicle is currently entering.

Detection AND classification both come from one trained CNN
(vision_models/sign_classifier/detector_model.py, SignDetectorNet) run
directly on the full frame - there is NO color thresholding anywhere in
this file. This supersedes an earlier version that used classical
HSV red-ring detection to localize a candidate region before classifying
it; per explicit request, localization itself had to stop depending on
color, not just the digit-reading step.

The network predicts, per frame: an objectness score (is a sign present),
a single bounding box (cx, cy, w, h - the task only ever has one relevant
sign in view at a time, matching the real competition layout), and a
10-way class distribution (stage number). See
vision_models/sign_classifier/dataset_detector.py and train_detector.py
for how it's trained (procedurally-generated synthetic scenes - no real
sign photos exist yet, same rationale as the original classifier, see
CLAUDE.md).

NOT YET VALIDATED against a real printed sign or the actual front camera -
only synthetic data and a laptop-webcam smoke test so far (see
vision_models/sign_classifier/live_test.py). Test against a real printed
sign before trusting this on the physical vehicle, per CLAUDE.md's
hardware-validation policy.

Publishes to /vehicle/current_stage (std_msgs/Int32): the stage number
(1-10, matching docs/robot_project_plan.md's numbering) the vehicle is
believed to currently be approaching/in.
"""
import sys
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge

SIGN_CLASSIFIER_DIR = Path(__file__).resolve().parent.parent / "vision_models" / "sign_classifier"
sys.path.insert(0, str(SIGN_CLASSIFIER_DIR))
from detector_model import INPUT_SIZE  # noqa: E402

MODEL_PATH = SIGN_CLASSIFIER_DIR / "sign_detector.onnx"
OBJECTNESS_THRESHOLD = 0.6   # tune against real footage - see CLAUDE.md's hardware-validation note
DEBOUNCE_FRAMES = 8          # consecutive detecting frames required before counting a NEW sign
REDETECT_COOLDOWN_FRAMES = 30  # frames to ignore further detections after counting one (avoid double-counting the same sign while still in view)

MEAN = np.array([0.5, 0.5, 0.5], dtype=np.float32)
STD = np.array([0.25, 0.25, 0.25], dtype=np.float32)

# Fixed stage sequence - see docs/robot_project_plan.md parkour layout.
STAGE_NAMES = {
    1: 'fording_and_shower', 2: 'aggregate_road', 3: 'side_slope',
    4: 'vertical_obstacle', 5: 'traffic_cones', 6: 'sliding_doors',
    7: 'bumped_road', 8: 'incline', 9: 'fire_zone', 10: 'decline',
}


def sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


class SignDetector(Node):
    def __init__(self):
        super().__init__('sign_detector')
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Int32, '/vehicle/current_stage', 10)
        self.sub = self.create_subscription(Image, '/front/image_raw', self.on_image, 10)

        if not MODEL_PATH.exists():
            raise FileNotFoundError(
                f'{MODEL_PATH} not found - run vision_models/sign_classifier/train_detector.py first '
                f'(needs the IOAI2026 venv: torch + onnx + onnxruntime).'
            )
        import onnxruntime as ort
        self.session = ort.InferenceSession(str(MODEL_PATH), providers=['CPUExecutionProvider'])

        self.stage_index = 0  # 0 = not started yet; increments to 1..10 as signs are found
        self.candidate_streak = 0
        self.pending_stage = None
        self.cooldown = 0
        self.get_logger().info('sign_detector started (single-shot CNN detector, no color thresholding), '
                                'waiting for /front/image_raw')

    def detect(self, frame_bgr):
        """Returns (stage 1-10, objectness_confidence) or (None, objectness_confidence)."""
        h, w = frame_bgr.shape[:2]
        side = min(h, w)
        y0, x0 = (h - side) // 2, (w - side) // 2
        square = frame_bgr[y0:y0 + side, x0:x0 + side]
        rgb = cv2.cvtColor(square, cv2.COLOR_BGR2RGB)
        resized = cv2.resize(rgb, (INPUT_SIZE, INPUT_SIZE), interpolation=cv2.INTER_LINEAR)
        arr = (resized.astype(np.float32) / 255.0 - MEAN) / STD
        arr = np.transpose(arr, (2, 0, 1))[None, ...]

        objectness, _bbox, class_logits = self.session.run(None, {'input': arr})
        obj_conf = float(sigmoid(objectness[0]))
        if obj_conf < OBJECTNESS_THRESHOLD:
            return None, obj_conf

        probs = np.exp(class_logits[0] - class_logits[0].max())
        probs /= probs.sum()
        stage = int(probs.argmax()) + 1  # class head is 0-indexed for stages 1-10
        return stage, obj_conf

    def on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'image conversion failed: {e}')
            return

        if self.cooldown > 0:
            self.cooldown -= 1
            return

        stage, obj_conf = self.detect(frame)
        if stage is None:
            self.candidate_streak = 0
            self.pending_stage = None
            return

        # Require the same stage prediction to hold for DEBOUNCE_FRAMES
        # consecutive frames before counting it - guards against a single
        # noisy frame flipping the class.
        if stage == self.pending_stage:
            self.candidate_streak += 1
        else:
            self.pending_stage = stage
            self.candidate_streak = 1

        if self.candidate_streak < DEBOUNCE_FRAMES:
            return

        self.stage_index = stage
        self.get_logger().info(
            f'detector confirmed stage {stage} ({STAGE_NAMES.get(stage, "?")}), objectness={obj_conf:.2f}'
        )
        self.pub.publish(Int32(data=self.stage_index))
        self.candidate_streak = 0
        self.pending_stage = None
        self.cooldown = REDETECT_COOLDOWN_FRAMES


def main():
    rclpy.init()
    node = SignDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
