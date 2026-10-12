#!/usr/bin/env python3
"""
Publishes a drivable-area mask from the front camera for Nav2's semantic
costmap layer (Stage 6 in CLAUDE.md - "Vision costmap (segmentation
layer)"). Per the project's existing vision strategy, this is a
PREFERENCE signal only: lidar stays authoritative for lethal-obstacle
avoidance regardless of what this publishes.

Two backends, auto-selected:
  - CNN (vision_models/path_segmentation/cnn_model.py + a trained
    drivable_segnet.onnx), if a checkpoint file is present.
  - Classical ground-plane color-similarity heuristic
    (vision_models/path_segmentation/ground_plane_heuristic.py) otherwise -
    needs no training data at all, so this is the one actually running
    today.

NOT YET TESTED against real camera frames - no trained CNN checkpoint
exists yet (see cnn_model.py's docstring for why: terrain appearance can't
be procedurally synthesized the way the sign was, so training needs real
front-camera footage first). Do not trust either backend's output for
navigation decisions until validated against real footage per CLAUDE.md's
hardware-validation policy - test the classical fallback against
/front/image_raw as soon as the Orange Pi and front camera are reachable
together.

Publishes:
  /front/drivable_mask    sensor_msgs/Image, mono8 (255=drivable, 0=not)
  /front/ground_obstacles std_msgs/Float32MultiArray, flat triples of
                          [range_m, bearing_deg, width_px] per obstacle,
                          nearest first - obstacles standing ON the drivable
                          surface, which during stage 5 are the traffic cones.

The obstacle list exists because stage 5 is worth 50 points, loses 5 per
contact, and is the one stage the autonomous run may not pass on. Lidar
stays authoritative for it; this is the second opinion, plus the drivable-
surface judgement that a 2D lidar cannot make at all. See cone_terrain.py
for the extraction rule and its flat-ground projection caveats.
"""
import sys
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge

from cone_terrain import find_ground_obstacles

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent
# Deployed on the Pi everything lives flat in ~/teknofest_ws/, so the repo's
# vision_models/ tree does not exist there. Search beside the node first and
# fall back to the repo layout for local runs. Without this the node looked
# only under ~/vision_models/, never found a deployed checkpoint, and silently
# fell back to the classical heuristic - a trained model would have had no
# effect in production while appearing to work.
_SEARCH_DIRS = [HERE, REPO_ROOT / "vision_models" / "path_segmentation"]
for _d in _SEARCH_DIRS:
    if _d.is_dir():
        sys.path.insert(0, str(_d))
from ground_plane_heuristic import segment_drivable  # noqa: E402


def _find_model(filename):
    for directory in _SEARCH_DIRS:
        candidate = directory / filename
        if candidate.exists():
            return candidate
    return _SEARCH_DIRS[0] / filename  # reported in the log when missing


# Backend preference: NPU > CPU > classical. The RK3588's NPU sits idle while
# onnxruntime burns a full CPU core on the same work, so a .rknn is always
# preferred when one has been built and deployed.
RKNN_PATH = _find_model("drivable_segnet.rknn")
ONNX_PATH = _find_model("drivable_segnet.onnx")
CNN_INPUT_SIZE = 256

# Segmentation runs on the CPU (the NPU path is not wired up), and at the full
# camera rate it saturated four cores and starved the camera thread - front
# camera fell from ~29 fps to 5. Nothing needs a 30 Hz drivable mask: the
# vehicle is slow, Nav2's local costmap updates a few times a second, and the
# cone geometry barely changes between frames. Process every Nth frame instead.
PROCESS_EVERY_N = 5


class PathSegmentationNode(Node):
    def __init__(self):
        super().__init__('path_segmentation_node')
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, '/front/drivable_mask', 10)
        self.obstacle_pub = self.create_publisher(
            Float32MultiArray, '/front/ground_obstacles', 10
        )
        # Human-readable overlay. The raw mono8 mask is what the planner wants
        # but is near-useless to look at while tuning; this is what an operator
        # actually needs to judge whether segmentation is behaving.
        self.overlay_pub = self.create_publisher(Image, '/front/segmentation_overlay', 1)
        self.sub = self.create_subscription(Image, '/front/image_raw', self.on_image, 10)

        self.session = None
        self.rknn = None

        # 1) NPU
        if RKNN_PATH.exists():
            try:
                from rknnlite.api import RKNNLite
                rknn = RKNNLite()
                if rknn.load_rknn(str(RKNN_PATH)) != 0:
                    raise RuntimeError('load_rknn failed')
                # RK3588 has three NPU cores; AUTO lets the runtime schedule.
                if rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_AUTO) != 0:
                    raise RuntimeError('init_runtime failed')
                self.rknn = rknn
                self.get_logger().info(f'loaded NPU segmentation model from {RKNN_PATH}')
            except Exception as e:
                self.get_logger().warn(f'found {RKNN_PATH} but could not use the NPU ({e})')

        # 2) CPU
        if self.rknn is None and ONNX_PATH.exists():
            try:
                import onnxruntime as ort
                self.session = ort.InferenceSession(str(ONNX_PATH), providers=['CPUExecutionProvider'])
                self.get_logger().info(
                    f'loaded CNN segmentation model from {ONNX_PATH} (CPU - '
                    f'build a .rknn to move this to the NPU)'
                )
            except Exception as e:
                self.get_logger().warn(f'found {ONNX_PATH} but failed to load it ({e}) - using classical fallback')

        # 3) classical
        if self.session is None and self.rknn is None:
            self.get_logger().warn(
                'no trained drivable_segnet.onnx found - using the classical ground-plane color-similarity '
                'heuristic (vision_models/path_segmentation/ground_plane_heuristic.py). This is a real, '
                'usable fallback but has not been tuned against real footage yet.'
            )

    def on_image(self, msg: Image):
        self.frame_index = getattr(self, 'frame_index', 0) + 1
        if self.frame_index % PROCESS_EVERY_N:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'image conversion failed: {e}')
            return

        mask = self.run_cnn(frame) if self.session is not None else segment_drivable(frame)

        out_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
        out_msg.header = msg.header
        self.pub.publish(out_msg)

        obstacles = find_ground_obstacles(mask)
        obstacle_msg = Float32MultiArray()
        obstacle_msg.data = [
            value
            for obstacle in obstacles
            for value in (
                float(obstacle.range_m),
                float(obstacle.bearing_deg),
                float(obstacle.width_px),
            )
        ]
        self.obstacle_pub.publish(obstacle_msg)

        if self.overlay_pub.get_subscription_count() > 0:
            overlay = self.render_overlay(frame, mask, obstacles)
            overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
            overlay_msg.header = msg.header
            self.overlay_pub.publish(overlay_msg)

    def render_overlay(self, frame_bgr, mask, obstacles):
        """Green tint on drivable ground, each detected obstacle marked with
        the range and bearing actually being handed to the planner - so a wrong
        projection is visible rather than buried in a topic."""
        import cv2
        vis = frame_bgr.copy()
        drivable = mask > 0
        if drivable.any():
            tint = np.zeros_like(vis)
            tint[:] = (0, 190, 0)
            vis[drivable] = cv2.addWeighted(vis, 0.55, tint, 0.45, 0)[drivable]

        for obstacle in obstacles:
            x, y = obstacle.base_x_px, obstacle.base_y_px
            cv2.drawMarker(vis, (x, y), (0, 90, 255), cv2.MARKER_TRIANGLE_UP, 18, 2)
            cv2.putText(vis, f'{obstacle.range_m:.1f}m {obstacle.bearing_deg:+.0f}deg',
                        (max(0, x - 46), max(14, y - 14)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 90, 255), 1)

        backend = 'CNN' if self.session is not None else 'heuristic'
        cv2.putText(vis, f'{backend} | drivable {drivable.mean()*100:.0f}% | {len(obstacles)} obstacles',
                    (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.putText(vis, f'{backend} | drivable {drivable.mean()*100:.0f}% | {len(obstacles)} obstacles',
                    (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        return vis

    def run_cnn(self, frame_bgr):
        import cv2
        h, w = frame_bgr.shape[:2]
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        resized = cv2.resize(rgb, (CNN_INPUT_SIZE, CNN_INPUT_SIZE), interpolation=cv2.INTER_LINEAR)

        mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
        std = np.array([0.229, 0.224, 0.225], dtype=np.float32)
        inp = (resized.astype(np.float32) / 255.0 - mean) / std
        inp = np.transpose(inp, (2, 0, 1))[None, ...]

        logits = self.session.run(None, {'input': inp})[0]
        pred = logits.argmax(axis=1)[0].astype(np.uint8) * 255
        return cv2.resize(pred, (w, h), interpolation=cv2.INTER_NEAREST)


def main():
    rclpy.init()
    node = PathSegmentationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
