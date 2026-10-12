#!/usr/bin/env python3
"""
Autonomous laser aim+fire node - detects the competition's concentric-circle
target on the aim camera, tracks it via pan/tilt commands, and fires once
locked on target for the required 1-second hold.

Target geometry (from the official spec figure, confirmed by direct
measurement): three concentric circles, radii in a clean 1:2:3 ratio
(6cm/12cm/18cm in the real competition target). Detection uses that ratio
to reject false-positive circles rather than trusting a single Hough
circle in isolation.

Publishes to /esp32/aim_cmd as a Float32MultiArray [pan, tilt, fire] which
serial_bridge.py forwards to the ESP32 as 'A,<pan>,<tilt>,<fire>\\n' - see
esp32_firmware.ino's header comment for the full protocol. The ESP32 only
actually acts on this when SwB=autonomous AND SwC is in an aim position -
this node runs regardless and just publishes its best current command,
same pattern as how /cmd_vel already works.

Control loop (revised 2026-09-08): TargetFilter (exponential smoothing +
jump/miss rejection) feeds two PID controllers (pan/tilt) operating in
DEGREES via pixels_to_degrees(), not raw pixels.

Explicit acquisition state machine (2026-09-10, per explicit request:
"more robust detection, don't detect random noise" + "a better algorithm
to guide the mechanism"). Previously, ANY single frame's detection - real
or a false-positive contour glitch - immediately drove the PID and started
commanding gimbal motion. Now there are two states:

  SEARCHING - no confirmed target. Raw detections feed AcquisitionConfirmer
    (below) but the gimbal is held still - nothing drives the PID yet.
    Moves to TRACKING only once several recent frames agree on roughly the
    same position.
  TRACKING  - a confirmed lock exists. Raw detections feed TargetFilter for
    smoothing/jump-rejection as before, and the PID+rate-clamp stack
    guides the gimbal toward it. Drops back to SEARCHING if the target is
    genuinely lost for MAX_LOST_FRAMES.

AcquisitionConfirmer directly reuses the DESIGN (not the code - our target
position is continuous, not a discrete label) of
ugv_stage_sign_detector's TemporalConfirmer: a sliding window requiring a
majority of recent frames to agree before confirming anything, validated
on 4,300 real frames of that project's own footage. The adaptation here is
spatial clustering (do enough recent detections fall within a small pixel
radius of each other) instead of counting exact label matches. A lone
false-positive - glare, motion blur, some other round object in the room -
is very unlikely to reappear at the same pixel position several frames
running the way a real, physically-present target will.
"""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge

from target_detection import (
    AcquisitionConfirmer, PID, TargetFilter, detect_target, pixels_to_degrees,
)
# ACQUIRE_REQUIRED_HITS/ACQUIRE_HISTORY only needed here for the debug-image
# "confirming N/M" label - the class itself and its real defaults live in
# target_detection.py (see AcquisitionConfirmer there for why: no ROS
# dependency needed, so it belongs with TargetFilter/PID, not here).
from target_detection import ACQUIRE_HISTORY, ACQUIRE_REQUIRED_HITS


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# Target geometry (1:2:3 concentric ring ratio) and its matching tolerance
# are NOT configured here - detect_target() in target_detection.py owns
# both (RING_RATIOS, RATIO_TOLERANCE). A duplicate, unused set of the same
# constants used to live here; tuning ring-detection tolerance means editing
# them in target_detection.py, not this file.

# ---- Aim camera field of view - PLACEHOLDER, not measured. Needed to
# convert pixel error into degrees (pixels_to_degrees) so PID gains are
# expressed in real angular units instead of being entangled with whatever
# resolution the aim camera happens to be running at (2560x1440 per
# camera_manager_node.py's PROFILES). Ballparked from a typical low-cost USB
# webcam's spec range - measure the real C50's FOV before trusting the
# absolute gain values below, though the closed-loop behavior (PID toward
# zero pixel error) degrades gracefully even with a wrong FOV constant since
# it only rescales the effective gain, not the direction or stability.
AIM_CAMERA_HFOV_DEG = 70.0
AIM_CAMERA_VFOV_DEG = 50.0

# ---- Control tuning (all untuned starting guesses, same caveat as the
# face-tracking demo this is adapted from - watch behavior and adjust).
# PAN_KP/TILT_KP chosen so early behavior is roughly equivalent to the old
# 0.006-per-pixel proportional gain at the aim camera's nominal resolution
# and placeholder FOV above - a continuity starting point, not a validated
# value. ki=kd=0: no hardware exists yet to tune an integral/derivative term
# against, and a wrong nonzero ki risks windup with nobody watching it.
PAN_KP = 0.22              # normalized command per degree of horizontal error
TILT_KP = 0.17             # normalized command per degree of vertical error (frame is 1440 tall vs 2560 wide, so this differs from PAN_KP even for an equivalent starting response)

# Sign of each axis. Pan: CONFIRMED on real hardware 2026-09-10 (both +1/+1
# and -1/-1 tested live; +1/+1 moved backwards), -1.0 is correct.
# Tilt: flipped back to +1.0 on 2026-09-10 per a direct live-hardware report
# that tilt was moving reversed at -1.0. Each axis is independent - if
# either is ever wrong again, flip only that one here.
AIM_PAN_SIGN = -1.0
AIM_TILT_SIGN = 1.0

# Per-update command-change clamp, borrowed from vision_prototyping's
# face_track_aim_demo.py (a working, hardware-confirmed pan/tilt tracker -
# per its own comment: "smaller+more-frequent moves read as smoother than
# big infrequent jumps"). Our PID's out_limit clamps the ABSOLUTE command
# but not how fast it can CHANGE between updates - a single bad detection
# could otherwise swing the command from one extreme to the other in one
# step. This clamps that regardless of what the detector says. 0.15 means
# a full 0->1 swing takes ~7 updates minimum - untuned, watch for either
# lag (raise it) or continued jumpiness (lower it) on real hardware.
MAX_COMMAND_DELTA_PER_UPDATE = 0.15
DEADZONE_PX = 6            # pixel error below this counts as "centered" (kept in pixel space - a detector-noise floor, not an angle)
LOCK_HOLD_SECONDS = 1.2    # spec requires holding designation 1s - small margin added
MAX_LOST_FRAMES = 5        # consecutive no-detection frames (while TRACKING) before dropping back to SEARCHING

class AimTracker(Node):
    def __init__(self):
        super().__init__('aim_tracker')
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Float32MultiArray, '/esp32/aim_cmd', 10)
        # Operator view of the engagement: what the tracker sees, where it
        # thinks the target is, and how long it has held lock. Firing is gated
        # on a 1 s continuous hold, so being able to watch that hold build (or
        # keep resetting) is the difference between diagnosing an aim problem
        # and guessing at one.
        self.debug_pub = self.create_publisher(Image, '/aim/debug_image', 1)
        self.sub = self.create_subscription(Image, '/aim/image_raw', self.on_image, 10)

        self.state = 'searching'  # 'searching' | 'tracking'
        self.acquisition = AcquisitionConfirmer()
        self.locked_since = None
        self.lost_frames = 0
        self.target_filter = TargetFilter()
        # ki=kd=0 for now (see the PAN_KP/TILT_KP comment above) - deadband=0
        # because DEADZONE_PX above already decides "centered" before either
        # PID is even consulted; out_limit=1.0 matches the aim_cmd protocol's
        # normalized -1..1 range, so no separate clamp() call is needed on
        # the output.
        self.pan_pid = PID(kp=PAN_KP, ki=0.0, kd=0.0, deadband=0.0, out_limit=1.0)
        self.tilt_pid = PID(kp=TILT_KP, ki=0.0, kd=0.0, deadband=0.0, out_limit=1.0)
        # Baseline for MAX_COMMAND_DELTA_PER_UPDATE - reset to 0 whenever we
        # stop commanding motion (centered/lock, or target lost) so the
        # clamp doesn't hold a stale baseline from before the stop.
        self._last_pan = 0.0
        self._last_tilt = 0.0
        self.get_logger().info('aim_tracker started, waiting for /aim/image_raw')

    def publish_debug(self, frame, result, held_seconds, firing, source_msg):
        vis = frame.copy()
        h, w = vis.shape[:2]
        cx, cy = w // 2, h // 2

        # Boresight: where the laser points. The target must be brought here.
        cv2.drawMarker(vis, (cx, cy), (200, 200, 200), cv2.MARKER_CROSS, 26, 1)
        cv2.rectangle(vis, (cx - DEADZONE_PX, cy - DEADZONE_PX),
                      (cx + DEADZONE_PX, cy + DEADZONE_PX), (140, 140, 140), 1)

        if result is not None:
            tx, ty, inner_r = result
            tx, ty, inner_r = int(tx), int(ty), int(inner_r)
            colour = (0, 0, 255) if firing else (0, 200, 255)
            for scale in (1, 2, 3):  # the 1:2:3 concentric ring set
                cv2.circle(vis, (tx, ty), max(1, inner_r * scale), colour, 2)
            cv2.line(vis, (cx, cy), (tx, ty), colour, 1)
            err = f'err {tx - cx:+d},{ty - cy:+d}px'
        else:
            err = 'no target'

        if firing:
            banner, colour = 'FIRING', (0, 0, 255)
        elif self.locked_since is not None:
            banner, colour = f'LOCK {held_seconds:.2f}/{LOCK_HOLD_SECONDS:.2f}s', (0, 200, 255)
        elif self.state == 'searching':
            hits = getattr(self.acquisition, 'last_hit_count', 0)
            banner, colour = f'SEARCHING (confirming {hits}/{ACQUIRE_REQUIRED_HITS})', (200, 200, 200)
        else:
            banner, colour = 'TRACKING', (0, 200, 255)

        cv2.putText(vis, f'{banner}  {err}', (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
        cv2.putText(vis, f'{banner}  {err}', (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, colour, 1)

        out = self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
        out.header = source_msg.header
        self.debug_pub.publish(out)

    def find_target(self, frame_bgr):
        """Delegates to target_detection.detect_target - a nested-contour
        walk, which is the right primitive for concentric rings and replaces
        the Hough-circle version that scored 0/7 on clean targets. Returns
        (cx, cy, outer_radius) or None."""
        result = detect_target(frame_bgr)
        if result is None:
            return None
        cx, cy, radius, _confidence = result
        return (float(cx), float(cy), float(radius))

    def _stop(self):
        self._last_pan = 0.0
        self._last_tilt = 0.0
        self.publish_cmd(0.0, 0.0, False)

    def on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'image conversion failed: {e}')
            return

        raw_result = self.find_target(frame)

        if self.state == 'searching':
            self._handle_searching(frame, msg, raw_result)
        else:
            self._handle_tracking(frame, msg, raw_result)

    def _handle_searching(self, frame, msg, raw_result):
        """No confirmed lock yet. The gimbal is held still regardless of
        what any single frame detects - only AcquisitionConfirmer deciding
        several recent frames agree is allowed to start commanding motion."""
        confirmed = self.acquisition.update(raw_result)
        self._stop()
        if confirmed is not None:
            self.state = 'tracking'
            self.acquisition.reset()
            self.target_filter.reset()
            self.target_filter.update(confirmed)
            self.pan_pid.reset()
            self.tilt_pid.reset()
            self.lost_frames = 0
            self.get_logger().info(
                f'target acquired at ({confirmed[0]:.0f},{confirmed[1]:.0f}) after '
                f'{ACQUIRE_REQUIRED_HITS}/{ACQUIRE_HISTORY}-frame confirmation - tracking'
            )
        if self.debug_pub.get_subscription_count() > 0:
            self.publish_debug(frame, raw_result, 0.0, False, msg)

    def _handle_tracking(self, frame, msg, raw_result):
        """A confirmed lock exists - smooth, guide, and hold/fire as before."""
        h, w = frame.shape[:2]
        center_x, center_y = w / 2.0, h / 2.0
        now = self.get_clock().now()

        if raw_result is None:
            self.lost_frames += 1
            # TargetFilter tracks its own miss-timeout internally and only
            # does so correctly if it is fed None on every missed frame,
            # same as a real detection - this is what lets a 1-2 frame
            # dropout hold the last smoothed position instead of snapping to
            # "no target" immediately.
            self.target_filter.update(None)
            if self.lost_frames > MAX_LOST_FRAMES:
                # Genuinely gone - drop back to requiring a fresh multi-frame
                # confirmation rather than resuming tracking on whatever
                # reappears next, which could be a different object entirely.
                self.state = 'searching'
                self.acquisition.reset()
                self.target_filter.reset()
                self.pan_pid.reset()
                self.tilt_pid.reset()
                self.locked_since = None
                self.get_logger().info('target lost - back to searching')
            self._stop()
            if self.debug_pub.get_subscription_count() > 0:
                self.publish_debug(frame, None, 0.0, False, msg)
            return

        self.lost_frames = 0
        filtered = self.target_filter.update(raw_result)
        if filtered is None:
            # Should not happen right after a non-None detection (see
            # TargetFilter.update), but fail toward "hold still" rather than
            # crash on an unexpected None.
            self._stop()
            return
        tx, ty, _ = filtered
        err_x = tx - center_x
        err_y = ty - center_y

        centered = abs(err_x) < DEADZONE_PX and abs(err_y) < DEADZONE_PX

        held_seconds = 0.0
        fire = False
        if centered:
            if self.locked_since is None:
                self.locked_since = now
            held_seconds = (now - self.locked_since).nanoseconds / 1e9
            fire = held_seconds >= LOCK_HOLD_SECONDS
            # Centered and (about to be / already) firing - per the spec,
            # nothing may move while the laser is active, so send zero
            # pan/tilt rather than any residual correction. Reset both PIDs
            # so a later re-acquisition doesn't inherit stale integral/
            # derivative state from before the target was centered.
            self.pan_pid.reset()
            self.tilt_pid.reset()
            self._last_pan = 0.0
            self._last_tilt = 0.0
            self.publish_cmd(0.0, 0.0, fire)
        else:
            self.locked_since = None
            err_x_deg = pixels_to_degrees(err_x, w, AIM_CAMERA_HFOV_DEG)
            err_y_deg = pixels_to_degrees(err_y, h, AIM_CAMERA_VFOV_DEG)
            pan_raw = self.pan_pid.update(AIM_PAN_SIGN * err_x_deg)
            tilt_raw = self.tilt_pid.update(AIM_TILT_SIGN * err_y_deg)
            # Clamp how much the command may change from the last one,
            # regardless of what the PID/detector just said - see
            # MAX_COMMAND_DELTA_PER_UPDATE above. This is what actually
            # protects the gimbal from a single bad detection (a
            # false-positive contour elsewhere in frame) commanding a
            # full-scale swing in one step.
            pan = clamp(pan_raw, self._last_pan - MAX_COMMAND_DELTA_PER_UPDATE,
                        self._last_pan + MAX_COMMAND_DELTA_PER_UPDATE)
            tilt = clamp(tilt_raw, self._last_tilt - MAX_COMMAND_DELTA_PER_UPDATE,
                         self._last_tilt + MAX_COMMAND_DELTA_PER_UPDATE)
            self._last_pan = pan
            self._last_tilt = tilt
            self.get_logger().info(
                f'err=({err_x:+.0f},{err_y:+.0f})px -> cmd=({pan:+.3f},{tilt:+.3f}) '
                f'[flip AIM_PAN_SIGN/AIM_TILT_SIGN if either moves away from target]',
                throttle_duration_sec=0.5,
            )
            self.publish_cmd(pan, tilt, False)

        if self.debug_pub.get_subscription_count() > 0:
            self.publish_debug(frame, filtered, held_seconds, fire, msg)

    def publish_cmd(self, pan, tilt, fire):
        msg = Float32MultiArray()
        msg.data = [float(pan), float(tilt), 1.0 if fire else 0.0]
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = AimTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
