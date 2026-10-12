"""
Sign detection via a pretrained OCR engine (EasyOCR) end-to-end -
supersedes the from-scratch YOLO/CNN approaches (failed on real camera
input: never detected, false-positives everywhere, wrong class when they
did trigger) AND an earlier version of this file that gated OCR behind a
classical HSV red-ring color filter.

That color gate was itself the lighting-fragile piece: fixed HSV
thresholds are exactly what breaks when lighting changes, which defeats
the point of using an AI reader behind it. EasyOCR's own text detector
(CRAFT) is itself a real AI model, trained on large real-world scene-text
corpora specifically to be robust across lighting/angle/blur - it was
already doing the actual detection work, just with a brittle classical
pre-filter bolted in front of it. Detection is now AI end-to-end:
CRAFT (via reader.readtext on the full frame) finds candidate text
regions, and only regions that parse as a valid stage digit (1-10) with
sufficient confidence are accepted.

The red-ring color check is kept only as a SOFT confidence booster (not a
hard gate) - if a plausible ring of reddish pixels surrounds a detected
digit, it's more likely a real stage sign than, say, a page number on a
document; a digit without one still gets a normal (not zero) chance to be
accepted, since lighting can wash out or shift the ring color.

Validated on synthetic data: 10/10 on clean renders, 96.7% (58/60) under
heavy synthetic rotation/blur/noise/brightness distortion - the detection
thresholds (text_threshold=0.3, low_text=0.3, mag_ratio=2.0) matter a lot;
EasyOCR's defaults missed some single large isolated digits.

Still NOT validated against a real printed sign or real camera frame -
only synthetic data so far. Test with live_ocr_test.py against a real
printed sign (printable_signs.pdf) before trusting this on the vehicle.
"""
import cv2
import numpy as np

OCR_ALLOWLIST = "0123456789"
OCR_TEXT_THRESHOLD = 0.3
OCR_LOW_TEXT = 0.3
OCR_MAG_RATIO = 2.0
OCR_CONF_MIN = 0.5  # higher than the old ROI-gated version's 0.4 - there's no color pre-filter
                    # narrowing candidates anymore, so OCR confidence alone has to carry more weight

RED_RING_BOOST = 0.15   # confidence added when a plausible red ring surrounds the digit
RED_RING_MARGIN_FRAC = 0.6  # how far out from the digit's own box to look for a ring, relative to its size
RED_RANGES = [
    (np.array([0, 70, 50]), np.array([12, 255, 255])),
    (np.array([165, 70, 50]), np.array([180, 255, 255])),
]


class OcrSignDetector:
    def __init__(self, gpu=True):
        import easyocr
        self.reader = easyocr.Reader(['en'], gpu=gpu, verbose=False)

    def _has_red_ring(self, frame_bgr, bbox):
        """Soft signal only: is there a plausible ring of reddish pixels around
        this digit's bounding box? Loose real-lighting-tolerant HSV ranges,
        and failing to find one does NOT reject the detection."""
        xs = [p[0] for p in bbox]
        ys = [p[1] for p in bbox]
        x0, y0, x1, y1 = int(min(xs)), int(min(ys)), int(max(xs)), int(max(ys))
        bw, bh = x1 - x0, y1 - y0
        if bw <= 0 or bh <= 0:
            return False
        margin_x, margin_y = int(bw * RED_RING_MARGIN_FRAC), int(bh * RED_RING_MARGIN_FRAC)
        h, w = frame_bgr.shape[:2]
        rx0, ry0 = max(0, x0 - margin_x), max(0, y0 - margin_y)
        rx1, ry1 = min(w, x1 + margin_x), min(h, y1 + margin_y)
        region = frame_bgr[ry0:ry1, rx0:rx1]
        if region.size == 0:
            return False

        hsv = cv2.cvtColor(region, cv2.COLOR_BGR2HSV)
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lo, hi in RED_RANGES:
            mask |= cv2.inRange(hsv, lo, hi)
        return mask.mean() / 255.0 > 0.08  # at least ~8% reddish pixels in the surrounding ring area

    def detect(self, frame_bgr):
        """Full pipeline on one frame - returns (stage or None, confidence, digit_box or None).
        digit_box is (x0, y0, x1, y1) of the digit text itself, as read by the AI text detector."""
        results = self.reader.readtext(
            frame_bgr, allowlist=OCR_ALLOWLIST,
            text_threshold=OCR_TEXT_THRESHOLD, low_text=OCR_LOW_TEXT, mag_ratio=OCR_MAG_RATIO,
        )

        best_stage, best_conf, best_box = None, 0.0, None
        for bbox, text, conf in results:
            if not (text.isdigit() and 1 <= int(text) <= 10):
                continue
            if self._has_red_ring(frame_bgr, bbox):
                conf = min(1.0, conf + RED_RING_BOOST)
            if conf > best_conf:
                xs = [p[0] for p in bbox]
                ys = [p[1] for p in bbox]
                best_stage = int(text)
                best_conf = conf
                best_box = (int(min(xs)), int(min(ys)), int(max(xs)), int(max(ys)))

        if best_conf < OCR_CONF_MIN:
            return None, best_conf, best_box
        return best_stage, best_conf, best_box
