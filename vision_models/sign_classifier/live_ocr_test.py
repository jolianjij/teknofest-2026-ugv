"""
Live test of ocr_sign_detector.py (classical red-ring localization +
EasyOCR digit reading) against a webcam - replaces the from-scratch
YOLO/CNN approaches that failed on real camera input.

Print printable_signs.pdf and hold a sign up to the camera to test
detection + digit reading. Press 'q' to quit.

Run with the IOAI2026 venv: C:/Users/jolian/Desktop/IOAI2026/.venv/Scripts/python.exe live_ocr_test.py
"""
import cv2

from ocr_sign_detector import OcrSignDetector


def main():
    detector = OcrSignDetector(gpu=True)

    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print("could not open webcam (index 0)")
        return

    print("running - press 'q' in the video window to quit")
    while True:
        ok, frame = cap.read()
        if not ok:
            print("frame grab failed")
            break

        stage, conf, box = detector.detect(frame)

        vis = frame.copy()
        if box is not None:
            x0, y0, x1, y1 = box
            color = (0, 255, 0) if stage is not None else (0, 165, 255)
            cv2.rectangle(vis, (x0, y0), (x1, y1), color, 2)
            if stage is not None:
                cv2.putText(vis, f"stage {stage} ({conf:.2f})", (x0, max(20, y0 - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            else:
                cv2.putText(vis, "sign-like blob, no digit read", (x0, max(20, y0 - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        else:
            cv2.putText(vis, "no candidate", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.imshow("OCR sign detector live test", vis)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
