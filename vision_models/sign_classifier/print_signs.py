"""
Generates a printable PDF, one page per stage sign (1-10), each a large
centered red-ring/white-fill/black-digit disc - for bridging the sim-to-real
gap: the YOLO detector trained purely on procedural synthetic renders
fails on real camera input (confirmed: never detects, false-positives on
random objects, wrong class when it does trigger). Printing these and
capturing real webcam frames of them (capture_real_data.py) grounds the
model in real lighting/camera-noise/background statistics that no amount
of synthetic augmentation was reproducing.

~15cm diameter per sign - comfortable to hold up to a laptop webcam at
arm's length. Same visual design as dataset.py's _render_sign (red ring,
white fill, black bold digit) - not a claim this matches the real
competition sign exactly (that's still unverified, see CLAUDE.md), just a
consistent target for closing the camera/lighting domain gap.

Run with the IOAI2026 venv (or any Python with reportlab):
    python print_signs.py
Print at 100% / "actual size", not "fit to page".
"""
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import cm
from reportlab.pdfgen import canvas

OUT_PATH = "printable_signs.pdf"
DIAMETER_CM = 15
RING_FRAC = 0.18  # ring thickness as a fraction of outer radius, matches dataset.py's typical range

page_w, page_h = A4
c = canvas.Canvas(OUT_PATH, pagesize=A4)

for stage in range(1, 11):
    center_x, center_y = page_w / 2, page_h / 2 + 1 * cm
    outer_r = (DIAMETER_CM / 2) * cm
    inner_r = outer_r * (1 - RING_FRAC)

    c.setFillColorRGB(0.78, 0.08, 0.08)
    c.circle(center_x, center_y, outer_r, stroke=0, fill=1)
    c.setFillColorRGB(0.96, 0.96, 0.96)
    c.circle(center_x, center_y, inner_r, stroke=0, fill=1)

    text = str(stage)
    c.setFillColorRGB(0.04, 0.04, 0.04)
    font_size = inner_r * (1.5 if len(text) == 1 else 1.15)
    c.setFont("Helvetica-Bold", font_size)
    tw = c.stringWidth(text, "Helvetica-Bold", font_size)
    c.drawString(center_x - tw / 2, center_y - font_size * 0.35, text)

    c.setFont("Helvetica", 11)
    c.drawCentredString(page_w / 2, 2 * cm, f"Stage {stage} sign - print at 100% / actual size")

    c.showPage()

c.save()
print(f"wrote {OUT_PATH} (10 pages, one per stage)")
