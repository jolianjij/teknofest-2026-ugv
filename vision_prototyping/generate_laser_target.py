"""
Generates a precisely-scaled printable laser-fire target matching the
official spec figure (radii 6/12/18cm exactly, 1:2:3 ratio, alternating
black/white/black). The real target is 36cm diameter, which does NOT fit
on A3 paper (A3's short side is 29.7cm) despite the spec text calling it
"A3 sheet size" - a real contradiction in the official document. This
generates a 50%-scale version (radii 3/6/9cm, 18cm diameter) that fits
comfortably on A4/Letter for quick detection/tracking testing, plus a
printed calibration line so you can verify your printer didn't silently
rescale the page (a common "fit to page" default gotcha).

Run with: python generate_laser_target.py
Print at 100% / "actual size" - NOT "fit to page" - or the calibration
line will not measure correctly and the scale note will be wrong.
"""
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import cm
from reportlab.pdfgen import canvas

OUT_PATH = "laser_target_50pct_scale.pdf"

SCALE = 0.5  # this print is at 50% of the real competition target
REAL_RADII_CM = [6, 12, 18]  # inner, middle, outer - from the spec figure
PRINT_RADII_CM = [r * SCALE for r in REAL_RADII_CM]

page_w, page_h = A4
c = canvas.Canvas(OUT_PATH, pagesize=A4)

center_x = page_w / 2
center_y = page_h / 2 + 2 * cm  # nudge up to leave room for the calibration line/notes below

# Outer ring: black disc from outer radius down to middle radius (draw
# outer black circle first, then paint over the middle-to-inner area white,
# then the inner disc black again - simplest way to get black/white/black
# without a stroke-only donut primitive).
inner_r, middle_r, outer_r = [r * cm for r in PRINT_RADII_CM]  # PRINT_RADII_CM is [inner, middle, outer]

c.setFillColorRGB(0, 0, 0)
c.circle(center_x, center_y, outer_r, stroke=0, fill=1)
c.setFillColorRGB(1, 1, 1)
c.circle(center_x, center_y, middle_r, stroke=0, fill=1)
c.setFillColorRGB(0, 0, 0)
c.circle(center_x, center_y, inner_r, stroke=0, fill=1)

# Center crosshair, thin, doesn't interfere with Hough circle detection
c.setStrokeColorRGB(1, 0, 0)
c.setLineWidth(0.5)
c.line(center_x - 0.3 * cm, center_y, center_x + 0.3 * cm, center_y)
c.line(center_x, center_y - 0.3 * cm, center_x, center_y + 0.3 * cm)

# Calibration line - measure this with a ruler after printing. If it's not
# exactly 5.00cm, your printer rescaled the page; fix print settings
# (100% / actual size, not "fit to page") and reprint.
cal_y = 2.5 * cm
c.setStrokeColorRGB(0, 0, 0)
c.setLineWidth(1)
c.line(2 * cm, cal_y, 2 * cm + 5 * cm, cal_y)
c.setFont("Helvetica", 9)
c.drawString(2 * cm, cal_y + 0.3 * cm, "Calibration line - must measure exactly 5.00 cm after printing")

c.setFont("Helvetica", 9)
c.drawString(2 * cm, 1.3 * cm,
             f"PRINTED AT 50% SCALE. Real competition target radii: 6/12/18cm (36cm diameter).")
c.drawString(2 * cm, 0.8 * cm,
             f"This print's radii: {PRINT_RADII_CM[0]:.1f}/{PRINT_RADII_CM[1]:.1f}/{PRINT_RADII_CM[2]:.1f}cm ({PRINT_RADII_CM[2]*2:.0f}cm diameter).")
c.drawString(2 * cm, 0.3 * cm,
             "Print at 100% / 'actual size' - NOT 'fit to page' - or these numbers won't be correct.")

c.save()
print(f"Wrote {OUT_PATH}")
