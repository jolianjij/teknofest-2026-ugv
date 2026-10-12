# Robot Project Plan — TEKNOFEST 2026 Unmanned Ground Vehicle Competition

Source of truth: [`2026_UGV_Competition_Specification_EN.pdf`](2026_UGV_Competition_Specification_EN.pdf) (official English specification, V1.0, 29.1.2026). This document distills that spec into engineering-actionable form. Where this file and the PDF disagree, the PDF wins — update this file.

## Competition format
Three timed runs, each ≤15 minutes (including the fire mission), same parkour, points summed across all runs:
1. **Run 1 — Remote-controlled (manual):** human operator, sees the vehicle/parkour, navigates via onboard cameras/sensors over a remote link, fires the laser manually.
2. **Run 2 — Fully autonomous:** navigation, laser designation, and laser fire all autonomous.
3. Teams may invoke **"right to pass-on"** once per run: hand-carry the vehicle past one stage, run clock keeps going, and the team eats a penalty equal to that stage's max points (acceleration road and traffic cones in the autonomous run cannot be passed on).

Final grade = 20% Critical Design Report + 10% Technical Presentation + 70% Final Competition (sum of both runs).

## Parkour layout (Figure 1, PDF page 14 — actually viewed, not inferred from text)
The parkour is a **3-lane serpentine (boustrophedon)**, not a single line — two ~180° U-turns connect three parallel straight lanes:

- **Bottom lane** (start → left): Start → Fording & Shower (1) → Aggregate/rocky road (2) → Side slope (3) → *U-turn at far left*
- **Middle lane** (left → right): Vertical obstacle (4) → Traffic cones (5) → Sliding doors (6) → *U-turn at far right*
- **Top lane** (right → left): Bumped road (7) → Incline, stop 2s (8) → Fire zone (9) → Decline, stop 2s (10) → Finish

The **laser target sits near the finish line**, off to the side, roughly facing back down the top lane — the fire zone (between incline and decline) and the target are both on/near that same top lane, consistent with a ~10 m line-of-sight between them. The **acceleration road is a fully separate straight track** (own start/end markers), not connected to the main loop, confirmed visually.

- Parkour width: 3 m, continuous red/white side barriers 80 cm ±10 cm high (including turns) — **two tight U-turns within a 3 m corridor is a real geometry constraint** given the spec's 5 m turning radius requirement; worth checking this actually closes given real chassis dimensions once measured.
- Straight sections: 20 m
- Each stage marked with a 60 cm road sign (Figure 2, PDF page 15 — actually viewed): a plain, high-contrast pattern — red ring, white fill, black Arial Black digit centered. Simple enough that a color-threshold detector (red ring + white interior) is likely sufficient without needing OCR on the digit, since the stage sequence is fixed and known in advance — a coarse "sign ahead" trigger plus a stage counter may be enough, with digit-reading as a nice-to-have cross-check, not a hard requirement.

## Stage-by-stage requirements & scoring
| Stage | Requirement | Points |
|---|---|---|
| Fording & Shower | Ford 40 cm water depth *while simultaneously* passing under overhead shower heads (Fig. 3, PDF page 15 — actually viewed: rain and water crossing happen at the same time, not sequentially — sensors/electronics face both threats together) | 10 |
| Aggregate road | Traverse pebbles ≤5 cm dia., aggregate, soil | 5 |
| Side slope | Move forward across a 20% side slope without losing ground contact | 5 |
| Vertical obstacle | Climb up and safely down a 15 cm × 250 cm obstacle | 5 |
| Traffic cones | Navigate without contact; cones 40±10 cm base, 75±5 cm tall | 50 (−5/contact) |
| Sliding doors | 1 m wide door, slides fully out-of-parkour and back, **20 m/s** — pass without contact while detecting position/direction | 50 (pass without contact) |
| Bumped road | Straddle bump pairs so both left and right wheels/tracks contact their own bump line (Fig. 4: 5 cm × 20 cm base bumps) | 5 (−5 for single-line contact) |
| Incline/decline (45% grade) | Halt statically ≥2 s at marked point on incline and on decline | 15 (with proper 2 s stop, no slipping) / 10 (without) — **the spec PDF contradicts itself here**: §6.10's narrative says failing to stop 2s gets "zero points," but Table 5 (§9) explicitly lists 10 points for "without stopping." Worth an official Q&A question; in the meantime the incentive is unambiguous either way (always attempt the stop — 15 > 10 > 0 under any reading) |
| Laser fire / designation | Aim at 10 m A3 target (Fig. 5, PDF page 17 — actually viewed: concentric circles, radii **6 cm / 12 cm / 18 cm** exactly, a clean 1:2:3 ratio, alternating black/white/black for visual contrast — total target diameter 36 cm, fits an A3 sheet); laser only on during "fire," must hold designation 1 s; vehicle and laser director must not move while laser active | Inner 50 / Middle 25 / Outer 15; **−10 if anything moves while laser active**; landing on a ring boundary is graded as the higher-scoring ring |
| Target detection (autonomous) | Autonomously detect the fire point | 10 |
| Acceleration road | Separate 30 m straight, 0-to-end timed, ranked against other teams | 25 (1st place) down to 5 (5th place), 0 after 6th |
| Total run time | Fastest completion, per run, ranked against other teams | 100 (1st place) down to 5 (20th place), 0 after 21st |

**3 fire attempts allowed** (across runs; not obligated to use all 3) — only the single best-scoring attempt counts. This meaningfully de-risks the laser stage: it doesn't need to land perfectly on the first try.

**Completing Run 1 is an explicit minimum requirement for reward-ranking eligibility** ("Teams are expected to be able to complete the 1st run in order to enter the reward ranking," §8) — this isn't just the more time-efficient priority, it's a hard gate. A team that never gets Run 1 to a complete state cannot be ranked for prize money regardless of Run 2 performance.

⚠️ **The 20 m/s sliding-door speed is confirmed as literally written in the official PDF** (§6.8: "continuously slides right-to-left and left-to-right with 20 m/s") — not a transcription error introduced by this doc. It's still worth an official Q&A question given how physically extreme that is (~72 km/h for a parkour-scale sliding door), but don't assume it's a typo without confirmation — design the door-detection timing budget around the stated value unless/until corrected.

⚠️ **"Right to pass-on" may be ONE total for the entire final competition, not one per run.** The PDF's exact wording (§ following Table 5): "all participant teams has the right to pass on **only one stage of the competition parkour** during **the final competition**" — singular stage, and "the final competition" (both runs) rather than "each run." This doc previously assumed one pass-on per run (two total). Confirm via Q&A; until then, plan conservatively as if only one pass-on exists across both runs combined, since that changes whether it's safe to bank the door-stage pass-on for Run 2 (see CLAUDE.md).

**5-minute self-trial window** is provided before competing, for configuring/testing the vehicle's motion capabilities on-site (§6.13) — worth having a short pre-flight checklist ready (arm/disarm sanity, stick centering, gimbal range check) to use that window well.

## Vehicle physical requirements
- **Length:** 1.2–2.0 m. **Width:** 0.75–2.0 m. **Height:** ≥0.4 m and <1.25× width (excluding laser director/antennas). This is a large ground vehicle, not a small hobby robot.
- Wheeled (≥4 wheels, all-wheel-traction not required) or tracked.
- Must climb 45% (24.5°) slopes, cross a 15 cm vertical obstacle, hold position on a 20% side slope, 5 m turning radius, stay inside the 3 m corridor while turning.
- Brakes must hold static on a 45% incline/decline; hydraulic or mechanical actuation, normally closed.
- Body: all wiring/electronics enclosed (or sealed if external), rain/water-resistant, no sharp protrusions, headlights required, no silicone-adhesive-only joints on critical parts.

## Required subsystems not yet in this repo's hardware plan
The current `CLAUDE.md` hardware table only accounts for autonomous lidar/camera navigation. The spec requires substantially more:

1. **Laser designator + aiming mechanism** — a pan/tilt (or similar) mount carrying a laser module and a dedicated aiming camera, precise enough to hit a target from 10 m with a spot ≤1.5 cm diameter. Nothing like this exists in the current plan.
2. **Three cameras minimum**: front drive, rear drive, aiming — vs. the single Pi Camera v3 currently planned.
3. **Remote/manual control path (Run 1)** — an RC link or command-console link with live video feedback for a human operator; the current plan is autonomous-only (Nav2/ESP32 `/cmd_vel`) with no teleop path defined.
4. **Independent safety subsystem** — spec requires e-stop and comms-loss detection hardware/software to be *separate from the main vehicle control unit*, so a fault in the main computer can't defeat the e-stop. Current plan has no separate safety module — the ESP32 is both real-time control and (implicitly) safety, which doesn't meet this requirement as written.
5. **Drivetrain sizing** — climbing a 45% grade and fording water at this vehicle scale (1.2–2 m, likely 30–100+ kg) needs far more torque/power than typical hobby BLDC+ESC tank-drive setups. The current "2× bidirectional BLDC + ESC" entry needs real specs (motor kV, gear reduction, ESC current rating) before it can be validated against this requirement.
6. **Data recording** — front/rear/aim camera footage must be recorded and submitted (Data Package, §6.14).

These are flagged, not resolved — see Open Decisions in `CLAUDE.md`.

## Timeline (from Table 2)
| Date | Milestone |
|---|---|
| 2026-02-20 | Competition application deadline |
| 2026-03-31 17:00 | Technical Competency Form (TCF) due |
| 2026-04-14 | TCF results announced |
| 2026-06-01 17:00 | Critical Design Report (CDR) due — 25 pages max |
| 2026-06-22 | CDR results announced |
| 2026-07-20 17:00 | Vehicle Proofing Video (VPV) due — YouTube, unlisted, 2–5 min, ≥720p |
| 2026-07-31 | Finalists announced |
| 2026-08 – 2026-09 | Technical presentations + Final Competition |
| 2026-09-30 – 2026-10-04 | TEKNOFEST Şanlıurfa (event dates) |

## Safety/compliance checklist (spec §7.8, applies before final competition technical check)
- E-stop halts all motion unconditionally; comms loss auto-triggers e-stop
- Safety-monitoring hardware/software isolated from main control unit
- BMS on any Li-ion propulsion battery (overcurrent/short/over-temp/overvoltage/over-discharge protection)
- All power lines overcurrent/short-circuit protected
- No exposed wiring/connections anywhere on the vehicle
