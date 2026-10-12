#include "motor_mixer.h"
#include "config.h"
#include <algorithm>
#include <cmath>

// AUTONOMOUS-ONLY function: mixDifferential() is called exactly once in
// loop(), inside the autonomousMode branch, to turn the Pi's /cmd_vel into
// per-wheel outputs. MANUAL driving never calls this - it mixes each stick
// directly onto its own track. That makes this the correct, and only,
// place to put an autonomous-specific rule without touching manual at all.
//
// 2026-09-11, final correction after two earlier attempts at this same
// question (see CLAUDE.md for the full history - each earlier version is
// preserved there, not repeated here): explicit report that the robot was
// visibly backing up mid-course in AUTONOMOUS mode ("the robot is going
// backward, not in the parkour way") - one wheel reversing to pivot during
// a turn, technically correct differential-drive kinematics, but wrong for
// navigating an obstacle course that should only ever advance and turn,
// never retreat. Reverse is banned outright for the autonomous path only:
// both wheels are clamped to [0, ...], full stop, regardless of what the
// mix computes. This trades away the tight, less-than-track-separation
// pivot radius (see the reverse-assisted-pivot discussion in CLAUDE.md)
// for a vehicle that never moves backward on its own - an explicit,
// deliberate choice for this specific failure mode, not a step back to
// the earlier "no reverse from steering" fix (that one still allowed a
// negative *linear* command through; this one does not, since the
// vehicle should never autonomously decide to go backward at all).
// MANUAL driving is completely unaffected - a human pulling a stick back
// still gets full reverse, protected by applyReverseCommitment()'s
// minimum-hold-time in esp32_firmware.ino.
MotorOutputs mixDifferential(float linear, float angular) {
    MotorOutputs out;
    out.left  = std::max(0.0f, linear - angular);
    out.right = std::max(0.0f, linear + angular);

    float maxMag = std::max(1.0f, std::max(std::fabs(out.left), std::fabs(out.right)));
    out.left  /= maxMag;
    out.right /= maxMag;
    return out;
}
