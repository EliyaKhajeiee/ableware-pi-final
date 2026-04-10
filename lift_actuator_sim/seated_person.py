"""
Seated person model for the wheelchair sling-lift simulation.

Builds a simplified human figure sitting in the wheelchair with a
waist harness that connects to the sling.  All body parts are visual-only
PyBullet bodies that can be animated upward when the actuators lift.

Usage:
    from seated_person import SeatedPerson

    person = SeatedPerson(seat_z, seat_h, seat_w, seat_d)
    person.build()                     # create all PyBullet geometry
    person.update(extension=0.015)     # move person up by 15 mm
"""

import math
import pybullet as p


# ── Geometry constants (metres, Z-up) ─────────────────────────────────────

PELVIS_H      = 0.12                       # pelvis block height
TORSO_H       = 0.30                       # chest/torso height
TORSO_W       = 0.34                       # shoulder width
TORSO_D       = 0.20                       # front-to-back depth
SHOULDER_W    = TORSO_W / 2 + 0.02        # shoulder joint Y offset from centre
NECK_H        = 0.06
NECK_R        = 0.042
HEAD_R        = 0.092                      # head sphere radius
UPPER_ARM_L   = 0.26
FOREARM_L     = 0.24
ARM_R         = 0.032                      # limb cylinder radius
HAND_R        = 0.028
THIGH_R       = 0.052
SHIN_R        = 0.038
FOOT_HW       = [0.095, 0.042, 0.025]     # foot half-extents

# Waist harness
HARNESS_W     = TORSO_W + 0.04            # harness wraps wider than torso
HARNESS_H     = 0.065                      # belt height
HARNESS_T     = 0.012                      # belt thickness
HARNESS_STRAP_W = 0.038                    # side strap width


# ── Colour palette ────────────────────────────────────────────────────────

C_SKIN      = [0.78, 0.62, 0.50, 1.0]     # warm medium skin tone
C_SKIN_DK   = [0.65, 0.50, 0.40, 1.0]     # slightly darker (hands, neck)
C_HAIR      = [0.14, 0.10, 0.08, 1.0]     # dark hair
C_SHIRT     = [0.22, 0.38, 0.58, 1.0]     # muted blue shirt
C_PANTS     = [0.18, 0.20, 0.26, 1.0]     # dark grey trousers
C_SHOE      = [0.12, 0.12, 0.14, 1.0]     # dark shoes
C_HARNESS   = [0.92, 0.42, 0.08, 1.0]     # high-vis orange belt
C_HARNESS_S = [0.82, 0.36, 0.06, 1.0]     # darker side straps
C_BUCKLE    = [0.72, 0.74, 0.78, 1.0]     # steel buckle


# ── PyBullet visual-only helpers ──────────────────────────────────────────

def _q(roll=0.0, pitch=0.0, yaw=0.0):
    return p.getQuaternionFromEuler([roll, pitch, yaw])


def _vbox(half, pos, orn=None, color=None):
    orn = orn or _q()
    vis = (p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=color)
           if color else -1)
    return p.createMultiBody(0.001, -1, vis, pos, orn)


def _vcyl(r, h, pos, orn=None, color=None):
    orn = orn or _q()
    vis = (p.createVisualShape(p.GEOM_CYLINDER, radius=r, length=h, rgbaColor=color)
           if color else -1)
    return p.createMultiBody(0.001, -1, vis, pos, orn)


def _vsph(r, pos, color=None):
    vis = (p.createVisualShape(p.GEOM_SPHERE, radius=r, rgbaColor=color)
           if color else -1)
    return p.createMultiBody(0.001, -1, vis, pos)


def _move(body_id, pos, orn=None):
    p.resetBasePositionAndOrientation(body_id, pos, orn or _q())


# ── Main class ────────────────────────────────────────────────────────────

class SeatedPerson:
    """
    Visual model of a person sitting in a wheelchair with a waist harness.

    Call build() after PyBullet is initialised, then update(ext) each frame
    to raise or lower the person with the sling.

    Args:
        seat_z:    height of the sitting surface (top of sling at rest)
        seat_h:    seat frame height from floor (used for armrest reference)
        seat_w:    seat width (used for armrest / thigh spacing)
        seat_d:    seat depth front-to-back (used for thigh / leg placement)
    """

    def __init__(self, seat_z: float, seat_h: float,
                 seat_w: float, seat_d: float,
                 harness: bool = True):
        self._seat_z = seat_z       # person sits here (sling top surface)
        self._seat_h = seat_h
        self._seat_w = seat_w
        self._seat_d = seat_d
        self._harness = harness

        self._body_parts: list = []     # (body_id, x, y, z_offset_from_seat)
        self._harness_parts: list = []  # same format

    # ── Build ─────────────────────────────────────────────────────────────

    def build(self) -> None:
        """Create all PyBullet visual bodies for the person and harness."""
        z0 = self._seat_z
        seat_h = self._seat_h
        seat_w = self._seat_w
        seat_d = self._seat_d

        self._body_parts = []
        self._harness_parts = []

        def _add(bid, x, y, z_abs):
            self._body_parts.append((bid, x, y, z_abs - z0))
            return bid

        def _add_h(bid, x, y, z_abs):
            self._harness_parts.append((bid, x, y, z_abs - z0))
            return bid

        # ── Pelvis / hips ─────────────────────────────────────────────
        pelvis_z = z0 + PELVIS_H / 2
        _add(_vbox([TORSO_D / 2 - 0.01, TORSO_W / 2 + 0.01, PELVIS_H / 2],
                    [0, 0, pelvis_z], color=C_PANTS), 0, 0, pelvis_z)

        # ── Torso / chest (leaning back ~5°) ──────────────────────────
        torso_z = z0 + PELVIS_H + TORSO_H / 2
        torso_pitch = -0.09
        _add(_vbox([TORSO_D / 2, TORSO_W / 2, TORSO_H / 2],
                    [-0.02, 0, torso_z],
                    _q(0, torso_pitch, 0), color=C_SHIRT),
             -0.02, 0, torso_z)

        # ── Neck ──────────────────────────────────────────────────────
        neck_z = z0 + PELVIS_H + TORSO_H + NECK_H / 2
        _add(_vcyl(NECK_R, NECK_H,
                   [-0.03, 0, neck_z], color=C_SKIN_DK),
             -0.03, 0, neck_z)

        # ── Head + hair ───────────────────────────────────────────────
        head_z = z0 + PELVIS_H + TORSO_H + NECK_H + HEAD_R
        _add(_vsph(HEAD_R, [-0.03, 0, head_z], color=C_SKIN),
             -0.03, 0, head_z)
        hair_z = head_z + HEAD_R * 0.25
        _add(_vsph(HEAD_R * 0.88, [-0.035, 0, hair_z], color=C_HAIR),
             -0.035, 0, hair_z)

        # ── Arms (both sides) ─────────────────────────────────────────
        shoulder_z = z0 + PELVIS_H + TORSO_H - 0.03
        armrest_z  = seat_h + 0.240
        elbow_z    = (shoulder_z + armrest_z) / 2 + 0.02

        for sign in (+1, -1):
            sy = sign * SHOULDER_W

            # Shoulder joint
            _add(_vsph(ARM_R * 1.5,
                       [-0.02, sy, shoulder_z], color=C_SHIRT),
                 -0.02, sy, shoulder_z)

            # Upper arm
            ua_len = shoulder_z - elbow_z
            ua_cz  = (shoulder_z + elbow_z) / 2
            _add(_vcyl(ARM_R, ua_len,
                       [-0.01, sy, ua_cz], color=C_SHIRT),
                 -0.01, sy, ua_cz)

            # Elbow
            _add(_vsph(ARM_R * 1.3,
                       [0.0, sy, elbow_z], color=C_SKIN),
                 0.0, sy, elbow_z)

            # Forearm resting on armrest
            forearm_cx = FOREARM_L / 2 * 0.6
            armrest_y  = sign * (seat_w / 2 + 0.020)
            fa_cy      = (sy + armrest_y) / 2
            _add(_vcyl(ARM_R * 0.9, FOREARM_L,
                       [forearm_cx, fa_cy, elbow_z],
                       _q(0, math.pi / 2 - 0.15, 0), color=C_SKIN),
                 forearm_cx, fa_cy, elbow_z)

            # Hand
            hand_x = forearm_cx + FOREARM_L / 2 - 0.02
            _add(_vsph(HAND_R,
                       [hand_x, armrest_y, elbow_z - 0.01], color=C_SKIN_DK),
                 hand_x, armrest_y, elbow_z - 0.01)

        # ── Thighs (horizontal on seat) ───────────────────────────────
        hip_x    = -seat_d / 2 + 0.12
        knee_x   =  seat_d / 2 - 0.06
        thigh_cx = (hip_x + knee_x) / 2
        thigh_len = knee_x - hip_x

        for sign in (+1, -1):
            ty = sign * 0.10

            _add(_vcyl(THIGH_R, thigh_len,
                       [thigh_cx, ty, z0 + 0.04],
                       _q(0, math.pi / 2, 0), color=C_PANTS),
                 thigh_cx, ty, z0 + 0.04)

            # Knee
            _add(_vsph(THIGH_R * 1.05,
                       [knee_x, ty, z0 + 0.04], color=C_PANTS),
                 knee_x, ty, z0 + 0.04)

            # Shin (angling down to footrest)
            foot_z      = 0.143 + FOOT_HW[2]
            shin_top    = z0 + 0.04
            shin_cz     = (shin_top + foot_z + 0.08) / 2
            shin_actual = math.sqrt(0.08**2 + (shin_top - foot_z - 0.08)**2)
            shin_angle  = math.atan2(0.08, shin_top - foot_z - 0.08)
            _add(_vcyl(SHIN_R, shin_actual,
                       [knee_x + 0.04, ty, shin_cz],
                       _q(0, shin_angle, 0), color=C_PANTS),
                 knee_x + 0.04, ty, shin_cz)

            # Foot
            foot_x = knee_x + 0.08
            _add(_vbox(FOOT_HW,
                       [foot_x, ty, foot_z], color=C_SHOE),
                 foot_x, ty, foot_z)

        # ── Waist harness (optional) ───────────────────────────────────
        if not self._harness:
            return

        harness_z = z0 + PELVIS_H + 0.01

        # Belt
        _add_h(_vbox([TORSO_D / 2 + 0.008, HARNESS_W / 2, HARNESS_H / 2],
                      [0, 0, harness_z], color=C_HARNESS),
               0, 0, harness_z)

        # Front buckle
        _add_h(_vbox([0.018, 0.032, HARNESS_H / 2 + 0.003],
                      [TORSO_D / 2 + 0.012, 0, harness_z], color=C_BUCKLE),
               TORSO_D / 2 + 0.012, 0, harness_z)

        # Side mounting plates (where actuator shaft bolts to belt)
        for sign in (+1, -1):
            plate_y = sign * (HARNESS_W / 2 - 0.005)

            # Reinforced side plate
            _add_h(_vbox([0.035, 0.014, HARNESS_H / 2 + 0.008],
                          [0, plate_y, harness_z], color=C_HARNESS_S),
                   0, plate_y, harness_z)

            # Steel bolt plate (actuator attachment point)
            _add_h(_vbox([0.025, 0.008, 0.018],
                          [-0.01, plate_y, harness_z], color=C_BUCKLE),
                   -0.01, plate_y, harness_z)

    # ── Animation ─────────────────────────────────────────────────────────

    def update(self, extension: float) -> None:
        """
        Move the person and harness upward by the given actuator extension.

        Args:
            extension: current actuator extension in metres (0 = at rest)
        """
        z0 = self._seat_z
        for bid, ox, oy, z_off in self._body_parts:
            _move(bid, [ox, oy, z0 + z_off + extension])
        for bid, ox, oy, z_off in self._harness_parts:
            _move(bid, [ox, oy, z0 + z_off + extension])