"""Open underarm harness lift device.

Read this file when you want to change the custom lift device shape.

The simulator uses three kinds of PyBullet objects:
- visual-only boxes for cosmetic parts
- collision boxes for parts that the figure can touch
- debug lines/text for labels and force arrows

The main simulator owns actuator force/control. This module only builds and
moves the harness geometry.
"""

import pybullet as p

from core.sim_constants import (
    ACT_STROKE,
    C_ACT_DEAD,
    C_ACT_HOUSE,
    C_ACT_OK,
    C_ACT_ROD,
    C_ACT_WARN,
    C_FRAME_DK,
    C_FRAME_MD,
    C_LABEL_Y,
    C_RULER,
    C_SLING_DN,
    C_SLING_UP,
    C_TIE,
    RULER_X,
)
from core.sim_pybHelper import _q


# ── DEVICE SHAPE EDIT BLOCK ───────────────────────────────────────────────
# Change the underarm harness shape here.
#
# Coordinate reminder:
#   X = front/back, Y = left/right, Z = up/down.
#
# The two blue collision pads are the real lifting surfaces. The black frame
# bars also have collision. The center stays open so the torso fits inside.
DEVICE_SHAPE = {
    # Move the whole device without changing its shape.
    # X = front/back, Y = left/right, Z = up/down.
    # Example: [0.05, 0.00, 0.02] moves the device forward and slightly up.
    "offset_xyz": [0.0, 0.0, 0.1],

    # Underarm hook/collision pad.
    "pad_x": 0.02,             # Front/back location of both underarm pads.
    "pad_y_offset": 0.145,     # Left/right pad center spacing from body center.
    "pad_z_rest": 0.50,        # Pad center height when lift target is zero.
    "pad_half_x": 0.135,       # Pad half-length front/back; larger = longer hook.
    "pad_half_y": 0.005,       # Pad half-width left/right; larger = wider hook.
    "pad_half_z": 0.026,       # Pad half-thickness.
    "lift_gain": 3.35,         # Pad travel multiplier from actuator extension.

    # Visual open-frame harness around the collision pads.
    "side_y": 0.290,           # Side module spacing; larger = wider device interval.
    "back_x": -0.055,          # Rear/back strap X position behind the torso.
    "front_x": 0.080,          # Forward end of the shoulder guide.
    "shoulder_z_offset": 0.185,  # Height from pad center to shoulder guide.
    "label_z": 1.42,
}


def _visual_box(half, pos, orn=None, color=None):
    """Create a kinematic visual-only box."""
    orn = orn or _q()
    visual = (
        p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=color)
        if color
        else -1
    )
    return p.createMultiBody(0.0, -1, visual, pos, orn)


def _collision_box(half, pos, orn=None, color=None):
    """Create a kinematic box with both visual and collision geometry."""
    orn = orn or _q()
    collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)
    visual = (
        p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=color)
        if color
        else -1
    )
    return p.createMultiBody(0.0, collision, visual, pos, orn)


def _move(body_id, pos, orn=None):
    p.resetBasePositionAndOrientation(body_id, pos, orn or _q())


def _recolor(body_id, rgba):
    p.changeVisualShape(body_id, -1, rgbaColor=rgba)


class UnderarmHarness:
    """Builds and animates the open underarm harness device."""

    def __init__(self, shape=None):
        self.shape = shape or DEVICE_SHAPE
        self.moving_visual_parts = []     # (body_id, x, y, z_offset, orn, color_mode)
        self.collision_parts = []         # (body_id, x, y, z_offset)
        self.force_arrows = {}
        self.indicator_line_id = -1
        self.sling_id = None              # Back-compat name used by old code/tests.

    def build(self):
        """Build every harness body once at rest height."""
        self._build_visual_frame()
        self._build_collision_hooks()
        return self

    def _offset(self):
        """Return the global device offset from DEVICE_SHAPE."""
        return self.shape.get("offset_xyz", [0.0, 0.0, 0.0])

    def _pos(self, x, y, z):
        """Apply the device offset to one local harness position."""
        ox, oy, oz = self._offset()
        return [x + ox, y + oy, z + oz]

    def _add_moving_visual_box(self, half, pos, color, z_offset, color_mode, orn=None):
        """Add one visual frame piece that moves together with the lift."""
        orn = orn or _q()

        # Black frame bars collide with the figure.
        # Other cosmetic pieces stay visual-only.
        if color_mode == "frame":
            body_id = _collision_box(half, pos, orn, color)
            p.changeDynamics(
                body_id,
                -1,
                lateralFriction=1.2,
                spinningFriction=0.03,
                rollingFriction=0.01,
                restitution=0.0,
            )
        else:
            body_id = _visual_box(half, pos, orn, color)

        self.moving_visual_parts.append(
            (body_id, pos[0], pos[1], z_offset, orn, color_mode)
        )
        return body_id

    def _build_visual_frame(self):
        """Build the visible open frame around the armpit hooks."""
        shape = self.shape
        support_z = shape["pad_z_rest"]
        shoulder_z = support_z + shape["shoulder_z_offset"]

        # Build symmetric left/right sides. side=+1 and side=-1 mirror the Y axis.
        for side in (1.0, -1.0):
            side_y = side * shape["side_y"]
            pad_y = side * shape["pad_y_offset"]
            reach_center_y = (side_y + pad_y) / 2.0
            reach_half_y = abs(side_y - pad_y) / 2.0

            # Outer rectangular side module beside the ribs.
            self._add_moving_visual_box(
                [0.036, 0.030, 0.150],
                self._pos(0.025, side_y, support_z + 0.060),
                C_ACT_HOUSE,
                0.060,
                "carriage",
            )
            # Inner guide/rod drawn inside the side module.
            self._add_moving_visual_box(
                [0.010, 0.034, 0.126],
                self._pos(0.030, side_y, support_z + 0.065),
                C_ACT_ROD,
                0.065,
                "active",
            )
            # Short side bridge. It does not cross the torso center.
            self._add_moving_visual_box(
                [0.018, reach_half_y, 0.018],
                self._pos(shape["pad_x"], reach_center_y, support_z + 0.020),
                C_FRAME_MD,
                0.020,
                "active",
            )
            # Visual hook lip. The physical collision hook is built separately.
            self._add_moving_visual_box(
                [0.030, 0.012, 0.070],
                self._pos(
                    shape["pad_x"] - 0.012,
                    side * (shape["pad_y_offset"] + shape["pad_half_y"] + 0.010),
                    support_z + 0.058,
                ),
                C_FRAME_MD,
                0.058,
                "active",
            )
            # Black rear upright. This has collision.
            self._add_moving_visual_box(
                [0.016, 0.020, 0.150],
                self._pos(shape["back_x"], side_y, support_z + 0.130),
                C_FRAME_DK,
                0.130,
                "frame",
            )
            # Black shoulder guide. This has collision.
            self._add_moving_visual_box(
                [0.052, 0.020, 0.018],
                self._pos((shape["back_x"] + shape["front_x"]) / 2.0, side_y, shoulder_z),
                C_FRAME_DK,
                shape["shoulder_z_offset"],
                "frame",
                _q(0.0, -0.45, 0.0),
            )

        # Black rear cross strap behind the torso. This has collision.
        self._add_moving_visual_box(
            [0.016, shape["side_y"], 0.018],
            self._pos(shape["back_x"], 0.0, support_z + 0.145),
            C_FRAME_DK,
            0.145,
            "frame",
        )

    def _build_collision_hooks(self):
        """Build the physical armpit pads and hook lips."""
        shape = self.shape
        z0 = shape["pad_z_rest"]

        for sy in (shape["pad_y_offset"], -shape["pad_y_offset"]):
            side = 1.0 if sy > 0 else -1.0

            # Blue horizontal support pad under the armpit.
            pad = _collision_box(
                [shape["pad_half_x"], shape["pad_half_y"], shape["pad_half_z"]],
                self._pos(shape["pad_x"], sy, z0),
                color=C_SLING_DN,
            )
            p.changeDynamics(
                pad,
                -1,
                lateralFriction=1.5,
                spinningFriction=0.04,
                rollingFriction=0.02,
                restitution=0.0,
            )
            pad_x, pad_y, _ = self._pos(shape["pad_x"], sy, z0)
            self.collision_parts.append((pad, pad_x, pad_y, 0.0))

            # Vertical hook wall so the arm does not slide off the pad.
            lip_y = sy + side * (shape["pad_half_y"] + 0.012)
            lip = _collision_box(
                [shape["pad_half_x"] * 0.82, 0.012, 0.074],
                self._pos(shape["pad_x"] - 0.012, lip_y, z0 + 0.060),
                color=C_FRAME_MD,
            )
            p.changeDynamics(
                lip,
                -1,
                lateralFriction=1.8,
                spinningFriction=0.05,
                rollingFriction=0.02,
                restitution=0.0,
            )
            lip_x, lip_y_offset, _ = self._pos(shape["pad_x"] - 0.012, lip_y, z0 + 0.060)
            self.collision_parts.append((lip, lip_x, lip_y_offset, 0.060))

        self.sling_id = self.collision_parts[0][0] if self.collision_parts else None

    def support_height(self, extension):
        """Return (support_top_z, support_center_z) for an actuator extension."""
        shape = self.shape
        _, _, offset_z = self._offset()
        top_z = (
            shape["pad_z_rest"]
            + shape["pad_half_z"]
            + extension * shape["lift_gain"]
            + offset_z
        )
        center_z = top_z - shape["pad_half_z"]
        return top_z, center_z

    def update(self, extension, stalled, overloaded, at_target, force_per_act):
        """Move and recolor the harness for the current actuator extension."""
        arm_c, pad_c, arrow_c = self._colors_for_state(extension, stalled, overloaded, at_target)
        support_top_z, support_center_z = self.support_height(extension)

        for body_id, x, y, z_offset, orn, color_mode in self.moving_visual_parts:
            _move(body_id, [x, y, support_center_z + z_offset], orn)
            if color_mode == "active":
                _recolor(body_id, arm_c)
            elif color_mode == "strap":
                _recolor(body_id, C_TIE)
            elif color_mode == "frame":
                _recolor(body_id, C_FRAME_DK)
            else:
                _recolor(body_id, C_ACT_HOUSE)

        self._update_force_arrows(support_top_z, force_per_act, arrow_c)

        for body_id, x, y, z_offset in self.collision_parts:
            _move(body_id, [x, y, support_center_z + z_offset])
            _recolor(body_id, pad_c)

    def _colors_for_state(self, extension, stalled, overloaded, at_target):
        if stalled or overloaded:
            return C_ACT_DEAD, C_SLING_DN, [0.95, 0.15, 0.10]
        if at_target and extension > 0.002:
            return C_ACT_OK, C_SLING_UP, [0.18, 0.90, 0.30]
        if extension > ACT_STROKE * 0.72:
            return C_ACT_WARN, C_SLING_DN, [0.95, 0.70, 0.10]
        if extension > 0.002:
            return C_ACT_OK, C_SLING_DN, [0.18, 0.90, 0.30]
        return C_ACT_ROD, C_SLING_DN, [0.55, 0.55, 0.60]

    def _update_force_arrows(self, support_top_z, force_per_act, arrow_color):
        arrow_len = min(0.30, force_per_act / 100.0 * 0.05)
        for i, sy in enumerate((self.shape["pad_y_offset"], -self.shape["pad_y_offset"])):
            x, y, _ = self._pos(self.shape["pad_x"], sy, 0.0)
            base = [x, y, support_top_z + 0.010]
            tip = [x, y, support_top_z + arrow_len]
            key = f"arr{i}"
            if key in self.force_arrows:
                p.addUserDebugLine(
                    base,
                    tip,
                    arrow_color,
                    lineWidth=4,
                    replaceItemUniqueId=self.force_arrows[key],
                )
            else:
                self.force_arrows[key] = p.addUserDebugLine(
                    base, tip, arrow_color, lineWidth=4, lifeTime=0
                )

    def build_labels(self):
        ox, oy, oz = self._offset()
        p.addUserDebugText(
            "DUAL UNDERARM HARNESS LIFT SYSTEM",
            [ox, oy, self.shape["label_z"] + oz],
            textColorRGB=[0.90, 0.92, 0.95],
            textSize=1.10,
            lifeTime=0,
        )
        p.addUserDebugText(
            "PA-14 Class  |  4-inch Stroke  |  12 / 24 V DC  |  Paired Side Modules",
            [ox, oy, self.shape["label_z"] - 0.12 + oz],
            textColorRGB=C_LABEL_Y,
            textSize=0.75,
            lifeTime=0,
        )

    def build_lift_indicator(self):
        ox, _, oz = self._offset()
        ruler_x = RULER_X + ox
        z_bot = self.shape["pad_z_rest"] + self.shape["pad_half_z"] + oz
        z_top = z_bot + ACT_STROKE * self.shape["lift_gain"]

        p.addUserDebugLine([ruler_x, 0, z_bot], [ruler_x, 0, z_top], C_RULER, lineWidth=3, lifeTime=0)

        total_rise_in = (z_top - z_bot) * 39.37
        for frac, label in (
            (0.0, "0%  (seated)"),
            (0.25, "25%"),
            (0.50, "50%"),
            (0.75, "75%"),
            (1.0, f"100% ({total_rise_in:.1f} in)"),
        ):
            zt = z_bot + frac * (z_top - z_bot)
            p.addUserDebugLine([ruler_x - 0.016, 0, zt], [ruler_x + 0.016, 0, zt], C_RULER, lineWidth=2, lifeTime=0)
            p.addUserDebugText(
                label,
                [ruler_x + 0.022, 0, zt],
                textColorRGB=C_RULER,
                textSize=0.62,
                lifeTime=0,
            )

    def update_indicator(self, extension):
        t = extension / max(ACT_STROKE, 1e-9)
        ox, _, oz = self._offset()
        ruler_x = RULER_X + ox
        z_bot = self.shape["pad_z_rest"] + self.shape["pad_half_z"] + oz
        z_top = z_bot + ACT_STROKE * self.shape["lift_gain"]
        z_ind = z_bot + t * (z_top - z_bot)

        ind_from = [ruler_x, 0, z_ind]
        ind_to = [ruler_x - 0.032, 0, z_ind]
        if self.indicator_line_id >= 0:
            self.indicator_line_id = p.addUserDebugLine(
                ind_from,
                ind_to,
                [1.0, 0.85, 0.0],
                lineWidth=5,
                replaceItemUniqueId=self.indicator_line_id,
                lifeTime=0,
            )
        else:
            self.indicator_line_id = p.addUserDebugLine(
                ind_from, ind_to, [1.0, 0.85, 0.0], lineWidth=5, lifeTime=0
            )
