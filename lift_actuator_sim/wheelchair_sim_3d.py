#!/usr/bin/env python3
"""
3D Wheelchair Underarm Lift Simulation — PyBullet (Demo Edition)

Simulates the assistive underarm-lift device:
  - Manual wheelchair with tubular steel frame and spoked wheels
  - Two padded supports that contact the user's armpit area
  - Two linear actuators drive the underarm yoke upward
  - The lift raises the upper body without pushing from below the seat

Physics driven by LinearActuator / LiftController / UserLoad modules.
PyBullet handles rendering and the interactive GUI sliders.

Usage:
    python3 wheelchair_sim_3d.py

Sliders (left panel):
    User Weight (kg)            — 40–220 kg; affects actuator load
    Lift Target (0→1)           — 0 = down, 1 = full 1-inch lift
    Max Force Per Actuator (N)  — lower this to see stall
    Speed Multiplier            — 0.1× slow-motion to 4× fast
"""

import sys
import os
import time
import math
import json as _json
import queue as _queue
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

import numpy as np


from keyInput import KeyReader

from UI import build_demo_scenarios_panel, create_demo_live, update_demo_live
from UI.interactive_screen_ui import (
    show_interactive_screen as show_interactive_live_screen,
    create_interactive_live,
    update_interactive_live,
)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import pybullet as p
import pybullet_data

from actuator import LinearActuator
from load import UserLoad
from controller import LiftController

from core.sim_constants import *
from core.sim_pybHelper import _q, _box, _cyl


# Visual-only (no collision) — for kinematic animated bodies

def _vbox(half, pos, orn=None, color=None):
    orn = orn or _q()
    vis = (p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=color)
           if color else -1)
    return p.createMultiBody(0.0, -1, vis, pos, orn)


def _vcyl(r, h, pos, orn=None, color=None):
    orn = orn or _q()
    vis = (p.createVisualShape(p.GEOM_CYLINDER, radius=r, length=h, rgbaColor=color)
           if color else -1)
    return p.createMultiBody(0.0, -1, vis, pos, orn)


def _vsph(r, pos, color=None):
    vis = (p.createVisualShape(p.GEOM_SPHERE, radius=r, rgbaColor=color)
           if color else -1)
    return p.createMultiBody(0.0, -1, vis, pos)


def _kbox(half, pos, orn=None, color=None):
    """Massless box with collision, used for kinematic lift-contact surfaces."""
    orn = orn or _q()
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)
    vis = (p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=color)
           if color else -1)
    return p.createMultiBody(0.0, col, vis, pos, orn)


def _tube(x0, y0, z0, x1, y1, z1, r=None, color=None):
    """Draw a cylinder tube from point A to point B."""
    r = r if r is not None else TUBE_R
    dx, dy, dz = x1 - x0, y1 - y0, z1 - z0
    length = math.sqrt(dx*dx + dy*dy + dz*dz)
    if length < 1e-6:
        return
    cx, cy, cz = (x0+x1)/2, (y0+y1)/2, (z0+z1)/2
    tx, ty, tz = dx/length, dy/length, dz/length
    cos_a = tz  # dot(Z_axis, target)
    if abs(cos_a - 1.0) < 1e-6:
        orn = _q()
    elif abs(cos_a + 1.0) < 1e-6:
        orn = (1.0, 0.0, 0.0, 0.0)
    else:
        ax, ay = -ty, tx
        n = math.sqrt(ax*ax + ay*ay)
        ax, ay = ax/n, ay/n
        a = math.acos(max(-1.0, min(1.0, cos_a)))
        s = math.sin(a / 2)
        orn = (ax*s, ay*s, 0.0, math.cos(a / 2))
    _cyl(r, length, [cx, cy, cz], orn, color)


def _move(body_id, pos, orn=None):
    p.resetBasePositionAndOrientation(body_id, pos, orn or _q())


def _recolor(body_id, rgba):
    p.changeVisualShape(body_id, -1, rgbaColor=rgba)


WHEEL_ORN = _q(math.pi / 2, 0, 0)   # lay cylinder on its side

ASSET_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets")
WHEELCHAIR_URDF = os.path.join(ASSET_DIR, "wheelchair_pybullet", "wheelchair.urdf")
LAY_FIGURE_URDF = os.path.join(ASSET_DIR, "lay_figure_pybullet", "lay_figure.urdf")
LAY_FIGURE_BASE_COLOR = os.path.join(
    ASSET_DIR, "lay_figure_pybullet", "textures", "lay_figure_BaseColor_2K.png")

WHEELCHAIR_MODEL_POS = [0.03, -0.02, 0.25]
WHEELCHAIR_MODEL_ORN = _q(0.0, 0.0, math.pi / 2.0)
WHEELCHAIR_MODEL_SCALE = 0.25

LAY_FIGURE_DROP_POS = [0.03, -0.02, 1.42]
LAY_FIGURE_ORN = _q(0.0, 0.0, math.pi / 2.0)
LAY_FIGURE_SCALE = 0.7
ENABLE_LAY_FIGURE_RAGDOLL = True

LAY_FIGURE_SITTING_POSE = {
    "body_joint": -0.08,
    "head_joint": 0.0,
    "l_thigh_joint": -1.32,
    "r_thigh_joint": -1.32,
    "l_shin_joint": 1.25,
    "r_shin_joint": 1.25,
    "l_ankle_joint": -0.2,
    "r_ankle_joint": -0.2,
    "l_foot_joint": 0.0,
    "r_foot_joint": 0.0,
}

# ── FIGURE ARM JOINT EDIT BLOCK ───────────────────────────────────────────
# These are the locked arm joint angles, in radians.
# Change these numbers if you want a new default pose.
FIGURE_ARM_JOINT_TARGETS = {
    "l_shoulder_joint": 0.55,
    "r_shoulder_joint": -0.55,
    "l_forearm_joint": 0.85,
    "r_forearm_joint": -0.85,
    "l_hand_joint": 0.0,
    "r_hand_joint": 0.0,
}
LAY_FIGURE_SITTING_POSE.update(FIGURE_ARM_JOINT_TARGETS)

# Keep only the arms controlled so the underarm hooks have stable contact.
# Body, head, and legs remain ragdoll/passive.
LAY_FIGURE_LOCKED_ARM_JOINTS = set(FIGURE_ARM_JOINT_TARGETS)
LAY_FIGURE_ARM_HOLD_FORCE = 1200.0

# ── DEVICE SHAPE EDIT BLOCK ───────────────────────────────────────────────
# Change the underarm harness shape here.
#
# Coordinate reminder:
#   X = front/back, Y = left/right, Z = up/down.
#
# The two collision pads are the real lifting surfaces. The visual harness is
# built around those pads, with an empty center so the torso fits inside.
DEVICE_SHAPE = {
    # Underarm hook/collision pad.
    "pad_x": 0.02,             # Front/back location of both underarm pads.
    "pad_y_offset": 0.145,     # Left/right pad center spacing from body center.
    "pad_z_rest": 0.50,        # Pad center height when lift target is zero.
    "pad_half_x": 0.135,       # Pad half-length front/back; larger = longer hook.
    "pad_half_y": 0.055,       # Pad half-width left/right; larger = wider hook.
    "pad_half_z": 0.026,       # Pad half-thickness.
    "lift_gain": 3.35,         # Pad travel multiplier from actuator extension.

    # Visual open-frame harness around the collision pads.
    "side_y": 0.290,           # Side module spacing; larger = wider device interval.
    "back_x": -0.055,          # Rear/back strap X position behind the torso.
    "front_x": 0.080,          # Forward end of the shoulder guide.
    "shoulder_z_offset": 0.185,  # Height from pad center to shoulder guide.
    "label_z": 1.42,
}


# ── Network API (used when running with --serve) ───────────────────────────

STEP_SIZE = 0.0254   # metres per UP/DOWN command (1 inch per click)

_cmd_queue: _queue.Queue = _queue.Queue()
_sim_ref = None      # holds the active WheelchairLiftSim3D instance in serve mode


class _SimAPIHandler(BaseHTTPRequestHandler):
    """Minimal HTTP handler exposing /command and /state for the Ableware hub."""

    def do_POST(self):
        if self.path != '/command':
            self._respond(404, {'error': 'not found'})
            return
        length = int(self.headers.get('Content-Length', 0))
        body = _json.loads(self.rfile.read(length)) if length else {}
        _cmd_queue.put(body.get('command', '').upper())
        self._respond(200, {'status': 'ok'})

    def do_GET(self):
        if self.path == '/state':
            sim = _sim_ref
            if sim is None:
                self._respond(503, {'error': 'sim not ready'})
                return
            act  = sim.act_L
            ctrl = sim.ctrl
            self._respond(200, {
                'position':          float(act.position),
                'velocity':          float(act.velocity),
                'acceleration':      float(act.acceleration),
                'pwm':               float(act.pwm_input),
                'emergency_stopped': bool(act.emergency_stopped),
                'stalled':           bool(act.stalled),
                'target_position':   float(ctrl.target_position),
                'at_target':         bool(ctrl.at_target()),
                'is_stable':         bool(ctrl.is_stable()),
            })
        elif self.path == '/health':
            self._respond(200, {'status': 'ok'})
        else:
            self._respond(404, {'error': 'not found'})

    def _respond(self, code: int, data: dict) -> None:
        body = _json.dumps(data).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *_):
        pass  # suppress default request logging


# ── Main simulation class ──────────────────────────────────────────────────

class WheelchairLiftSim3D:
    """
    Interactive 3D wheelchair underarm-lift simulation.

    LinearActuator + LiftController + UserLoad handle all physics.
    PyBullet provides rendering, real-time GUI, and visual feedback.
    """

    def __init__(self, headless: bool = False):
        self._headless = headless
        self._connect()

        # PyBullet gravity is needed for the seated ragdoll figure. The lift
        # actuator dynamics are still computed by the custom controller below.
        p.setGravity(0, 0, -9.81)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        if not self._headless:
            p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 0)
            p.resetDebugVisualizerCamera(
                cameraDistance=1.95,
                cameraYaw=36,
                cameraPitch=-22,
                cameraTargetPosition=[-0.02, 0.0, 0.66])

        self._init_physics(user_mass=80.0, max_force=750.0)

        self._build_floor()
        self._build_wheelchair()
        self._build_sling()
        self._build_actuators()
        self._build_lay_figure()

        if not self._headless:
            self._create_sliders()
            self._build_labels()
            self._build_lift_indicator()

        self._txt         = {}   # HUD text item IDs
        self._arrows      = {}   # force arrow debug-line IDs
        self._ind_line_id = -1   # animated lift-indicator line
        self._duty_on     = 0    # frames spent actively driving
        self._duty_total  = 0    # total frames (for duty-cycle %)


    # ── Connection ────────────────────────────────────────────────────────

    def _connect(self):
        if self._headless:
            self.client = p.connect(p.DIRECT)
            return
        try:
            self.client = p.connect(p.GUI)
        except Exception as exc:
            print(f"[ERROR] Cannot open PyBullet GUI: {exc}")
            print("Make sure a display is available (X11 / VNC / WSL).")
            sys.exit(1)

    # ── Physics init ──────────────────────────────────────────────────────

    def _init_physics(self, user_mass: float, max_force: float):
        self.user_mass = user_mass
        self.max_force = max_force
        self._lay_figure_mass_fractions = None
        self._lay_figure_arm_constraint_ids = []

        self.act_L = LinearActuator(
            max_force=max_force, stroke_length=ACT_STROKE,
            max_velocity=ACT_MAX_VEL, max_acceleration=ACT_MAX_ACC,
            acceleration_ramp_time=ACT_RAMP_T)
        self.act_R = LinearActuator(
            max_force=max_force, stroke_length=ACT_STROKE,
            max_velocity=ACT_MAX_VEL, max_acceleration=ACT_MAX_ACC,
            acceleration_ramp_time=ACT_RAMP_T)
        self.load = UserLoad(mass=user_mass / 2.0)
        self.ctrl = LiftController(
            actuator=self.act_L, load=self.load,
            kp=10.0, ki=0.12, kd=2.5,
            position_tolerance=0.002)   # 2 mm — realistic real-world tolerance
        # Lower ki reduces integrator windup on the long 4-inch approach.
        # Higher kd damps overshoot when decelerating near target.
        self.ctrl.set_target_position(0.0)

    def _set_user_mass(self, user_mass: float):
        """Keep the actuator load model and PyBullet ragdoll mass in sync."""
        self.user_mass = max(0.0, float(user_mass))
        # The two actuators share the user's weight evenly.
        self.load.set_mass(self.user_mass / 2.0)
        self._apply_user_mass_to_lay_figure(self.user_mass)

    # ── Safety ────────────────────────────────────────────────────────────

    def emergency_stop(self):
        """Halt both actuators and the controller."""
        self.ctrl.emergency_stop_triggered = True
        self.act_L.emergency_stop()
        self.act_R.emergency_stop()

    # ── Imported wheelchair / seated figure assets ───────────────────────

    def _load_imported_wheelchair(self) -> bool:
        """Load the Blender-derived wheelchair URDF if the asset is present."""
        if not os.path.exists(WHEELCHAIR_URDF):
            print(f"[WARN] Imported wheelchair URDF not found: {WHEELCHAIR_URDF}")
            return False

        self.wheelchair_model_id = p.loadURDF(
            WHEELCHAIR_URDF,
            WHEELCHAIR_MODEL_POS,
            WHEELCHAIR_MODEL_ORN,
            useFixedBase=True,
            flags=p.URDF_USE_MATERIAL_COLORS_FROM_MTL,
            globalScaling=WHEELCHAIR_MODEL_SCALE,
        )
        return True

    def _apply_lay_figure_texture(self):
        """Force PyBullet to use the packed base-color texture for the figure."""
        if getattr(self, "lay_figure_id", None) is None:
            return
        if not os.path.exists(LAY_FIGURE_BASE_COLOR):
            return

        tex_id = p.loadTexture(LAY_FIGURE_BASE_COLOR)
        for visual_shape in p.getVisualShapeData(self.lay_figure_id):
            link_index = visual_shape[1]
            p.changeVisualShape(
                self.lay_figure_id,
                link_index,
                rgbaColor=[1, 1, 1, 1],
                textureUniqueId=tex_id,
            )

    def _lay_figure_joint_indices(self):
        return {
            p.getJointInfo(self.lay_figure_id, i)[1].decode(): i
            for i in range(p.getNumJoints(self.lay_figure_id))
        }

    def _reset_lay_figure_pose(self):
        if getattr(self, "lay_figure_id", None) is None:
            return
        indices = self._lay_figure_joint_indices()
        for joint_name, target in LAY_FIGURE_SITTING_POSE.items():
            joint_index = indices.get(joint_name)
            if joint_index is not None:
                p.resetJointState(self.lay_figure_id, joint_index, target)

    def _remove_lay_figure_arm_constraints(self):
        for constraint_id in getattr(self, "_lay_figure_arm_constraint_ids", []):
            try:
                p.removeConstraint(constraint_id)
            except Exception:
                pass
        self._lay_figure_arm_constraint_ids = []

    def _set_lay_figure_arm_targets(self, targets, rebuild_constraints=True):
        """Update the non-ragdoll arm pose and optionally rebuild arm constraints."""
        if getattr(self, "lay_figure_id", None) is None:
            return

        indices = self._lay_figure_joint_indices()
        for joint_name, target in targets.items():
            if joint_name not in FIGURE_ARM_JOINT_TARGETS:
                continue
            FIGURE_ARM_JOINT_TARGETS[joint_name] = float(target)
            LAY_FIGURE_SITTING_POSE[joint_name] = float(target)
            joint_index = indices.get(joint_name)
            if joint_index is not None:
                p.resetJointState(self.lay_figure_id, joint_index, float(target))

        if rebuild_constraints:
            self._remove_lay_figure_arm_constraints()
            self._create_lay_figure_arm_constraints()

    def _disable_lay_figure_motors(self):
        if getattr(self, "lay_figure_id", None) is None:
            return
        indices = self._lay_figure_joint_indices()
        locked_arm_indices = {
            indices[name]
            for name in LAY_FIGURE_LOCKED_ARM_JOINTS
            if name in indices
        }
        for joint_index in range(p.getNumJoints(self.lay_figure_id)):
            if joint_index in locked_arm_indices:
                continue
            p.setJointMotorControl2(
                self.lay_figure_id,
                joint_index,
                p.VELOCITY_CONTROL,
                targetVelocity=0,
                force=0,
            )
        self._lock_lay_figure_arm_pose(indices)

    def _lock_lay_figure_arm_pose(self, indices=None):
        """Hold only arm joints; the rest of the figure stays as a ragdoll."""
        if getattr(self, "lay_figure_id", None) is None:
            return
        indices = indices or self._lay_figure_joint_indices()
        for joint_name in LAY_FIGURE_LOCKED_ARM_JOINTS:
            joint_index = indices.get(joint_name)
            target = LAY_FIGURE_SITTING_POSE.get(joint_name)
            if joint_index is None or target is None:
                continue
            p.setJointMotorControl2(
                self.lay_figure_id,
                joint_index,
                p.POSITION_CONTROL,
                targetPosition=target,
                force=LAY_FIGURE_ARM_HOLD_FORCE,
                positionGain=0.55,
                velocityGain=0.85,
                maxVelocity=3.0,
            )

    def _create_lay_figure_arm_constraints(self):
        """Make only the arms rigid relative to the torso for hook lifting."""
        if getattr(self, "lay_figure_id", None) is None:
            return
        indices = self._lay_figure_joint_indices()
        torso_link = indices.get("body_joint", 0)
        arm_links = [
            indices[name]
            for name in LAY_FIGURE_LOCKED_ARM_JOINTS
            if name in indices
        ]
        parent_pos, parent_orn = p.getLinkState(self.lay_figure_id, torso_link)[:2]
        inv_parent_pos, inv_parent_orn = p.invertTransform(parent_pos, parent_orn)

        self._lay_figure_arm_constraint_ids = []
        for arm_link in arm_links:
            child_pos, child_orn = p.getLinkState(self.lay_figure_id, arm_link)[:2]
            rel_pos, rel_orn = p.multiplyTransforms(
                inv_parent_pos,
                inv_parent_orn,
                child_pos,
                child_orn,
            )
            try:
                constraint_id = p.createConstraint(
                    self.lay_figure_id,
                    torso_link,
                    self.lay_figure_id,
                    arm_link,
                    p.JOINT_FIXED,
                    [0, 0, 0],
                    rel_pos,
                    [0, 0, 0],
                    rel_orn,
                    [0, 0, 0, 1],
                )
                p.changeConstraint(constraint_id, maxForce=3500.0)
                self._lay_figure_arm_constraint_ids.append(constraint_id)
            except Exception as exc:
                print(f"[WARN] Could not lock arm link {arm_link}: {exc}")

    def _set_lay_figure_dynamics(self):
        if getattr(self, "lay_figure_id", None) is None:
            return
        p.changeDynamics(self.lay_figure_id, -1, linearDamping=0.04, angularDamping=0.08)
        for link_index in range(p.getNumJoints(self.lay_figure_id)):
            p.changeDynamics(
                self.lay_figure_id,
                link_index,
                lateralFriction=0.8,
                spinningFriction=0.03,
                rollingFriction=0.02,
                linearDamping=0.04,
                angularDamping=0.08,
                jointDamping=0.03,
            )

    def _cache_lay_figure_mass_distribution(self):
        if getattr(self, "lay_figure_id", None) is None:
            return

        body_parts = [-1] + list(range(p.getNumJoints(self.lay_figure_id)))
        original_masses = [
            max(0.0, p.getDynamicsInfo(self.lay_figure_id, part_index)[0])
            for part_index in body_parts
        ]
        total_mass = sum(original_masses)
        if total_mass <= 0.0:
            equal_fraction = 1.0 / len(body_parts)
            self._lay_figure_mass_fractions = {
                part_index: equal_fraction for part_index in body_parts
            }
            return

        # Preserve the URDF's original mass ratio across the torso, arms, and legs.
        self._lay_figure_mass_fractions = {
            part_index: mass / total_mass
            for part_index, mass in zip(body_parts, original_masses)
        }

    def _apply_user_mass_to_lay_figure(self, user_mass: float):
        if getattr(self, "lay_figure_id", None) is None:
            return
        if not self._lay_figure_mass_fractions:
            self._cache_lay_figure_mass_distribution()
        if not self._lay_figure_mass_fractions:
            return

        for part_index, fraction in self._lay_figure_mass_fractions.items():
            p.changeDynamics(
                self.lay_figure_id,
                part_index,
                mass=max(0.001, user_mass * fraction),
            )

    def _build_lay_figure(self):
        """Drop the controllable lay figure onto the imported wheelchair."""
        self.lay_figure_id = None
        if not ENABLE_LAY_FIGURE_RAGDOLL:
            return
        if not os.path.exists(LAY_FIGURE_URDF):
            print(f"[WARN] Lay figure URDF not found: {LAY_FIGURE_URDF}")
            return

        flags = p.URDF_USE_MATERIAL_COLORS_FROM_MTL | p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT
        self.lay_figure_id = p.loadURDF(
            LAY_FIGURE_URDF,
            LAY_FIGURE_DROP_POS,
            LAY_FIGURE_ORN,
            useFixedBase=False,
            flags=flags,
            globalScaling=LAY_FIGURE_SCALE,
        )
        self._apply_lay_figure_texture()
        self._set_lay_figure_dynamics()
        self._cache_lay_figure_mass_distribution()
        self._apply_user_mass_to_lay_figure(self.user_mass)
        self._reset_lay_figure_pose()
        self._create_lay_figure_arm_constraints()
        self._disable_lay_figure_motors()

    # ── Floor ─────────────────────────────────────────────────────────────

    def _build_floor(self):
        # Near-black charcoal base — polished-showroom feel
        _box([4.5, 4.5, 0.012], [0, 0, -0.012], color=C_FLOOR_A)
        # Very fine grid at 50 cm intervals (barely visible — professional)
        for i in range(-8, 9):
            _box([4.5, 0.0010, 0.0002], [0, i * 0.50, 0.0005], color=C_FLOOR_B)
            _box([0.0010, 4.5, 0.0002], [i * 0.50, 0, 0.0005], color=C_FLOOR_B)
        # Brighter centre reference cross
        C_GRID_CTR = [0.26, 0.28, 0.33, 1.0]
        _box([4.5, 0.0022, 0.0003], [0, 0, 0.0006], color=C_GRID_CTR)
        _box([0.0022, 4.5, 0.0003], [0, 0, 0.0006], color=C_GRID_CTR)

    # ── Wheelchair frame ──────────────────────────────────────────────────

    def _build_wheelchair(self):
        if self._load_imported_wheelchair():
            return

        sz  = SEAT_H
        bx  = -SEAT_D / 2        # rear edge X
        fx  =  SEAT_D / 2        # front edge X

        # ── seat cushion ──────────────────────────────────────────────
        _box([SEAT_D/2 - 0.012, SEAT_W/2 - 0.012, CUSHION_T/2],
             [0, 0, sz - CUSHION_T/2], color=C_CUSHION)
        # side bolsters
        for y in (SEAT_W/2 - 0.022, -(SEAT_W/2 - 0.022)):
            _box([SEAT_D/2 - 0.015, 0.020, CUSHION_T/2 + 0.006],
                 [0, y, sz - CUSHION_T/2], color=C_CUSHION)

        # ── seat frame rails ──────────────────────────────────────────
        for y in (SEAT_W/2 - 0.022, -(SEAT_W/2 - 0.022)):
            _tube(bx + 0.02, y, sz - CUSHION_T - 0.008,
                   fx - 0.02, y, sz - CUSHION_T - 0.008, color=C_FRAME_MD)
        _tube(-0.06, -(SEAT_W/2-0.022), sz-CUSHION_T-0.008,
               -0.06,  (SEAT_W/2-0.022), sz-CUSHION_T-0.008, color=C_FRAME_MD)
        _tube( 0.06, -(SEAT_W/2-0.022), sz-CUSHION_T-0.008,
                0.06,  (SEAT_W/2-0.022), sz-CUSHION_T-0.008, color=C_FRAME_MD)

        # ── main side frames ──────────────────────────────────────────
        for y in (SEAT_W/2, -SEAT_W/2):
            # rear vertical post
            _tube(-0.10, y, 0.04, -0.10, y, sz - CUSHION_T - 0.015,
                  r=TUBE_R*1.2, color=C_FRAME_DK)
            # front vertical post
            _tube(fx - 0.03, y, 0.04, fx - 0.03, y, sz - CUSHION_T - 0.015,
                  r=TUBE_R*1.1, color=C_FRAME_DK)
            # bottom rail
            _tube(-0.10, y, 0.040, fx - 0.03, y, 0.040,
                  r=TUBE_R, color=C_FRAME_DK)
            # diagonal brace
            _tube(-0.10, y, 0.040, fx - 0.03, y, sz - CUSHION_T - 0.015,
                  r=TUBE_R*0.85, color=C_FRAME_MD)

        # cross braces under seat
        _tube(-0.10, -SEAT_W/2, 0.040, -0.10, SEAT_W/2, 0.040, color=C_FRAME_DK)
        _tube(fx-0.03, -SEAT_W/2, 0.040, fx-0.03, SEAT_W/2, 0.040, color=C_FRAME_DK)

        # ── backrest ──────────────────────────────────────────────────
        for y in (SEAT_W/2 - 0.030, -(SEAT_W/2 - 0.030)):
            _tube(bx+0.015, y, sz, bx+0.015, y, sz + BACK_H, color=C_FRAME_MD)
        _tube(bx+0.015, -(SEAT_W/2-0.030), sz + BACK_H,
               bx+0.015,  (SEAT_W/2-0.030), sz + BACK_H, color=C_FRAME_MD)
        _tube(bx+0.015, -(SEAT_W/2-0.030), sz + BACK_H*0.55,
               bx+0.015,  (SEAT_W/2-0.030), sz + BACK_H*0.55, color=C_FRAME_MD)
        # upholstery panel
        _box([0.018, SEAT_W/2 - 0.055, BACK_H/2 - 0.020],
             [bx + 0.022, 0, sz + BACK_H/2], color=C_CUSHION)

        # ── push handles ──────────────────────────────────────────────
        handle_z = sz + BACK_H + 0.11
        for y in (SEAT_W/2 - 0.025, -(SEAT_W/2 - 0.025)):
            _tube(bx+0.015, y, sz + BACK_H,
                   bx - 0.015, y, handle_z, r=TUBE_R, color=C_FRAME_MD)
        _tube(bx-0.015, -(SEAT_W/2-0.025), handle_z,
               bx-0.015,  (SEAT_W/2-0.025), handle_z,
               r=TUBE_R*1.1, color=C_FRAME_LT)

        # ── rear wheels ───────────────────────────────────────────────
        wx = -0.08
        for y in (SEAT_W/2 + 0.048, -(SEAT_W/2 + 0.048)):
            _cyl(WHEEL_R,          WHEEL_T,        [wx, y, WHEEL_R], WHEEL_ORN, C_TYRE)
            _cyl(WHEEL_R - 0.006,  WHEEL_T * 0.45, [wx, y, WHEEL_R], WHEEL_ORN, C_RIM)
            _cyl(WHEEL_R * 0.82,   WHEEL_T * 0.28, [wx, y, WHEEL_R], WHEEL_ORN, C_SPOKE)
            _cyl(0.028,            WHEEL_T * 0.80, [wx, y, WHEEL_R], WHEEL_ORN, C_HUB)
            # 6 spokes as full-diameter cylinders crossing the hub
            for angle_deg in range(0, 180, 30):
                th = math.radians(angle_deg)
                _cyl(0.004, WHEEL_R * 1.85,
                     [wx, y, WHEEL_R], _q(0.0, th, 0.0), C_SPOKE)
            # axle stub
            _cyl(0.018, 0.055, [wx, y, WHEEL_R], WHEEL_ORN, C_HUB)

        # ── front casters ─────────────────────────────────────────────
        cx = fx + 0.036
        for y in (SEAT_W/2 - 0.072, -(SEAT_W/2 - 0.072)):
            _cyl(CASTER_R,        CASTER_T,        [cx, y, CASTER_R], WHEEL_ORN, C_TYRE)
            _cyl(CASTER_R * 0.68, CASTER_T * 0.55, [cx, y, CASTER_R], WHEEL_ORN, C_RIM)
            _cyl(0.016,           CASTER_T * 0.75, [cx, y, CASTER_R], WHEEL_ORN, C_HUB)
            _tube(cx, y, CASTER_R * 2.0,
                   cx, y, CASTER_R * 2.0 + 0.072, r=TUBE_R*0.8, color=C_FRAME_MD)
            _tube(cx, y, CASTER_R*2.0 + 0.072,
                   fx - 0.03, y, sz - CUSHION_T - 0.018,
                   r=TUBE_R*0.8, color=C_FRAME_MD)

        # ── armrests ──────────────────────────────────────────────────
        arm_z = sz + 0.240
        for y in (SEAT_W/2 + 0.020, -(SEAT_W/2 + 0.020)):
            _box([0.185, 0.022, 0.014], [0.005, y, arm_z], color=C_FRAME_LT)
            _tube(0.005, y, sz - 0.010, 0.005, y, arm_z - 0.014,
                  r=TUBE_R*0.85, color=C_FRAME_MD)

        # ── footrests ─────────────────────────────────────────────────
        for y in (SEAT_W/2 - 0.095, -(SEAT_W/2 - 0.095)):
            _tube(fx - 0.025, y, sz - CUSHION_T - 0.018,
                   fx + 0.12, y, 0.15,
                   r=TUBE_R, color=C_FRAME_MD)
        _tube(fx + 0.12, -(SEAT_W/2-0.095), 0.150,
               fx + 0.12,  (SEAT_W/2-0.095), 0.150,
               r=TUBE_R, color=C_FRAME_MD)
        _box([0.070, SEAT_W/2 - 0.110, 0.008],
             [fx + 0.12, 0, 0.143], color=C_FRAME_LT)

    # ── Dual underarm harness lift mechanism ─────────────────────────────────

    def _build_actuators(self):
        """
        Harness-style lift mechanism matching the front/side sketch:
          two rectangular side actuator blocks beside the ribs
          open center so the seated figure fits inside the device
          hook pads under each armpit, plus rear/shoulder guide pieces
        """
        self._arm_ids = []
        self._act_rod_ids = []
        self._strap_ids = []
        self._moving_device_parts = []   # (body_id, x, y, z_offset, orn, color_mode)

        def add_moving_box(half, pos, color, z_offset, color_mode, orn=None):
            # All harness boxes move together in Z. Store only the fixed X/Y,
            # the Z offset from the moving pad center, and the display color mode.
            body_id = _vbox(half, pos, orn or _q(), color=color)
            self._moving_device_parts.append(
                (body_id, pos[0], pos[1], z_offset, orn or _q(), color_mode)
            )

        shape = DEVICE_SHAPE
        support_cz = shape["pad_z_rest"]
        shoulder_z = support_cz + shape["shoulder_z_offset"]

        # Build the left and right halves. "side" is +1 for one side of the body
        # and -1 for the other, so the same dimensions stay symmetric.
        for side in (1.0, -1.0):
            side_y = side * shape["side_y"]
            pad_y = side * shape["pad_y_offset"]
            # This bridge fills only the side gap between the outer module and
            # the armpit hook. It does not cross the torso center.
            reach_center_y = (side_y + pad_y) / 2.0
            reach_half_y = abs(side_y - pad_y) / 2.0

            # Rib-side rectangular lift block visible in both front and side views.
            # Tune DEVICE_SHAPE["side_y"] to move this block farther from the torso.
            add_moving_box(
                [0.036, 0.030, 0.150],
                [0.025, side_y, support_cz + 0.060],
                C_ACT_HOUSE,
                0.060,
                "carriage",
            )
            # Dark inner guide/rod drawn inside the rectangular block.
            add_moving_box(
                [0.010, 0.034, 0.126],
                [0.030, side_y, support_cz + 0.065],
                C_ACT_ROD,
                0.065,
                "active",
            )
            # Short horizontal underarm bridge from side block into the armpit pad.
            # This makes the device look connected while preserving the open middle.
            add_moving_box(
                [0.018, reach_half_y, 0.018],
                [shape["pad_x"], reach_center_y, support_cz + 0.020],
                C_FRAME_MD,
                0.020,
                "active",
            )
            # Small outer lip makes the underarm support read like a hanging hook.
            # It is visual-only; the actual collision hook is created in _build_sling.
            add_moving_box(
                [0.030, 0.012, 0.070],
                [shape["pad_x"] - 0.012,
                 side * (shape["pad_y_offset"] + shape["pad_half_y"] + 0.010),
                 support_cz + 0.058],
                C_FRAME_MD,
                0.058,
                "active",
            )
            # Back upright and shoulder cap approximate the curved shoulder strap.
            # These pieces sit behind/over the shoulders and help match the sketch.
            add_moving_box(
                [0.016, 0.020, 0.150],
                [shape["back_x"], side_y, support_cz + 0.130],
                C_FRAME_DK,
                0.130,
                "frame",
            )
            add_moving_box(
                [0.052, 0.020, 0.018],
                [(shape["back_x"] + shape["front_x"]) / 2.0, side_y, shoulder_z],
                C_FRAME_DK,
                shape["shoulder_z_offset"],
                "frame",
                _q(0.0, -0.45, 0.0),
            )

        # Rear cross strap sits behind the torso; the center/front stays open.
        # Remove or shorten this part if you want a completely separate left/right frame.
        add_moving_box(
            [0.016, shape["side_y"], 0.018],
            [shape["back_x"], 0.0, support_cz + 0.145],
            C_FRAME_DK,
            0.145,
            "frame",
        )

    # ── Underarm lift pads ────────────────────────────────────────────────

    def _build_sling(self):
        """
        Underarm lift assembly:
          • two hook-like pads sit under the user's left/right armpits
          • the middle is empty so the torso fits inside the device
          • no bottom sling panel or chest-crossing collision is used
        """
        shape = DEVICE_SHAPE
        z0 = shape["pad_z_rest"]
        self._sling_parts = []   # (body_id, x_offset, y_offset, z_offset)

        for sy in (shape["pad_y_offset"], -shape["pad_y_offset"]):
            # This is the physical support surface. If the figure misses the
            # hook, adjust DEVICE_SHAPE["pad_y_offset"] and ["pad_half_y"] first.
            side = 1.0 if sy > 0 else -1.0
            bid = _kbox(
                [shape["pad_half_x"], shape["pad_half_y"], shape["pad_half_z"]],
                [shape["pad_x"], sy, z0],
                color=C_SLING_DN,
            )
            p.changeDynamics(
                bid,
                -1,
                lateralFriction=1.5,
                spinningFriction=0.04,
                rollingFriction=0.02,
                restitution=0.0,
            )
            self._sling_parts.append((bid, shape["pad_x"], sy, 0.0))

            # Physical outer lip: this forms the "hook" wall that prevents the
            # upper arm from sliding away from the underarm pad during lifting.
            lip_y = sy + side * (shape["pad_half_y"] + 0.012)
            lip = _kbox(
                [shape["pad_half_x"] * 0.82, 0.012, 0.074],
                [shape["pad_x"] - 0.012, lip_y, z0 + 0.060],
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
            self._sling_parts.append((lip, shape["pad_x"] - 0.012, lip_y, 0.060))

        # Back-compat: sling_id points to the first moving support pad.
        self.sling_id     = self._sling_parts[0][0]
        self._sling_edges = []   # now handled via _sling_parts

    # ── GUI sliders ───────────────────────────────────────────────────────

    def _create_sliders(self):
        self.sl_weight = p.addUserDebugParameter(
            "User Weight (kg)  [40 – 220]", 40, 220, 80)
        self.sl_target = p.addUserDebugParameter(
            "Lift Target  ( 0 = down   →   1 = full 4-inch lift )", 0.0, 1.0, 0.0)
        self.sl_force  = p.addUserDebugParameter(
            "Actuator Rated Force per unit (N)", 200, 3000, 1000)
        self.sl_volts  = p.addUserDebugParameter(
            "Supply Voltage (V)  [12 or 24]", 12, 24, 12)
        self.sl_speed  = p.addUserDebugParameter(
            "Sim Speed  (0.25 = slow-mo   1 = real-time   4 = fast)", 0.25, 4.0, 1.0)

    # ── Static 3-D annotations ───────────────────────────────────────────

    def _build_labels(self):
        title_z = DEVICE_SHAPE["label_z"]
        p.addUserDebugText(
            "DUAL UNDERARM HARNESS LIFT SYSTEM",
            [0.0, 0.0, title_z],
            textColorRGB=[0.90, 0.92, 0.95], textSize=1.10, lifeTime=0)
        p.addUserDebugText(
            "PA-14 Class  |  4-inch Stroke  |  12 / 24 V DC  |  Paired Side Modules",
            [0.0, 0.0, title_z - 0.12],
            textColorRGB=C_LABEL_Y, textSize=0.75, lifeTime=0)

    def _build_lift_indicator(self):
        """Vertical ruler showing underarm support height range."""
        shape = DEVICE_SHAPE
        z_bot = shape["pad_z_rest"] + shape["pad_half_z"]
        z_top = z_bot + ACT_STROKE * shape["lift_gain"]
        rx    = RULER_X

        p.addUserDebugLine([rx, 0, z_bot], [rx, 0, z_top],
                           C_RULER, lineWidth=3, lifeTime=0)

        total_rise_in = (z_top - z_bot) * 39.37   # metres -> inches
        for frac, label in ((0.0,  "0%  (seated)"),
                             (0.25, "25%"),
                             (0.50, "50%"),
                             (0.75, "75%"),
                             (1.0,  f"100% ({total_rise_in:.1f} in)")):
            zt = z_bot + frac * (z_top - z_bot)
            p.addUserDebugLine([rx - 0.016, 0, zt], [rx + 0.016, 0, zt],
                               C_RULER, lineWidth=2, lifeTime=0)
            p.addUserDebugText(label, [rx + 0.022, 0, zt],
                               textColorRGB=C_RULER, textSize=0.62, lifeTime=0)

    # ── Per-frame visual updates ──────────────────────────────────────────

    def _update_visuals(self, ext: float, stalled: bool,
                        overloaded: bool, at_target: bool,
                        force_per_act: float):
        # ── Colour state ───────────────────────────────────────────────
        if stalled or overloaded:
            arm_c, sling_c, arr_c = C_ACT_DEAD, C_SLING_DN, [0.95, 0.15, 0.10]
        elif at_target and ext > 0.002:
            arm_c, sling_c, arr_c = C_ACT_OK,   C_SLING_UP, [0.18, 0.90, 0.30]
        elif ext > ACT_STROKE * 0.72:
            arm_c, sling_c, arr_c = C_ACT_WARN,  C_SLING_DN, [0.95, 0.70, 0.10]
        elif ext > 0.002:
            arm_c, sling_c, arr_c = C_ACT_OK,   C_SLING_DN, [0.18, 0.90, 0.30]
        else:
            arm_c, sling_c, arr_c = C_ACT_ROD,  C_SLING_DN, [0.55, 0.55, 0.60]

        t = ext / max(ACT_STROKE, 1e-9)
        # The harness yoke uses a linkage gain so the supports can start low,
        # then rise into the armpit area as the actuator extends.
        shape = DEVICE_SHAPE
        support_top_z = shape["pad_z_rest"] + shape["pad_half_z"] + ext * shape["lift_gain"]
        support_cz    = support_top_z - shape["pad_half_z"]

        # Move every visual harness part to the same lift height as the pads.
        # X/Y stay fixed so the device keeps its open frame shape while rising.
        for body_id, x, y, z_offset, orn, color_mode in self._moving_device_parts:
            _move(body_id, [x, y, support_cz + z_offset], orn)
            if color_mode == "active":
                _recolor(body_id, arm_c)
            elif color_mode == "strap":
                _recolor(body_id, C_TIE)
            elif color_mode == "frame":
                _recolor(body_id, C_FRAME_DK)
            else:
                _recolor(body_id, C_ACT_HOUSE)

        # Debug arrows show the lift force direction from each armpit hook.
        for i, sy in enumerate((shape["pad_y_offset"], -shape["pad_y_offset"])):
            base = [shape["pad_x"], sy, support_top_z + 0.010]
            tip  = [shape["pad_x"], sy, support_top_z + min(0.30, force_per_act / 100.0 * 0.05)]
            key  = f'arr{i}'
            if key in self._arrows:
                p.addUserDebugLine(base, tip, arr_c, lineWidth=4,
                                   replaceItemUniqueId=self._arrows[key])
            else:
                self._arrows[key] = p.addUserDebugLine(
                    base, tip, arr_c, lineWidth=4, lifeTime=0)

        # ── Underarm support pads ──────────────────────────────────────
        # These are the two collision bodies that actually contact and lift the figure.
        for bid, ox, oy, oz in self._sling_parts:
            _move(bid, [ox, oy, support_cz + oz])
            _recolor(bid, sling_c)

        # ── Lift indicator (ruler pointer) ─────────────────────────────
        # Map support rise to ruler: support_top_z at rest -> z_bot, at full -> z_top
        z_bot = shape["pad_z_rest"] + shape["pad_half_z"]
        z_top = z_bot + ACT_STROKE * shape["lift_gain"]
        z_ind = z_bot + t * (z_top - z_bot)
        rx = RULER_X
        ind_from = [rx, 0, z_ind]
        ind_to   = [rx - 0.032, 0, z_ind]
        if self._ind_line_id >= 0:
            self._ind_line_id = p.addUserDebugLine(
                ind_from, ind_to, [1.0, 0.85, 0.0], lineWidth=5,
                replaceItemUniqueId=self._ind_line_id, lifeTime=0)
        else:
            self._ind_line_id = p.addUserDebugLine(
                ind_from, ind_to, [1.0, 0.85, 0.0], lineWidth=5, lifeTime=0)

    # ── Main run loop ─────────────────────────────────────────────────────

    def run(self, external_control: bool = False):
        BASE_DT = 0.01
        RENDER_EVERY = 2
        CTRL_MIN_WEIGHT = 40.0
        CTRL_MAX_WEIGHT = 220.0
        CTRL_MIN_TARGET = 0.0
        CTRL_MAX_TARGET = 1.0
        CTRL_MIN_FORCE = 200.0
        CTRL_MAX_FORCE = 3000.0
        CTRL_MIN_VOLTS = 12.0
        CTRL_MAX_VOLTS = 24.0
        CTRL_MIN_SPEED = 0.25
        CTRL_MAX_SPEED = 4.0

        prev_weight = -1.0
        prev_force = -1.0
        step = 0
        duty_window = 200
        ctrl_state = {
            "weight": 80.0,
            "target": 0.0,
            "force": 1000.0,
            "volts": 12.0,
            "speed": 1.0,
        }

        initial_ext = (self.act_L.position + self.act_R.position) / 2.0
        initial_state = {
            "user_mass": self.load.mass * 2.0,
            "ext_m": initial_ext,
            "pwm": self.act_L.pwm_input,
            "cap_force_each": self.act_L.max_force,
            "stalled": self.act_L.stalled or self.act_R.stalled,
            "overloaded": self.ctrl.overload_detected,
            "at_target": self.ctrl.at_target(),
            "supply_v": 12.0,
            "duty_pct": 0.0,
            "target_position": self.ctrl.target_position,
            "target": self.ctrl.target_position / max(ACT_STROKE, 1e-9),
            "speed": 1.0,
        }

        try:
            show_interactive_live_screen()
            with create_interactive_live(initial_state, refresh_per_second=10) as live:
                with KeyReader() as key_reader:
                    while p.isConnected():
                        t0 = time.perf_counter()
                        should_quit = False

                        # drain API commands when hub is driving the sim
                        if external_control:
                            while True:
                                try:
                                    cmd = _cmd_queue.get_nowait()
                                except _queue.Empty:
                                    break
                                current_pos = (self.act_L.position + self.act_R.position) / 2.0
                                if cmd == 'UP':
                                    new_target = min(self.ctrl.target_position + STEP_SIZE, ACT_STROKE)
                                    self.ctrl.set_target_position(new_target)
                                    ctrl_state["target"] = new_target / ACT_STROKE
                                    print(f"[serve] UP  → target {new_target*1000:.1f} mm")
                                elif cmd == 'DOWN':
                                    new_target = max(self.ctrl.target_position - STEP_SIZE, 0.0)
                                    self.ctrl.set_target_position(new_target)
                                    ctrl_state["target"] = new_target / ACT_STROKE
                                    print(f"[serve] DOWN → target {new_target*1000:.1f} mm")
                                elif cmd == 'START':
                                    self.ctrl.reset_emergency_stop()
                                    self.act_L.reset_emergency_stop()
                                    self.act_R.reset_emergency_stop()
                                    print("[serve] START — e-stop cleared")
                                elif cmd == 'STOP':
                                    self.ctrl.set_target_position(current_pos)
                                    ctrl_state["target"] = current_pos / max(ACT_STROKE, 1e-9)
                                    print(f"[serve] STOP — frozen at {current_pos*1000:.1f} mm")
                                _cmd_queue.task_done()

                        for ch in key_reader.poll():
                            c = ch.lower()
                            if c == "q":
                                should_quit = True
                                break
                            elif c == "j":
                                ctrl_state["weight"] -= 1.0
                            elif c == "k":
                                ctrl_state["weight"] += 1.0
                            elif not external_control and c == "n":
                                ctrl_state["target"] -= 0.02
                            elif not external_control and c == "m":
                                ctrl_state["target"] += 0.02
                            elif c == "u":
                                ctrl_state["force"] -= 50.0
                            elif c == "i":
                                ctrl_state["force"] += 50.0
                            elif c == "v":
                                ctrl_state["volts"] -= 1.0
                            elif c == "b":
                                ctrl_state["volts"] += 1.0
                            elif c == "1":
                                ctrl_state["speed"] -= 0.05
                            elif c == "2":
                                ctrl_state["speed"] += 0.05

                        if should_quit:
                            break

                        ctrl_state["weight"] = max(CTRL_MIN_WEIGHT, min(CTRL_MAX_WEIGHT, ctrl_state["weight"]))
                        ctrl_state["target"] = max(CTRL_MIN_TARGET, min(CTRL_MAX_TARGET, ctrl_state["target"]))
                        ctrl_state["force"] = max(CTRL_MIN_FORCE, min(CTRL_MAX_FORCE, ctrl_state["force"]))
                        ctrl_state["volts"] = max(CTRL_MIN_VOLTS, min(CTRL_MAX_VOLTS, ctrl_state["volts"]))
                        ctrl_state["speed"] = max(CTRL_MIN_SPEED, min(CTRL_MAX_SPEED, ctrl_state["speed"]))

                        user_mass = ctrl_state["weight"]
                        target_in = ctrl_state["target"]
                        max_force = ctrl_state["force"]
                        supply_v = ctrl_state["volts"]
                        speed_mult = ctrl_state["speed"]
                        target_m = target_in * ACT_STROKE

                        if abs(user_mass - prev_weight) > 0.05:
                            self._set_user_mass(user_mass)
                            prev_weight = user_mass

                        if abs(max_force - prev_force) > 0.5:
                            self.act_L.max_force = max_force
                            self.act_R.max_force = max_force
                            prev_force = max_force

                        # keyboard controls target only when hub is NOT driving it
                        if not external_control and abs(target_m - self.ctrl.target_position) > 0.0001:
                            clamped = max(0.0, min(target_m, ACT_STROKE))
                            if not self.ctrl.set_target_position(clamped):
                                self.ctrl.set_target_position(0.0)

                        n_steps = max(1, round(speed_mult))
                        pwm = 0.0
                        req_force = 0.0

                        for _ in range(n_steps):
                            req_force = self.load.get_required_force(self.act_L.acceleration)
                            pwm = self.ctrl.update(BASE_DT)
                            self.act_L.set_pwm(pwm)
                            self.act_R.set_pwm(pwm)
                            self.act_L.step(BASE_DT, req_force)
                            self.act_R.step(BASE_DT, req_force)
                            self._disable_lay_figure_motors()
                            p.stepSimulation()

                            self._duty_total += 1
                            if abs(pwm) > 0.02 and not self.act_L.stalled:
                                self._duty_on += 1

                        ext = (self.act_L.position + self.act_R.position) / 2.0
                        stalled = self.act_L.stalled or self.act_R.stalled
                        overloaded = self.ctrl.overload_detected
                        at_target = self.ctrl.at_target()

                        window = max(1, duty_window)
                        if self._duty_total > window * 3:
                            self._duty_on = int(self._duty_on * window / self._duty_total)
                            self._duty_total = window
                        duty_pct = self._duty_on / max(1, self._duty_total) * 100.0

                        step += 1
                        if step % RENDER_EVERY == 0:
                            self._update_visuals(ext, stalled, overloaded, at_target, force_per_act=req_force)

                        update_interactive_live(
                            live,
                            {
                                "user_mass": user_mass,
                                "ext_m": ext,
                                "pwm": pwm,
                                "cap_force_each": max_force,
                                "stalled": stalled,
                                "overloaded": overloaded,
                                "at_target": at_target,
                                "supply_v": supply_v,
                                "duty_pct": duty_pct,
                                "target_position": self.ctrl.target_position,
                                "target": ctrl_state["target"],
                                "speed": ctrl_state["speed"],
                            },
                        )

                        frame_real_dt = BASE_DT * n_steps / speed_mult
                        elapsed = time.perf_counter() - t0
                        slack = frame_real_dt - elapsed
                        if slack > 0:
                            time.sleep(slack)

        except KeyboardInterrupt:
            print("\nSimulation stopped.")
        finally:
            if p.isConnected():
                p.disconnect()


    # ── Automated demo ────────────────────────────────────────────────────

    def run_demo(self):
        """
        Self-running demo — no slider interaction needed.

        Cycles through 4 weight scenarios automatically at 5× real-time
        so every lift, hold, and lower is clearly visible.

        Scenarios:
          1. Light user   (60 kg / 132 lb)  — lifts easily
          2. Standard user (80 kg / 176 lb) — nominal design case
          3. Heavy user  (130 kg / 286 lb)  — high-force actuator needed
          4. Overload test (180 kg, low rated force) — intentional stall
        """
        SCENARIOS = [
            {"label": "LIGHT USER",      "mass_kg":  60, "force": 1000},
            {"label": "STANDARD USER",   "mass_kg":  80, "force": 1000},
            {"label": "HEAVY USER",      "mass_kg": 130, "force": 1500},
            {"label": "OVERLOAD TEST",   "mass_kg": 180, "force":  500},
        ]
        DEMO_STEPS  = 5       # physics steps per wall-clock frame → 5× real-time
        BASE_DT     = 0.01
        HOLD_TOP_S  = 3.0     # wall-clock seconds to hold at full extension
        HOLD_BTM_S  = 1.8     # wall-clock seconds between scenarios

        si          = 0
        phase       = "LIFTING"
        hold_start  = None
        demo_txt    = {}
        step        = 0

        z_top = SEAT_H + ACT_BODY_H + ACT_SHAFT_H + ACT_STROKE + 0.55
        tx    = 0.55

        # z positions for each HUD row (top → bottom)
        Zs = [z_top + 0.17,   # 0  top border
              z_top + 0.07,   # 1  title
              z_top - 0.04,   # 2  scenario counter
              z_top - 0.14,   # 3  mid border
              z_top - 0.28,   # 4  scenario label  (big)
              z_top - 0.40,   # 5  mass
              z_top - 0.52,   # 6  divider
              z_top - 0.61,   # 7  phase indicator (big)
              z_top - 0.73,   # 8  progress bar
              z_top - 0.83,   # 9  lift measurement
              z_top - 0.95,   # 10 divider
              z_top - 1.04,   # 11 FORCE CALC header
              z_top - 1.14,   # 12 total gravity
              z_top - 1.24,   # 13 per actuator
              z_top - 1.34,   # 14 rated / OK
              z_top - 1.45,   # 15 bottom border
              ]

        def _reset_and_load(idx):
            s = SCENARIOS[idx % len(SCENARIOS)]
            self._set_user_mass(s["mass_kg"])
            self.act_L.max_force = s["force"]
            self.act_R.max_force = s["force"]
            for act in (self.act_L, self.act_R):
                act.stalled            = False
                act.velocity           = 0.0
                act.acceleration       = 0.0
                act.pwm_input          = 0.0
                act.emergency_stopped  = False
            self.ctrl.overload_detected        = False
            self.ctrl.emergency_stop_triggered = False
            self.ctrl.integral_error           = 0.0
            self.ctrl.last_error               = 0.0
            self.ctrl.set_target_position(ACT_STROKE)
            return s           

        # ── bootstrap ──────────────────────────────────────────────────
        cur = _reset_and_load(si)

        last_event_text = ""

        initial_state = {
            "scenario": cur,
            "phase": phase,
            "ext_m": 0.0,
            "stalled": False,
            "overload": False,
            "idx": si,
            "total": len(SCENARIOS),
            "stroke_m": ACT_STROKE,
            "event_text": last_event_text,
        }

        try:
            with create_demo_live(initial_state, refresh_per_second=10) as live:
                with KeyReader() as key_reader:
                    build_demo_scenarios_panel(live, SCENARIOS)
                    while p.isConnected():
                        t0 = time.perf_counter()

                        should_quit = False

                        for ch in key_reader.poll():
                            c = ch.lower()
                            if c == "q":
                                should_quit = True
                                break

                        if should_quit:
                            break
                        
                        # ── 5 physics steps per frame ──────────────────────────
                        for _ in range(DEMO_STEPS):
                            req = self.load.get_required_force(self.act_L.acceleration)
                            pwm = self.ctrl.update(BASE_DT)
                            self.act_L.set_pwm(pwm)
                            self.act_R.set_pwm(pwm)
                            self.act_L.step(BASE_DT, req)
                            self.act_R.step(BASE_DT, req)
                            self._disable_lay_figure_motors()
                            p.stepSimulation()

                        ext      = (self.act_L.position + self.act_R.position) / 2.0
                        stalled  = self.act_L.stalled or self.act_R.stalled
                        overload = self.ctrl.overload_detected
                        at_tgt   = self.ctrl.at_target()

                        # ── state machine ──────────────────────────────────────
                        if phase == "LIFTING":
                            if at_tgt or stalled or overload:
                                phase      = "HOLD_UP"
                                hold_start = time.perf_counter()
                                tag = "STALLED" if (stalled or overload) else "AT TOP"
                                last_event_text += f"  [{cur['label']:16s}]  {cur['mass_kg']:3} kg  " \
                                                f"{tag:8}  {ext*1000:.1f} mm  " \
                                                f"need {cur['mass_kg']*9.81/2:.0f} N  " \
                                                f"rated {cur['force']} N"
                        elif phase == "HOLD_UP":
                            if time.perf_counter() - hold_start >= HOLD_TOP_S:
                                phase = "LOWERING"
                                # Clear stall so actuator can retract
                                for act in (self.act_L, self.act_R):
                                    act.stalled = False
                                self.ctrl.overload_detected = False
                                self.ctrl.set_target_position(0.0)

                        elif phase == "LOWERING":
                            if ext < 0.003 and at_tgt:
                                phase      = "HOLD_DOWN"
                                hold_start = time.perf_counter()

                        elif phase == "HOLD_DOWN":
                            if time.perf_counter() - hold_start >= HOLD_BTM_S:
                                si  = (si + 1) % len(SCENARIOS)
                                cur = _reset_and_load(si)
                                phase = "LIFTING"
                                last_event_text = f"Scenario {si+1}: {cur['label']} ({cur['mass_kg']} kg)\n"

                        # ── render ─────────────────────────────────────────────
                        if not p.isConnected():
                            break
                        step += 1
                        if step % 2 == 0:
                            try:
                                self._update_visuals(ext, stalled, overload, at_tgt,
                                                    force_per_act=req)
                            except Exception:
                                break   # window closed mid-frame
                        update_demo_live(
                            live,
                            {
                                "scenario": cur,
                                "phase": phase,
                                "ext_m": ext,
                                "stalled": stalled,
                                "overload": overload,
                                "idx": si,
                                "total": len(SCENARIOS),
                                "stroke_m": ACT_STROKE,
                                "event_text": last_event_text,
                            },
                        )

                        # ── real-time pacing (target 100 Hz wall clock) ────────
                        elapsed = time.perf_counter() - t0
                        slack   = BASE_DT - elapsed
                        if slack > 0:
                            time.sleep(slack)

        except KeyboardInterrupt:
            print("\n  Demo stopped.")
        finally:
            try:
                if p.isConnected():
                    p.disconnect()
            except Exception:
                pass


# ── Entry point ───────────────────────────────────────────────────────────

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Wheelchair Sling Lift 3D Simulation")
    parser.add_argument("--serve", action="store_true",
                        help="Expose HTTP API on :8001 so the Ableware hub can drive the sim")
    parser.add_argument("--demo", action="store_true",
                        help="Run the automated demo cycle instead of interactive mode")
    parser.add_argument("--port", type=int, default=8001,
                        help="Port for the HTTP API (default: 8001, only used with --serve)")
    args = parser.parse_args()

    sim = WheelchairLiftSim3D()

    if args.serve:
        _sim_ref = sim

        server = HTTPServer(("", args.port), _SimAPIHandler)
        t = threading.Thread(target=server.serve_forever, daemon=True)
        t.start()
        print(f"[serve] HTTP API listening on :{args.port}  (POST /command  GET /state)")

        try:
            if args.demo:
                sim.run_demo()
            else:
                sim.run(external_control=True)
        finally:
            server.shutdown()
    elif args.demo:
        sim.run_demo()
    else:
        sim.run()
