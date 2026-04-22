#!/usr/bin/env python3
"""
3D Wheelchair Sling Lift Simulation — PyBullet (Demo Edition)

Simulates the assistive sling-lift device:
  - Manual wheelchair with tubular steel frame and spoked wheels
  - Rigid sling board sitting on the seat cushion
  - Two linear actuators under the sling that push it up ~1 inch
  - 1-inch lift creates clearance to pull pants through the gap

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
    Interactive 3D wheelchair sling-lift simulation.

    LinearActuator + LiftController + UserLoad handle all physics.
    PyBullet provides rendering, real-time GUI, and visual feedback.
    """

    def __init__(self, headless: bool = False):
        self._headless = headless
        self._connect()

        p.setGravity(0, 0, 0)   # physics modules own gravity
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

    # ── Safety ────────────────────────────────────────────────────────────

    def emergency_stop(self):
        """Halt both actuators and the controller."""
        self.ctrl.emergency_stop_triggered = True
        self.act_L.emergency_stop()
        self.act_R.emergency_stop()

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

    # ── Pivot arm mechanism ───────────────────────────────────────────────────

    def _build_actuators(self):
        """
        Pivot-arm lift mechanism — one assembly per side.

        Each side:
          pivot bracket  — bolted to the rear upper frame post (static)
          rigid arm      — square-section aluminium bar, rotates around pivot
          linear actuator — diagonal PA-14 unit: motor body (static) + rod (animated)
          sling straps   — two nylon webbing drops from arm tip to sling corners
        """
        self._arm_ids     = []   # [L, R] rotating arm bodies
        self._act_rod_ids = []   # [L, R] animated actuator rod bodies
        self._strap_ids   = []   # 4 straps: L-front, L-rear, R-front, R-rear

        # Pre-compute initial actuator geometry (same for both sides)
        θ0   = ARM_ANGLE_DOWN
        Mx0  = PIVOT_X + ARM_ACT_D * math.cos(θ0)
        Mz0  = PIVOT_Z + ARM_ACT_D * math.sin(θ0)
        dx0  = Mx0 - ACT_BASE_X
        dz0  = Mz0 - ACT_BASE_Z
        L0   = math.sqrt(dx0 * dx0 + dz0 * dz0)
        ux0, uz0 = dx0 / L0, dz0 / L0
        act0_orn = _q(0, math.atan2(ux0, uz0), 0)   # initial actuator orientation

        # Strap length: arm tip Z at rest minus sling-top Z
        tip_z0          = PIVOT_Z + ARM_LEN * math.sin(θ0)
        self._strap_len = tip_z0 - (SEAT_H + SLING_T)

        for sign, y_piv in ((+1, PIVOT_Y_OFF), (-1, -PIVOT_Y_OFF)):

            # ── Pivot bracket (bolted to rear vertical post) ───────────
            _box([PVTBKT_W / 2, 0.016, PVTBKT_H / 2],
                 [PIVOT_X, y_piv, PIVOT_Z], color=C_FRAME_DK)
            # Gusset plate (triangular stiffener, approximated as a box)
            _box([0.022, 0.012, 0.036],
                 [PIVOT_X - 0.020, y_piv, PIVOT_Z - 0.022], color=C_FRAME_DK)
            # Pivot pin (through-bolt, runs in Y)
            _cyl(0.010, 0.060,
                 [PIVOT_X, y_piv, PIVOT_Z], _q(math.pi / 2, 0, 0), C_FRAME_LT)
            # Hex nut at pin end
            _cyl(0.014, 0.012,
                 [PIVOT_X, y_piv + sign * 0.038, PIVOT_Z],
                 _q(math.pi / 2, 0, 0), C_CAP)

            # ── Rigid arm (visual box, animated each frame) ────────────
            arm_cx = PIVOT_X + ARM_LEN / 2 * math.cos(θ0)
            arm_cz = PIVOT_Z + ARM_LEN / 2 * math.sin(θ0)
            arm_orn = _q(0, -θ0, 0)   # pitch around Y so X-axis points along arm

            aid = _vbox([ARM_LEN / 2, ARM_W / 2, ARM_W / 2],
                        [arm_cx, y_piv, arm_cz], arm_orn, C_FRAME_MD)
            self._arm_ids.append(aid)

            # Reinforcing strip along arm top face (chrome highlight)
            strip_id = _vbox([ARM_LEN / 2, ARM_W / 2 * 0.55, 0.004],
                             [arm_cx, y_piv, arm_cz], arm_orn, C_FRAME_LT)
            self._arm_ids.append(strip_id)   # paired: updated together with main arm

            # Sling attachment eye at arm tip (ball-joint sphere)
            tip_x0 = PIVOT_X + ARM_LEN * math.cos(θ0)
            _vsph(ARM_W * 1.5, [tip_x0, y_piv, tip_z0], C_FRAME_LT)

            # ── Actuator foot (clevis mount bolted to lower frame) ──────
            _box([0.024, 0.018, 0.030],
                 [ACT_BASE_X, y_piv, ACT_BASE_Z + 0.015], color=C_CAP)
            _cyl(0.006, 0.040,
                 [ACT_BASE_X, y_piv, ACT_BASE_Z + 0.028],
                 _q(math.pi / 2, 0, 0), C_FRAME_LT)
            # Foot mounting plate
            _box([0.036, 0.026, 0.008],
                 [ACT_BASE_X, y_piv, ACT_BASE_Z], color=C_FRAME_DK)

            # ── Motor / gearbox body (static, lower 38% of actuator) ───
            motor_len = L0 * 0.38
            mc_x = ACT_BASE_X + ux0 * motor_len / 2
            mc_z = ACT_BASE_Z + uz0 * motor_len / 2
            _cyl(ACT_NEW_MOTOR_R, motor_len,
                 [mc_x, y_piv, mc_z], act0_orn, C_ACT_MOTOR)
            # Power cable stub exiting motor body
            cable_side = sign * ACT_NEW_MOTOR_R * 1.1
            _cyl(0.007, 0.045,
                 [mc_x - 0.018, y_piv + cable_side, mc_z - 0.012],
                 _q(0, math.pi * 0.30, 0), C_CABLE)

            # Flange ring between motor and barrel
            fl_x = ACT_BASE_X + ux0 * motor_len
            fl_z = ACT_BASE_Z + uz0 * motor_len
            _cyl(ACT_NEW_MOTOR_R * 1.28, 0.010,
                 [fl_x, y_piv, fl_z], act0_orn, C_CAP)

            # ── Actuator barrel (static outer sleeve, upper 38%) ────────
            barrel_len = L0 * 0.38
            bc_x = fl_x + ux0 * barrel_len / 2
            bc_z = fl_z + uz0 * barrel_len / 2
            _cyl(ACT_NEW_R, barrel_len,
                 [bc_x, y_piv, bc_z], act0_orn, C_ACT_HOUSE)
            # End-cap at barrel tip
            cap_x = fl_x + ux0 * barrel_len
            cap_z = fl_z + uz0 * barrel_len
            _cyl(ACT_NEW_R * 1.18, 0.010,
                 [cap_x, y_piv, cap_z], act0_orn, C_CAP)

            # ── Animated inner rod (extends from barrel exit to arm) ────
            rod_base_x = fl_x + ux0 * barrel_len * 0.10   # starts just inside barrel tip
            rod_base_z = fl_z + uz0 * barrel_len * 0.10
            rod_len = L0 * 0.62   # slightly longer than barrel for overlap
            rod_cx = rod_base_x + ux0 * rod_len / 2
            rod_cz = rod_base_z + uz0 * rod_len / 2
            rid = _vcyl(ACT_NEW_ROD_R, rod_len,
                        [rod_cx, y_piv, rod_cz], act0_orn, C_ACT_ROD)
            self._act_rod_ids.append(rid)

            # Clevis / rod-end at actuator tip (animated with rod)
            self._act_rod_ids.append(
                _vbox([0.018, 0.012, 0.014],
                      [Mx0, y_piv, Mz0], color=C_CAP)
            )

            # ── Suspension straps (front + rear, animated) ─────────────
            sling_top_z = SEAT_H + SLING_T
            for tx in (TIE_X_F, TIE_X_R):
                sdx = tx - tip_x0
                sdz = sling_top_z - tip_z0
                sL  = math.sqrt(sdx * sdx + sdz * sdz)
                sorn = _q(0, math.atan2(sdx / sL, sdz / sL), 0) if sL > 1e-6 else _q()
                mid_x = (tip_x0 + tx) / 2
                mid_z = (tip_z0 + sling_top_z) / 2
                t = _vbox([TIE_W / 2, SLING_STRAP_W / 2, sL / 2],
                          [mid_x, y_piv, mid_z], sorn, C_TIE)
                self._strap_ids.append(t)

        # ── Cross-brace connecting both pivot brackets ─────────────────
        _tube(PIVOT_X, -PIVOT_Y_OFF + PVTBKT_W, PIVOT_Z,
              PIVOT_X,  PIVOT_Y_OFF - PVTBKT_W, PIVOT_Z,
              r=TUBE_R * 1.6, color=C_FRAME_DK)

    # ── Sling ─────────────────────────────────────────────────────────────

    def _build_sling(self):
        """
        Sling assembly — matches a real lift sling:
          • 3 transverse nylon straps (front, mid, rear) spanning seat width
          • 2 longitudinal side rails connecting strap ends
          • All parts animate upward together
        """
        z0 = SEAT_H + SLING_T / 2
        self._sling_parts = []   # (body_id, x_offset, y_offset)

        # ── 3 transverse nylon straps ──────────────────────────────────
        for sx in (TIE_X_F, 0.0, TIE_X_R):
            bid = _vbox([SLING_STRAP_W / 2, SLING_W / 2, SLING_T / 2 + 0.005],
                        [sx, 0, z0], color=C_SLING_DN)
            self._sling_parts.append((bid, sx, 0.0))

        # ── 2 longitudinal side rails (span front-strap to rear-strap) ─
        rail_hw = (TIE_X_F - TIE_X_R) / 2.0   # half-length of rail
        for sy in (SLING_W / 2 - SLING_RAIL_W / 2,
                   -(SLING_W / 2 - SLING_RAIL_W / 2)):
            bid = _vbox([rail_hw, SLING_RAIL_W / 2, SLING_T / 2 + 0.003],
                        [0.0, sy, z0], color=C_SLING_EDG)
            self._sling_parts.append((bid, 0.0, sy))

        # back-compat: sling_id points to first strap
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
        title_z = PIVOT_Z + ARM_LEN + 0.32
        p.addUserDebugText(
            "PIVOT-ARM HIP LIFT SYSTEM",
            [0.0, 0.0, title_z],
            textColorRGB=[0.90, 0.92, 0.95], textSize=1.10, lifeTime=0)
        p.addUserDebugText(
            "PA-14 Class  |  4-inch Stroke  |  12 / 24 V DC  |  Pivot Arm Drive",
            [0.0, 0.0, title_z - 0.12],
            textColorRGB=C_LABEL_Y, textSize=0.75, lifeTime=0)

    def _build_lift_indicator(self):
        """Vertical ruler showing sling height range (rest → full lift)."""
        tip_z_rest = PIVOT_Z + ARM_LEN * math.sin(ARM_ANGLE_DOWN)
        tip_z_full = PIVOT_Z + ARM_LEN * math.sin(ARM_ANGLE_UP)
        # Ruler shows SLING height (tip minus constant strap length)
        # Approximate strap_len here since the object isn't built yet
        approx_strap = tip_z_rest - (SEAT_H + SLING_T)
        z_bot = tip_z_rest - approx_strap
        z_top = tip_z_full - approx_strap
        rx    = RULER_X

        p.addUserDebugLine([rx, 0, z_bot], [rx, 0, z_top],
                           C_RULER, lineWidth=3, lifeTime=0)

        total_rise_in = (z_top - z_bot) * 39.37   # metres → inches
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

        # ── Arm angle from actuator extension (linear interpolation) ───
        t = ext / max(ACT_STROKE, 1e-9)
        θ = ARM_ANGLE_DOWN + t * (ARM_ANGLE_UP - ARM_ANGLE_DOWN)
        arm_orn = _q(0, -θ, 0)   # pitch around Y rotates X-axis to angle θ

        # Geometry for THIS frame
        tip_x  = PIVOT_X + ARM_LEN     * math.cos(θ)
        tip_z  = PIVOT_Z + ARM_LEN     * math.sin(θ)
        act_x  = PIVOT_X + ARM_ACT_D   * math.cos(θ)   # actuator arm-attachment
        act_z  = PIVOT_Z + ARM_ACT_D   * math.sin(θ)

        # Actuator rod direction (from foot to arm-attachment)
        rdx = act_x - ACT_BASE_X
        rdz = act_z - ACT_BASE_Z
        rL  = math.sqrt(rdx * rdx + rdz * rdz)
        rux, ruz = rdx / rL, rdz / rL
        rod_orn = _q(0, math.atan2(rux, ruz), 0)

        # Precompute rest-length to find rod base position (fixed housing exit)
        θ0 = ARM_ANGLE_DOWN
        dx0 = PIVOT_X + ARM_ACT_D * math.cos(θ0) - ACT_BASE_X
        dz0 = PIVOT_Z + ARM_ACT_D * math.sin(θ0) - ACT_BASE_Z
        L0  = math.sqrt(dx0 * dx0 + dz0 * dz0)
        ux0 = dx0 / L0;  uz0 = dz0 / L0
        rod_base_x = ACT_BASE_X + ux0 * L0 * 0.38 * 1.10   # barrel exit (fixed)
        rod_base_z = ACT_BASE_Z + uz0 * L0 * 0.38 * 1.10

        # Sling height follows the arm tip minus constant strap length
        sling_top_z = tip_z - self._strap_len
        sling_cz    = sling_top_z - SLING_T / 2   # centre of sling panel

        # Force arrow scale
        arrow_len = min(0.30, force_per_act / 100.0 * 0.05)

        strap_idx = 0
        for i, (sign, y_piv) in enumerate(((+1, PIVOT_Y_OFF), (-1, -PIVOT_Y_OFF))):
            arm_body_idx  = i * 2          # main arm body
            strip_body_idx = i * 2 + 1    # chrome strip body

            # Move arm (main body + highlight strip)
            arm_cx = PIVOT_X + ARM_LEN / 2 * math.cos(θ)
            arm_cz = PIVOT_Z + ARM_LEN / 2 * math.sin(θ)
            _move(self._arm_ids[arm_body_idx],  [arm_cx, y_piv, arm_cz], arm_orn)
            _move(self._arm_ids[strip_body_idx], [arm_cx, y_piv, arm_cz], arm_orn)
            _recolor(self._arm_ids[arm_body_idx],  arm_c)
            _recolor(self._arm_ids[strip_body_idx], C_FRAME_LT)

            # Move actuator rod (rod body + clevis end)
            rod_cx = (rod_base_x + act_x) / 2
            rod_cz = (rod_base_z + act_z) / 2
            rod_body_idx   = i * 2
            clevis_body_idx = i * 2 + 1
            _move(self._act_rod_ids[rod_body_idx],
                  [rod_cx, y_piv, rod_cz], rod_orn)
            _move(self._act_rod_ids[clevis_body_idx],
                  [act_x, y_piv, act_z], arm_orn)
            _recolor(self._act_rod_ids[rod_body_idx],    arm_c)
            _recolor(self._act_rod_ids[clevis_body_idx], C_CAP)

            # Suspension straps (from arm tip to sling corners)
            for tx in (TIE_X_F, TIE_X_R):
                sling_corner_z = sling_top_z
                sdx = tx - tip_x
                sdz = sling_corner_z - tip_z
                sL  = math.sqrt(sdx * sdx + sdz * sdz)
                if sL > 1e-6:
                    sorn = _q(0, math.atan2(sdx / sL, sdz / sL), 0)
                else:
                    sorn = _q()
                mid_x = (tip_x + tx) / 2
                mid_z = (tip_z + sling_corner_z) / 2
                _move(self._strap_ids[strap_idx], [mid_x, y_piv, mid_z], sorn)
                _recolor(self._strap_ids[strap_idx], C_TIE)
                strap_idx += 1

            # Force arrows pointing upward from arm tip
            base = [tip_x, y_piv, tip_z]
            tip  = [tip_x, y_piv, tip_z + arrow_len]
            key  = f'arr{i}'
            if key in self._arrows:
                p.addUserDebugLine(base, tip, arr_c, lineWidth=4,
                                   replaceItemUniqueId=self._arrows[key])
            else:
                self._arrows[key] = p.addUserDebugLine(
                    base, tip, arr_c, lineWidth=4, lifeTime=0)

        # ── Sling assembly ─────────────────────────────────────────────
        for bid, ox, oy in self._sling_parts:
            _move(bid, [ox, oy, sling_cz])   # sling_cz is the panel centre
            _recolor(bid, sling_c)

        # ── Lift indicator (ruler pointer) ─────────────────────────────
        # Map sling rise to ruler: sling_top_z at rest → z_bot, at full → z_top
        tip_z_rest = PIVOT_Z + ARM_LEN * math.sin(ARM_ANGLE_DOWN)
        tip_z_full = PIVOT_Z + ARM_LEN * math.sin(ARM_ANGLE_UP)
        z_bot = tip_z_rest - self._strap_len
        z_top = tip_z_full - self._strap_len
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
                            self.load.set_mass(user_mass / 2.0)
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
            self.load.set_mass(s["mass_kg"] / 2.0)
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
