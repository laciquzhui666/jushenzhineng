"""A seven-axis style collaborative robot arm, styled after a clean
white / charcoal cobot with large dark joints, white bands and a white
end effector marked with a small red H.

The arm is posed with visual forward kinematics (no dynamics needed): the
scene feeds it a tool-centre point and an approach direction, and the links
are placed every frame.  This keeps the grab / stow choreography deterministic.
"""
from __future__ import annotations

import math

import numpy as np
import pybullet as p

import meshgen


def v3(x, y=None, z=None):
    if y is None:
        return np.asarray(x, dtype=float)
    return np.array([x, y, z], dtype=float)


def norm(v):
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    return v / n if n > 1e-9 else v


def quat_from_matrix(R):
    R = np.asarray(R, dtype=float)
    t = float(np.trace(R))
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return [x, y, z, w]


def frame_quat(z_axis, y_hint):
    z = norm(z_axis)
    y = np.asarray(y_hint, dtype=float)
    y = norm(y - z * np.dot(y, z))
    if np.linalg.norm(y) < 1e-6:
        y = np.array([1.0, 0.0, 0.0])
    x = norm(np.cross(y, z))
    y = np.cross(z, x)
    return quat_from_matrix(np.column_stack([x, y, z]))


def quat_from_z(direction, up_hint=(0.0, 0.0, 1.0)):
    z = norm(direction)
    up = np.asarray(up_hint, dtype=float)
    if abs(np.dot(z, norm(up))) > 0.96:
        up = np.array([1.0, 0.0, 0.0])
    return frame_quat(z, up)


def two_bone_ik(shoulder, wrist, l1, l2, pole_hint):
    s = np.asarray(shoulder, dtype=float)
    w = np.asarray(wrist, dtype=float)
    dvec = w - s
    d = float(np.linalg.norm(dvec))
    max_d = l1 + l2 - 1e-4
    min_d = abs(l1 - l2) + 1e-4
    if d > max_d:
        w = s + dvec * (max_d / d)
        d = max_d
    elif d < min_d:
        w = s + v3(0, 0, 1) * 0.01 if d < 1e-6 else s + dvec * (min_d / d)
        d = min_d
    u = norm(w - s)
    a = (l1 * l1 - l2 * l2 + d * d) / (2.0 * d)
    h = math.sqrt(max(0.0, l1 * l1 - a * a))
    pole = np.asarray(pole_hint, dtype=float)
    pole = pole - u * np.dot(pole, u)
    if np.linalg.norm(pole) < 1e-6:
        pole = np.cross(u, [0, 0, 1])
        if np.linalg.norm(pole) < 1e-6:
            pole = np.array([1.0, 0.0, 0.0])
    return s + u * a + norm(pole) * h, w


class RobotArm:
    WHITE = (0.965, 0.955, 0.930, 1.0)
    WHITE_SOFT = (0.885, 0.885, 0.860, 1.0)
    DARK = (0.125, 0.138, 0.155, 1.0)
    DARK_SOFT = (0.190, 0.205, 0.225, 1.0)
    GRAY = (0.690, 0.710, 0.730, 1.0)
    LIGHT = (0.800, 0.815, 0.825, 1.0)
    RED = (0.760, 0.075, 0.055, 1.0)
    RED_DARK = (0.360, 0.035, 0.030, 1.0)

    def __init__(self, scene, base=None):
        self.s = scene
        self.base = v3(base if base is not None else (-0.43, 1.93, 0.56))
        self.shoulder = self.base + v3(0, 0, 0.34)   # 0.90 m above the floor
        self.L1 = 0.46
        self.L2 = 0.42
        self.GRIP = 0.13
        self.hatch_angle = 0.0
        self.jaw_open = 0.008
        self.tcp = None
        self.approach = v3(0.3, -0.5, -0.8)
        self._build_base()
        self._build_arm()
        self.set_pose(self.home_tcp, self.home_approach, jaw_open=0.008)

    # ---- base -------------------------------------------------------------
    def _build_base(self):
        s = self.s
        b = self.base
        # thin shadow seam + white mounting flange
        s.add_box((0.130, 0.106, 0.0025), b + v3(0, 0, 0.004),
                  rgba=(0.16, 0.16, 0.15, 1.0), spec=(0.2, 0.2, 0.2))
        s.add_box((0.132, 0.108, 0.010), b + v3(0, 0, 0.014),
                  rgba=self.WHITE_SOFT, spec=(0.35, 0.35, 0.35))
        # lower storage / drawer module
        module = meshgen.rounded_rect_panel(
            "cobot_storage_base", 0.238, 0.180, 0.030,
            thickness=0.130, segments=14)
        self.base_module = s.add_mesh(
            module, (1, 1, 1), b + v3(0, 0, 0.075),
            rgba=self.WHITE, spec=(0.38, 0.37, 0.35))
        # thin dark trim ring where the column meets the drawer module
        s.add_cylinder(0.081, 0.012, b + v3(0, 0, 0.140),
                       rgba=self.DARK_SOFT, spec=(0.35, 0.35, 0.36))
        # white pedestal column
        self.column = s.add_cylinder(
            0.078, 0.190, b + v3(0, 0, 0.230),
            quat=quat_from_z((0, 0, 1)),
            rgba=self.WHITE, spec=(0.40, 0.39, 0.37))
        # dark collar bands
        s.add_cylinder(0.081, 0.024, b + v3(0, 0, 0.165),
                       rgba=self.DARK, spec=(0.35, 0.35, 0.36))
        s.add_cylinder(0.081, 0.020, b + v3(0, 0, 0.285),
                       rgba=self.DARK, spec=(0.35, 0.35, 0.36))
        # soft white shoulder collar
        s.add_ellipsoid((0.086, 0.086, 0.056),
                        b + v3(0, 0, 0.322),
                        rgba=self.WHITE_SOFT, spec=(0.35, 0.34, 0.32))
        s.add_cylinder(0.089, 0.024, b + v3(0, 0, 0.340),
                       rgba=self.WHITE, spec=(0.38, 0.37, 0.35))
        # small dark service port + red status light
        s.add_cylinder(0.014, 0.010, b + v3(0.061, -0.030, 0.245),
                       quat=quat_from_z((1, 0, 0)),
                       rgba=self.DARK, spec=(0.3, 0.3, 0.3))
        self.status_led = s.add_ellipsoid(
            (0.006, 0.006, 0.004),
            b + v3(-0.060, -0.092, 0.155),
            rgba=(1.0, 0.23, 0.12, 1.0), spec=(0.8, 0.2, 0.1))
        # ---- drawer in the lower module --------------------------------
        s.add_card(0.194, 0.096, b + v3(0.0, -0.0920, 0.086),
                   normal=(0, -1, 0), up_hint=(0, 0, 1),
                   rgba=(0.012, 0.014, 0.018, 1.0),
                   spec=(0.0, 0.0, 0.0))
        s.add_box((0.092, 0.002, 0.004), b + v3(0.0, -0.0910, 0.126),
                  rgba=self.DARK, spec=(0.25, 0.25, 0.27))
        drawer_path = meshgen.rounded_rect_panel(
            "cobot_drawer_front", 0.208, 0.100, 0.024,
            thickness=0.012, segments=10)
        q_face = frame_quat((0, -1, 0), (0, 0, 1))
        front_pos = b + v3(0.0, -0.104, 0.086)
        self.drawer_front = s.add_mesh(
            drawer_path, (1, 1, 1), front_pos, q_face,
            rgba=self.WHITE, spec=(0.40, 0.38, 0.36))
        # thin dark seam around the front
        seam_pos = b + v3(0.0, -0.1030, 0.086)
        seam = s.add_box((0.108, 0.0025, 0.053), seam_pos, q_face,
                         rgba=self.DARK_SOFT, spec=(0.2, 0.2, 0.2))
        # shallow handle slot and an indicator LED
        handle = s.add_box((0.055, 0.002, 0.004),
                           b + v3(0.0, -0.111, 0.068), q_face,
                           rgba=self.DARK, spec=(0.2, 0.2, 0.2))
        led = s.add_ellipsoid(
            (0.004, 0.003, 0.004),
            b + v3(0.078, -0.111, 0.118),
            rgba=(1.0, 0.22, 0.10, 1.0), spec=(0.8, 0.2, 0.1))
        # the dark opening sits behind the seam box: cut it visually with
        # a second, slightly larger dark card.
        s.add_card(0.214, 0.108, b + v3(0.0, -0.1026, 0.086),
                   normal=(0, -1, 0), up_hint=(0, 0, 1),
                   rgba=(0.055, 0.058, 0.062, 1.0),
                   spec=(0.05, 0.05, 0.05))
        self.drawer_bodies = [
            (self.drawer_front, np.array(front_pos), q_face),
            (seam, np.array(seam_pos), q_face),
            (handle, np.array(b + v3(0.0, -0.111, 0.068)), q_face),
            (led, np.array(b + v3(0.078, -0.111, 0.118)), q_face),
        ]
        # dark tray just behind the front panel, sized to fit inside the module
        tray_center = b + v3(0.0, -0.036, 0.078)
        tray_col = (0.075, 0.080, 0.090, 1.0)
        for half, off in (
            ((0.086, 0.056, 0.005), (0.0, 0.0, 0.0)),
            ((0.006, 0.056, 0.018), (-0.086, 0.0, 0.020)),
            ((0.006, 0.056, 0.018), (0.086, 0.0, 0.020)),
            ((0.086, 0.006, 0.018), (0.0, 0.020, 0.020)),
        ):
            pos = tray_center + v3(*off)
            body = s.add_box(half, pos, rgba=tray_col,
                             spec=(0.30, 0.30, 0.32))
            self.drawer_bodies.append((body, np.array(pos), (0, 0, 0, 1)))
        tray_rim = s.add_box((0.078, 0.003, 0.003),
                             tray_center + v3(0.0, 0.018, 0.014),
                             rgba=self.LIGHT, spec=(0.5, 0.5, 0.5))
        self.drawer_bodies.append((tray_rim, np.array(
            tray_center + v3(0.0, 0.018, 0.014)), (0, 0, 0, 1)))
        self.update_hatch(0.0)

    # ---- arm --------------------------------------------------------------
    def _build_arm(self):
        s = self.s
        # dark, thick main links with white bands
        self.upper_link = s.add_capsule(
            0.058, self.L1 - 2 * 0.058, (0, 0, -10),
            rgba=self.DARK, spec=(0.48, 0.46, 0.44))
        self.fore_link = s.add_capsule(
            0.055, self.L2 - 2 * 0.055, (0, 0, -10),
            rgba=self.DARK, spec=(0.48, 0.46, 0.44))
        self.upper_sleeves = [
            s.add_cylinder(0.0615, 0.030, (0, 0, -10),
                           rgba=self.WHITE, spec=(0.38, 0.37, 0.35))
            for _ in range(2)
        ]
        self.fore_sleeves = [
            s.add_cylinder(0.0585, 0.028, (0, 0, -10),
                           rgba=self.WHITE, spec=(0.38, 0.37, 0.35))
            for _ in range(2)
        ]
        # big dark joints with light gray circular caps
        self.shoulder_joint = s.add_cylinder(
            0.083, 0.108, (0, 0, -10),
            rgba=self.DARK, spec=(0.42, 0.42, 0.43))
        self.shoulder_caps = [
            s.add_cylinder(0.060, 0.016, (0, 0, -10),
                           rgba=self.LIGHT, spec=(0.55, 0.55, 0.55))
            for _ in range(2)
        ]
        self.shoulder_hubs = [
            s.add_cylinder(0.035, 0.018, (0, 0, -10),
                           rgba=self.GRAY, spec=(0.45, 0.45, 0.45))
            for _ in range(2)
        ]
        self.elbow_joint = s.add_cylinder(
            0.071, 0.094, (0, 0, -10),
            rgba=self.DARK, spec=(0.42, 0.42, 0.43))
        self.elbow_caps = [
            s.add_cylinder(0.050, 0.014, (0, 0, -10),
                           rgba=self.LIGHT, spec=(0.55, 0.55, 0.55))
            for _ in range(2)
        ]
        self.elbow_hubs = [
            s.add_cylinder(0.028, 0.016, (0, 0, -10),
                           rgba=self.GRAY, spec=(0.45, 0.45, 0.45))
            for _ in range(2)
        ]
        # wrist hardware and white tool head
        self.wrist_joint = s.add_cylinder(
            0.055, 0.028, (0, 0, -10),
            rgba=self.DARK, spec=(0.42, 0.42, 0.43))
        self.wrist_ring = s.add_cylinder(
            0.060, 0.012, (0, 0, -10),
            rgba=self.LIGHT, spec=(0.55, 0.55, 0.55))
        self.end_body = s.add_cylinder(
            0.049, 0.080, (0, 0, -10),
            rgba=self.WHITE, spec=(0.40, 0.39, 0.37))
        self.end_nose = s.add_ellipsoid(
            (0.048, 0.048, 0.036), (0, 0, -10),
            rgba=self.WHITE_SOFT, spec=(0.40, 0.39, 0.37))
        self.end_collar = s.add_cylinder(
            0.034, 0.016, (0, 0, -10),
            rgba=self.DARK, spec=(0.35, 0.35, 0.36))
        # red H logo on both sides of the white tool head
        self.h_bars = []
        for _side in (-1, 1):
            for _ in range(3):
                self.h_bars.append(s.add_box(
                    (0.0016, 0.014, 0.0026) if _ != 1
                    else (0.0016, 0.0026, 0.013),
                    (0, 0, -10), rgba=self.RED, spec=(0.3, 0.08, 0.07)))
        # small, dark gripper fingers (closed they read as the black tool tip)
        self.jaws = []
        self.pads = []
        for _ in range(2):
            self.jaws.append(s.add_box(
                (0.007, 0.011, 0.033), (0, 0, -10),
                rgba=self.DARK_SOFT, spec=(0.3, 0.3, 0.32)))
            self.pads.append(s.add_box(
                (0.002, 0.009, 0.024), (0, 0, -10),
                rgba=(0.018, 0.019, 0.022, 1.0), spec=(0.15, 0.15, 0.15)))
        self.jaw_open = 0.008
        self.jaw_axis = v3(1, 0, 0)
        self.gripper_q = (0, 0, 0, 1)
        self.wrist = v3(0, 0, -10)
        self.elbow = v3(0, 0, -10)
        self.plane_n = v3(0, 1, 0)

    # ---- poses ------------------------------------------------------------
    @property
    def home_tcp(self):
        return self.base + v3(0.31, -0.31, 0.62)

    @property
    def home_approach(self):
        return norm(v3(0.65, -0.35, -0.68))

    @property
    def store_tcp(self):
        return self.base + v3(0.0, -0.18, 0.29)

    @property
    def store_approach(self):
        return norm(v3(0.0, -0.30, -0.95))

    def set_pose(self, tcp, approach, jaw_open=None):
        tcp = np.asarray(tcp, dtype=float)
        A = norm(approach)
        self.tcp = tcp
        self.approach = A
        if jaw_open is not None:
            self.jaw_open = float(jaw_open)
        wrist_target = tcp - A * self.GRIP
        elbow, wrist = two_bone_ik(
            self.shoulder, wrist_target, self.L1, self.L2,
            v3(0.20, 0.05, 1.0))
        self.elbow = elbow
        self.wrist = wrist

        # arm plane and joint axes
        d1 = norm(elbow - self.shoulder)
        d2 = norm(wrist - elbow)
        plane_n = np.cross(d1, d2)
        if np.linalg.norm(plane_n) < 1e-6:
            plane_n = np.cross(d1, [0, 0, 1])
        plane_n = norm(plane_n)
        if np.dot(plane_n, v3(0, -1, 0)) < 0:
            plane_n = -plane_n
        self.plane_n = plane_n

        # main links
        q_up = quat_from_z(elbow - self.shoulder)
        p.resetBasePositionAndOrientation(
            self.upper_link, list((self.shoulder + elbow) * 0.5), list(q_up))
        q_fore = quat_from_z(wrist - elbow)
        p.resetBasePositionAndOrientation(
            self.fore_link, list((elbow + wrist) * 0.5), list(q_fore))
        # white bands on the dark links
        for frac, body in zip((0.19, 0.81), self.upper_sleeves):
            pos = self.shoulder + (elbow - self.shoulder) * frac
            p.resetBasePositionAndOrientation(body, list(pos), list(q_up))
        for frac, body in zip((0.20, 0.80), self.fore_sleeves):
            pos = elbow + (wrist - elbow) * frac
            p.resetBasePositionAndOrientation(body, list(pos), list(q_fore))

        # large joints with gray caps
        q_joint = quat_from_z(plane_n)
        p.resetBasePositionAndOrientation(
            self.shoulder_joint, list(self.shoulder), list(q_joint))
        p.resetBasePositionAndOrientation(
            self.elbow_joint, list(elbow), list(q_joint))
        for sign, cap, hub in zip((-1, 1), self.shoulder_caps,
                                  self.shoulder_hubs):
            p.resetBasePositionAndOrientation(
                cap, list(self.shoulder + plane_n * (sign * 0.062)),
                list(q_joint))
            p.resetBasePositionAndOrientation(
                hub, list(self.shoulder + plane_n * (sign * 0.071)),
                list(q_joint))
        for sign, cap, hub in zip((-1, 1), self.elbow_caps,
                                  self.elbow_hubs):
            p.resetBasePositionAndOrientation(
                cap, list(elbow + plane_n * (sign * 0.054)),
                list(q_joint))
            p.resetBasePositionAndOrientation(
                hub, list(elbow + plane_n * (sign * 0.063)),
                list(q_joint))

        # wrist and white end effector
        p.resetBasePositionAndOrientation(
            self.wrist_joint, list(wrist), list(quat_from_z(A)))
        p.resetBasePositionAndOrientation(
            self.wrist_ring, list(wrist + A * 0.020), list(quat_from_z(A)))
        p.resetBasePositionAndOrientation(
            self.end_body, list(wrist + A * 0.055), list(quat_from_z(A)))
        p.resetBasePositionAndOrientation(
            self.end_nose, list(wrist + A * 0.095), list(quat_from_z(A)))
        p.resetBasePositionAndOrientation(
            self.end_collar, list(wrist + A * 0.095), list(quat_from_z(A)))

        # gripper orientation: local Z = approach, local X = jaw axis
        jaw_axis = np.cross(A, v3(0, 0, 1))
        if np.linalg.norm(jaw_axis) < 0.05:
            jaw_axis = np.cross(A, v3(0, 1, 0))
        jaw_axis = norm(jaw_axis - A * np.dot(jaw_axis, A))
        self.jaw_axis = jaw_axis
        y = np.cross(A, jaw_axis)
        R = np.column_stack([jaw_axis, y, A])
        self.gripper_q = quat_from_matrix(R)

        # red H on the two sides of the white tool head
        h_center = wrist + A * 0.050
        h_q = quat_from_matrix(R)
        idx = 0
        for side in (-1, 1):
            for kind in range(3):
                if kind == 0:
                    half = np.array([0.0016, 0.014, 0.0026])
                    local = np.array([0.0, 0.0, -0.009])
                elif kind == 1:
                    half = np.array([0.0016, 0.014, 0.0026])
                    local = np.array([0.0, 0.0, 0.009])
                else:
                    half = np.array([0.0016, 0.0026, 0.013])
                    local = np.array([0.0, 0.0, 0.0])
                pos = (h_center + jaw_axis * (side * 0.0505)
                       + A * local[2] + y * local[1])
                p.resetBasePositionAndOrientation(
                    self.h_bars[idx], list(pos), list(h_q))
                idx += 1

        # small dark fingers, closed around the phone at grab time
        offset = 0.010 + self.jaw_open * 0.5
        for sign, jaw, pad in zip((-1, 1), self.jaws, self.pads):
            centre = wrist + A * 0.136 + jaw_axis * (sign * offset)
            p.resetBasePositionAndOrientation(jaw, list(centre), list(h_q))
            pad_pos = centre - jaw_axis * (sign * 0.006)
            p.resetBasePositionAndOrientation(pad, list(pad_pos), list(h_q))

    def update_hatch(self, amount):
        amount = float(np.clip(amount, 0.0, 1.0))
        offset = v3(0.0, -0.13 * amount, 0.0)
        for body, rest_pos, rest_q in self.drawer_bodies:
            p.resetBasePositionAndOrientation(
                body, list(rest_pos + offset), list(rest_q))
        self.hatch_angle = amount

    def set_open(self, amount):
        """0 closed .. 1 fully open (drawer slides out to the front)."""
        self.update_hatch(amount)
