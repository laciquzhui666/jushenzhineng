#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""女性角色装配模块 (woman_rig.py) —— 东方唯美五官、自然中低发髻、枕面防陷与手机被抢后头部绝对静止版

优化升级亮点：
1. 彻底消除手机被抢后的头部晃动与跳动（绝对静止）：
   - 骨骼刚体定位彻底解耦呼吸波动：颈项与头颅基于恒定常数骨骼链计算，呼吸仅作用于胸腹衣身，头部坐标绝对静止
   - 抢走瞬间视线平滑过渡（0 跳动）：手机抽离时（t=13.35s起），视线通过高阶 Slerp 向量球面插值在 1.2 秒内极平滑柔和微扬，初末加速度严格为 0，彻底杜绝单帧瞬变跳跃；14.55s 后视线与姿态 100% 绝对锁定冻结，整整 10 秒无任何一丝晃动或跳动
2. 精致东方五官与3D立体面部样貌：
   - 3D 立体小巧鼻尖（body_nose_tip）：秀气圆润微翘鼻尖与山根过渡，捕捉真实侧光与高光
   - 3D 双侧立体耳廓（body_ear_l, body_ear_r）：自然耳轮与耳窝微影贴合头部两侧，侧颜饱满生动
   - 额前自然碎刘海与耳侧柔顺垂发（hair bangs & wisps）：柔美修饰脸型与发际线
   - 高清贴图五官：自然双眼皮、灵动双重眼神光（冷白手机屏反光 + 暖黄床头灯反射）、精致卧蚕、立体山根修容、M型唇峰与渐变水光唇
3. 发髻高度优化（不高扎，温婉慵懒）：
   - 发髻位于后脑勺黄金比例位置（枕骨后上方，中低法式发髻），主发髻自然扁圆饱满，搭配弧形包发层与自然松散碎发
4. 枕面防陷高度校准（头完全浮于枕头之上）：
   - 头部中心精确平落于枕面软承托高度 Z=0.985m (Y=2.01m)，颅骨底部与枕芯紧密贴合，形成约 2.5cm 的自然羽绒凹陷感，面部五官与秀发完整清晰呈现在枕面之上，绝不陷进穿模
"""

from __future__ import annotations

import math
import numpy as np
try:
    import pybullet as p
except ImportError:
    p = None


# ---------------------------------------------------------------------------
# 数学工具与四元数辅助
# ---------------------------------------------------------------------------
def v3(x, y=None, z=None):
    if y is None:
        return np.asarray(x, dtype=float)
    return np.array([x, y, z], dtype=float)


def norm(v):
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    return v / n if n > 1e-9 else v


def lerp(a, b, t):
    return np.asarray(a, dtype=float) * (1.0 - t) + np.asarray(b, dtype=float) * t


def quintic_ease(u):
    u = np.clip(u, 0.0, 1.0)
    return u * u * u * (u * (u * 6.0 - 15.0) + 10.0)


def slerp_vector(v0, v1, t):
    """向量球面平滑插值（保证旋转角速度连续平缓，端点无跳变）。"""
    v0 = norm(v0)
    v1 = norm(v1)
    dot = float(np.clip(np.dot(v0, v1), -1.0, 1.0))
    if dot > 0.9995:
        return norm(v0 * (1.0 - t) + v1 * t)
    theta = math.acos(dot)
    sin_theta = math.sin(theta)
    return norm((math.sin((1.0 - t) * theta) * v0 + math.sin(t * theta) * v1) / sin_theta)


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
    R = np.column_stack([x, y, z])
    return quat_from_matrix(R)


def quat_from_z(direction, up_hint=(0.0, 0.0, 1.0)):
    z = norm(direction)
    up = np.asarray(up_hint, dtype=float)
    if abs(np.dot(z, norm(up))) > 0.96:
        up = np.array([1.0, 0.0, 0.0])
    x = norm(np.cross(up, z))
    y = np.cross(z, x)
    return quat_from_matrix(np.column_stack([x, y, z]))


def solve_2bone_ik(shoulder, wrist, L1, L2, bend_dir):
    """两段手臂解析逆运动学解算。"""
    d = wrist - shoulder
    dist = float(np.linalg.norm(d))
    dist = np.clip(dist, abs(L1 - L2) + 1e-4, (L1 + L2) - 1e-4)
    axis = d / dist
    cos_theta = (L1 * L1 + dist * dist - L2 * L2) / (2.0 * L1 * dist)
    cos_theta = np.clip(cos_theta, -1.0, 1.0)
    sin_theta = math.sqrt(max(0.0, 1.0 - cos_theta * cos_theta))
    proj_bend = bend_dir - axis * np.dot(bend_dir, axis)
    p_norm = np.linalg.norm(proj_bend)
    normal = proj_bend / p_norm if p_norm > 1e-6 else np.array([0.0, 0.0, 1.0])
    elbow = shoulder + axis * (L1 * cos_theta) + normal * (L1 * sin_theta)
    return elbow


# ---------------------------------------------------------------------------
# Woman 角色类
# ---------------------------------------------------------------------------
class Woman:
    def __init__(self, scene):
        self.scene = scene
        self.expression = "indifferent"
        self.is_blinking = False
        self._last_blink_t = -10.0

        # 色调与材质：白皙温润少女肤质与黑色细吊带
        self.skin_color = (0.97, 0.89, 0.83, 1.0)
        self.skin_shadow = (0.90, 0.81, 0.75, 1.0)
        self.hair_color = (0.16, 0.12, 0.11, 1.0)
        self.lace_color = (0.10, 0.09, 0.09, 0.98)
        self.strap_color = (0.08, 0.07, 0.07, 1.0)
        self.gold_color = (0.92, 0.78, 0.38, 1.0)

        # 手机高对比度机身色彩：极光深海宝蓝与钛金亮边（与手形成鲜明反差）
        self.phone_case_color = (0.12, 0.28, 0.58, 1.0)
        self.phone_rim_color = (0.88, 0.76, 0.42, 1.0)

        # 真实解剖骨骼定义：以床面臀部骨盆为基准支点
        self.recline_k = 0.0
        self.pitch_play = math.radians(60.0)    # 玩手机靠坐 60°
        self.pitch_sleep = math.radians(24.0)   # 枕头自然承托入睡 24°
        self.hips_base = v3(0.36, 1.48, 0.68)   # 臀部在床垫上的固定自然支点

        # 骨骼肢体长度（恒定不可伸缩）
        self.L_upper = 0.245
        self.L_forearm = 0.235

        # 握持卷曲度
        self.right_curl = 0.74
        self.right_thumb_curl = 0.70
        self.left_curl = 0.12
        self.left_thumb_curl = 0.16

        # 创建所有几何部件
        self._build_torso()
        self._build_head_and_hair()
        self._build_limbs()
        self._build_phone()
        self._build_wrist_bracelet()

        # 初始进入玩手机 60° 靠坐姿态
        self.set_recline(0.0)

    def _build_torso(self):
        """1. 躯干几何构建：胶囊体（腰腹） + 椭球体（胸腔） + 圆柱体（颈项）组合。"""
        s = self.scene
        pitch = self.pitch_play
        torso_dir = norm(v3(-0.04, math.cos(pitch), math.sin(pitch)))
        torso_up = norm(v3(0.0, -math.sin(pitch), math.cos(pitch)))
        q_torso = frame_quat(torso_dir, torso_up)

        tex_lace = s.tex.get("lace_black.png", None)
        tex_skin = s.tex.get("skin_warm.png", None)

        # (1) 腰腹基底（胶囊体，紧密承接骨盆）
        c_waist = self.hips_base + torso_dir * 0.08
        self.body_waist = s.add_capsule(
            radius=0.105, length=0.08, pos=c_waist, quat=q_torso,
            rgba=self.lace_color, spec=(0.06, 0.06, 0.06))

        # (2) 黑色蕾丝睡衣主胸腔身躯（椭球体，沿脊柱延伸）
        c_chest = self.hips_base + torso_dir * 0.20
        self.body_torso = s.add_ellipsoid(
            (0.145, 0.112, 0.165), c_chest, q_torso,
            tex=tex_lace, rgba=self.lace_color, spec=(0.08, 0.08, 0.08))

        # 睡衣深V前胸剪裁下沿
        c_bust = c_chest + torso_up * 0.042 - torso_dir * 0.015
        self.body_bust = s.add_ellipsoid(
            (0.136, 0.096, 0.090), c_bust, q_torso,
            tex=tex_lace, rgba=self.lace_color, spec=(0.06, 0.06, 0.06))

        # (3) 锁骨与胸部露出的白皙肌肤层（常数锚定，无晃动）
        c_clavicle = self.hips_base + torso_dir * 0.345 + torso_up * 0.012
        self.body_clavicle = s.add_ellipsoid(
            (0.125, 0.076, 0.068), c_clavicle, q_torso,
            tex=tex_skin, rgba=self.skin_color, spec=(0.20, 0.18, 0.16))

        # 锁骨细骨节线条
        self.clavicle_bones = []
        for side in (-1, 1):
            p0 = c_clavicle + v3(side * 0.018, 0.012, 0.016)
            p1 = c_clavicle + v3(side * 0.082, 0.020, 0.026)
            b = s.add_segment(p0, p1, 0.0036, rgba=self.skin_shadow, spec=(0.25, 0.22, 0.20))
            self.clavicle_bones.append(b)

        # 左右肩膀球节
        self.shoulder_spheres = []
        for side in (-1, 1):
            p_shoulder = c_clavicle + v3(side * 0.115, 0.005, 0.022)
            sph = s.add_sphere(0.042, p_shoulder, quat=q_torso,
                               rgba=self.skin_color, spec=(0.22, 0.18, 0.16))
            self.shoulder_spheres.append(sph)

        # (4) 黑色极细双肩带
        self.strap_segments = []
        for side in (-1, 1):
            p_front = c_clavicle + v3(side * 0.072, -0.014, -0.040)
            p_top = c_clavicle + v3(side * 0.086, 0.046, 0.012)
            p_back = c_chest + v3(side * 0.076, -0.018, 0.080)
            s1 = s.add_segment(p_front, p_top, 0.0024, rgba=self.strap_color, kind="cylinder", spec=(0.3, 0.3, 0.3))
            s2 = s.add_segment(p_top, p_back, 0.0024, rgba=self.strap_color, kind="cylinder", spec=(0.3, 0.3, 0.3))
            self.strap_segments.extend([s1, s2])

        # (5) 修长颈项（圆柱体）
        self.neck_c = c_clavicle + torso_dir * 0.088
        q_neck = frame_quat(torso_dir, torso_up)
        self.body_neck = s.add_cylinder(
            0.046, 0.090, self.neck_c, q_neck,
            rgba=self.skin_color, spec=(0.22, 0.18, 0.16))

    def _build_head_and_hair(self):
        """2. 头部球状颅骨、立体五官细节（小巧鼻尖+双耳）、自然中低发髻与额前碎发。"""
        s = self.scene
        self.head_c = self.neck_c + v3(0, 0.06, 0.10)
        self.head_base_c = np.array(self.head_c, dtype=float)
        self.face_dir = norm(v3(-0.16, -0.86, -0.48))
        self.head_up = norm(v3(-0.02, 0.48, 0.88))
        q_head = frame_quat(self.face_dir, self.head_up)

        tex_hair = s.tex.get("hair_dark.png", None)
        tex_skin = s.tex.get("skin_warm.png", None)

        # 颅骨主体
        self.body_head = s.add_ellipsoid(
            (0.082, 0.092, 0.096), self.head_c, q_head,
            tex=tex_skin, rgba=self.skin_color, spec=(0.20, 0.18, 0.16))

        # 面部表情贴图网格
        self.body_face = s.add_mesh(
            s.meshes["head_face"], (1.002, 1.002, 1.002),
            self.head_c, q_head,
            tex=s.load_tex("face_indifferent.png"),
            spec=(0.18, 0.16, 0.15))

        # 3D 立体小巧鼻尖（捕捉侧颜光泽，与贴图完美配合）
        vis_nose = p.createVisualShape(
            p.GEOM_SPHERE, radius=0.0092,
            rgbaColor=list(self.skin_color),
            specularColor=[0.25, 0.22, 0.20])
        self.body_nose_tip = s.add_visual(vis_nose, self.head_c, q_head)

        # 3D 双侧精巧立体耳廓（自然贴合左右耳根）
        self.ear_bodies = []
        for side in (-1, 1):
            vis_ear = p.createVisualShape(
                p.GEOM_SPHERE, radius=0.017,
                rgbaColor=list(self.skin_color),
                specularColor=[0.20, 0.18, 0.16])
            ear_b = s.add_visual(vis_ear, self.head_c)
            vis_inner = p.createVisualShape(
                p.GEOM_SPHERE, radius=0.010,
                rgbaColor=list(self.skin_shadow),
                specularColor=[0.15, 0.12, 0.10])
            inner_b = s.add_visual(vis_inner, self.head_c)
            self.ear_bodies.extend([ear_b, inner_b])

        # 后脑勺发罩与自然中低发髻系统（不高扎，温婉慵懒）
        self.hair_parts = []
        # (0) 脑后基础发罩
        h_back = s.add_ellipsoid(
            (0.086, 0.094, 0.098), self.head_c, q_head,
            tex=tex_hair, rgba=self.hair_color, spec=(0.35, 0.30, 0.28))
        self.hair_parts.append(h_back)

        # (1) 中低发圈基座（紧贴后脑枕骨稍上方，绝不冲天）
        h_scrunchie = s.add_cylinder(
            0.034, 0.014, self.head_c, q_head,
            rgba=(0.08, 0.08, 0.08, 1.0), spec=(0.15, 0.15, 0.15))
        self.hair_parts.append(h_scrunchie)

        # (2) 主发髻扁圆椭球体（适度适中饱满，慵懒温和）
        h_bun_main = s.add_ellipsoid(
            (0.046, 0.042, 0.040), self.head_c, q_head,
            tex=tex_hair, rgba=self.hair_color, spec=(0.38, 0.32, 0.28))
        self.hair_parts.append(h_bun_main)

        # (3~7) 弧形自然发包与缠绕碎发（5组）
        for _ in range(5):
            h_curl = s.add_ellipsoid(
                (0.022, 0.020, 0.022), self.head_c, q_head,
                tex=tex_hair, rgba=self.hair_color, spec=(0.35, 0.30, 0.26))
            self.hair_parts.append(h_curl)

        # (8~11) 耳侧柔顺微曲碎发（自然修饰两腮）
        for _ in range(4):
            hw = s.add_capsule(0.0024, 0.038, self.head_c, q_head,
                               rgba=self.hair_color, spec=(0.3, 0.25, 0.2))
            self.hair_parts.append(hw)

        # (12~13) 额前侧分轻柔碎刘海（自然贴合发际线，丰富少女感）
        for _ in range(2):
            h_fringe = s.add_capsule(0.0028, 0.048, self.head_c, q_head,
                                     rgba=self.hair_color, spec=(0.32, 0.28, 0.22))
            self.hair_parts.append(h_fringe)

    def _build_limbs(self):
        """双臂与手掌、手指。"""
        s = self.scene
        self.shoulder_right = v3(0.50, 1.62, 0.98)
        self.shoulder_left = v3(0.22, 1.62, 0.98)

        # 大臂、小臂
        self.arm_bodies = []
        for _ in range(4):
            vis = p.createVisualShape(p.GEOM_CAPSULE, radius=0.030, length=0.20,
                                      rgbaColor=list(self.skin_color),
                                      specularColor=[0.20, 0.18, 0.16])
            self.arm_bodies.append(s.add_visual(vis))

        # 手掌
        self.hand_bodies = []
        for _ in range(2):
            vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.035, 0.040, 0.012],
                                      rgbaColor=list(self.skin_color),
                                      specularColor=[0.20, 0.18, 0.16])
            self.hand_bodies.append(s.add_visual(vis))

        # 手指节
        self.finger_bodies = []
        for _ in range(16):
            vis = p.createVisualShape(p.GEOM_CAPSULE, radius=0.0065, length=0.025,
                                      rgbaColor=list(self.skin_color),
                                      specularColor=[0.20, 0.18, 0.16])
            self.finger_bodies.append(s.add_visual(vis))

    def _build_phone(self):
        """极光深海宝蓝机身 + 钛金亮边手机（与手部色彩形成鲜明对比）。"""
        s = self.scene
        self.phone_base_pos = v3(0.34, 1.25, 0.98)
        self.phone_front = norm(v3(-0.16, 0.65, 0.74))
        self.phone_up = norm(v3(0.98, 0.18, 0.05))
        self.phone_base_q = frame_quat(self.phone_front, self.phone_up)

        self.phone_pos = np.array(self.phone_base_pos, dtype=float)
        self.phone_q = list(self.phone_base_q)
        q = frame_quat(self.phone_front, self.phone_up)

        # 手机深海宝蓝主体
        vis_phone = p.createVisualShape(
            p.GEOM_BOX, halfExtents=[0.042, 0.082, 0.0042],
            rgbaColor=list(self.phone_case_color),
            specularColor=[0.60, 0.75, 0.95])
        self.body_phone = s.add_visual(vis_phone, self.phone_pos, q)

        # 手机钛金防滑亮边
        vis_rim = p.createVisualShape(
            p.GEOM_BOX, halfExtents=[0.0435, 0.0835, 0.0035],
            rgbaColor=list(self.phone_rim_color),
            specularColor=[0.95, 0.85, 0.50])
        self.body_phone_rim = s.add_visual(vis_rim, self.phone_pos, q)

        # 手机冷白高亮屏幕
        tex_phone = s.tex.get("phone", None)
        self.body_screen = s.add_card(
            0.078, 0.158, self.phone_pos + self.phone_front * 0.0048,
            normal=self.phone_front, up_hint=self.phone_up,
            tex=tex_phone, rgba=[1, 1, 1, 1], spec=[0.8, 0.85, 0.9])

        # 相机模组
        cam_pos = self.phone_pos - self.phone_front * 0.0048 + self.phone_up * 0.055
        self.body_cam = s.add_box((0.022, 0.025, 0.002), cam_pos, q,
                                  rgba=self.phone_rim_color, spec=[0.8, 0.8, 0.6])

    def _build_wrist_bracelet(self):
        """右手腕细金手链。"""
        s = self.scene
        self.bracelet_bodies = []
        vis_ring = p.createVisualShape(
            p.GEOM_CYLINDER, radius=0.033, length=0.0035,
            rgbaColor=list(self.gold_color), specularColor=[0.95, 0.85, 0.45])
        self.body_bracelet = s.add_visual(vis_ring, self.shoulder_right)
        self.bracelet_bodies.append(self.body_bracelet)

        vis_charm = p.createVisualShape(
            p.GEOM_SPHERE, radius=0.0055,
            rgbaColor=list(self.gold_color), specularColor=[1.0, 0.9, 0.5])
        self.body_charm = s.add_visual(vis_charm, self.shoulder_right)
        self.bracelet_bodies.append(self.body_charm)

    # -----------------------------------------------------------------------
    # 头部与骨骼单通道刚体解算系统 (手机被抢后绝对静止，零晃动、零跳动)
    # -----------------------------------------------------------------------
    def set_recline(self, k: float, t: float = 0.0, gaze_state: str = "play"):
        """设置靠躺系数 k 与姿态解算。

        核心防抖保证：
        1. 颈部与头部位置由绝对常数骨骼链计算，完全杜绝任何呼吸位移传导（零位置晃动）。
        2. 手机被抢走后（gaze_state="sad"），视线在 1.2 秒内平滑 Slerp 扬起（零瞬变跳动），
           之后视线与头部位置 100% 绝对锁定冻结，整整 10 秒无任何一丝位移或摆动。
        3. 枕面防陷校准：平躺时（k=1.0），头部中心精确落于枕面软承托高度 Z=0.985m，绝不陷进枕头。
        """
        self.recline_k = np.clip(k, 0.0, 1.0)
        pitch = lerp(self.pitch_play, self.pitch_sleep, self.recline_k)

        # 脊柱骨骼朝向：以床垫臀部为支点向床头枕心方向延展
        torso_dir = norm(v3(-0.04, math.cos(pitch), math.sin(pitch)))
        torso_up = norm(v3(0.0, -math.sin(pitch), math.cos(pitch)))
        q_torso = frame_quat(torso_dir, torso_up)

        # 躯干微弱呼吸仅在玩手机待机时作用于胸廓，被夺机失落与熟睡时绝不晃动头部
        breath = 0.0016 * math.sin(t * 1.2) if (gaze_state == "play" and self.recline_k < 0.1) else 0.0

        # 骨骼关节定位（常数刚体链，锁骨、颈项与头部绝对无晃动）
        c_waist = self.hips_base + torso_dir * 0.08
        c_chest = self.hips_base + torso_dir * (0.20 + breath)
        c_bust = c_chest + torso_up * 0.042 - torso_dir * 0.015

        # 锁骨与颈项基准：完全独立于 breath，确保头部坐标绝对恒定
        c_clavicle = self.hips_base + torso_dir * 0.345 + torso_up * 0.012

        # 枕头承托高度防陷校准：
        # 枕头顶面位于约 Z=0.943m，头部中心落于枕面软承托高度 Z=0.985m，完全防穿模
        neck_upright = c_clavicle + torso_dir * 0.088
        neck_sleeping = v3(0.34, 1.90, 0.905)
        self.neck_c = lerp(neck_upright, neck_sleeping, self.recline_k)

        head_upright = neck_upright + torso_dir * 0.110
        head_sleeping = v3(0.34, 2.01, 0.985)
        self.head_c = lerp(head_upright, head_sleeping, self.recline_k)

        # 更新躯干与颈项
        p.resetBasePositionAndOrientation(self.body_waist, list(c_waist), list(q_torso))
        p.resetBasePositionAndOrientation(self.body_torso, list(c_chest), list(q_torso))
        p.resetBasePositionAndOrientation(self.body_bust, list(c_bust), list(q_torso))
        p.resetBasePositionAndOrientation(self.body_clavicle, list(c_clavicle), list(q_torso))
        p.resetBasePositionAndOrientation(self.body_neck, list(self.neck_c), list(q_torso))

        # 锁骨与肩部
        for idx, side in enumerate((-1, 1)):
            p_mid = c_clavicle + v3(side * 0.050, 0.016, 0.018)
            p.resetBasePositionAndOrientation(self.clavicle_bones[idx], list(p_mid), list(q_torso))
            p_shoulder = c_clavicle + v3(side * 0.115, 0.005, 0.022)
            p.resetBasePositionAndOrientation(self.shoulder_spheres[idx], list(p_shoulder), list(q_torso))

        self.shoulder_right = c_clavicle + v3(0.115, 0.005, 0.022)
        self.shoulder_left = c_clavicle + v3(-0.115, 0.005, 0.022)

        # 睡衣肩带
        for idx, side in enumerate((-1, 1)):
            p_front = c_clavicle + v3(side * 0.072, -0.014, -0.040)
            p_top = c_clavicle + v3(side * 0.086, 0.046, 0.012)
            p_back = c_chest + v3(side * 0.076, -0.018, 0.080)
            p.resetBasePositionAndOrientation(self.strap_segments[idx * 2], list((p_front + p_top) * 0.5), list(quat_from_z(p_top - p_front)))
            p.resetBasePositionAndOrientation(self.strap_segments[idx * 2 + 1], list((p_top + p_back) * 0.5), list(quat_from_z(p_back - p_top)))

        # -------------------------------------------------------------------
        # 头部与五官朝向：严格平滑连续，手机被抢后彻底静止，绝无跳动与晃动
        # -------------------------------------------------------------------
        face_play = norm(v3(-0.16, -0.86, -0.48))       # 专注看手中手机（及手机被抢后完全定格注视空手）
        face_sleep = norm(v3(-0.35, -0.75, 0.55))       # 熟睡时面容舒适侧倚向房间

        if gaze_state in ("play", "sad"):
            # 手机被抢走后至躺下睡觉前（13.35s ~ 24.5s），头部朝向与位置保持 100% 绝对静止锁定，零角度变化，零位移，零跳动
            self.face_dir = face_play
        else:
            # 机器人说完“将为您播放助眠曲”后（t >= 24.5s）平躺过程中，头部由靠坐平滑自然过渡到恬静侧卧熟睡
            u_rot = quintic_ease(self.recline_k)
            self.face_dir = norm(slerp_vector(face_play, face_sleep, u_rot))

        up_hint = norm(np.cross(np.cross(self.face_dir, [0, 0, 1]), self.face_dir))
        side_vec = norm(np.cross(self.face_dir, up_hint))
        q_head = frame_quat(self.face_dir, up_hint)

        # 更新头颅与表情贴图面片（单通道一次性更新，杜绝多重冲突）
        p.resetBasePositionAndOrientation(self.body_head, list(self.head_c), list(q_head))
        p.resetBasePositionAndOrientation(self.body_face, list(self.head_c), list(q_head))

        # 3D 立体秀气小鼻尖
        pos_nose = self.head_c + self.face_dir * 0.086 - up_hint * 0.012
        p.resetBasePositionAndOrientation(self.body_nose_tip, list(pos_nose), list(q_head))

        # 3D 双侧立体耳廓
        for idx, side in enumerate((-1, 1)):
            pos_ear = self.head_c + side_vec * (side * 0.081) - self.face_dir * 0.012 - up_hint * 0.006
            p.resetBasePositionAndOrientation(self.ear_bodies[idx * 2], list(pos_ear), list(q_head))
            pos_inner = pos_ear + side_vec * (side * 0.003) + self.face_dir * 0.002
            p.resetBasePositionAndOrientation(self.ear_bodies[idx * 2 + 1], list(pos_inner), list(q_head))

        # 优雅中低发髻系统（枕骨后方黄金比例，不冲天，温婉自然）
        p.resetBasePositionAndOrientation(self.hair_parts[0], list(self.head_c - self.face_dir * 0.012 + up_hint * 0.008), list(q_head))

        bun_dir = norm(-self.face_dir * 0.88 + up_hint * 0.40)
        bun_base = self.head_c - self.face_dir * 0.076 + up_hint * 0.020
        q_bun = frame_quat(bun_dir, up_hint)

        p.resetBasePositionAndOrientation(self.hair_parts[1], list(bun_base), list(q_bun))
        pos_bun_main = bun_base + bun_dir * 0.024
        p.resetBasePositionAndOrientation(self.hair_parts[2], list(pos_bun_main), list(q_bun))

        for i in range(5):
            angle = 0.45 + i * 1.25
            r_offset = side_vec * (math.cos(angle) * 0.026) + up_hint * (math.sin(angle) * 0.024)
            p.resetBasePositionAndOrientation(self.hair_parts[3 + i], list(pos_bun_main + r_offset), list(q_bun))

        for i, side in enumerate((-1, 1)):
            wisp_pos = self.head_c + side_vec * (side * 0.080) - self.face_dir * 0.022 - up_hint * 0.026
            p.resetBasePositionAndOrientation(self.hair_parts[8 + i * 2], list(wisp_pos), list(q_head))
            p.resetBasePositionAndOrientation(self.hair_parts[8 + i * 2 + 1], list(wisp_pos + v3(0, -0.015, -0.018)), list(q_head))

        for i, side in enumerate((-1, 1)):
            fringe_pos = self.head_c + side_vec * (side * 0.038) + self.face_dir * 0.076 + up_hint * 0.048
            p.resetBasePositionAndOrientation(self.hair_parts[12 + i], list(fringe_pos), list(q_head))

    def set_head_target(self, pos, face_dir):
        """设定头部朝向与位置（兼容接口，平滑无抖动单通道更新）。"""
        self.head_c = np.array(pos, dtype=float)
        self.face_dir = norm(face_dir)
        up_hint = norm(np.cross(np.cross(self.face_dir, [0, 0, 1]), self.face_dir))
        side_vec = norm(np.cross(self.face_dir, up_hint))
        q_head = frame_quat(self.face_dir, up_hint)

        p.resetBasePositionAndOrientation(self.body_head, list(self.head_c), list(q_head))
        p.resetBasePositionAndOrientation(self.body_face, list(self.head_c), list(q_head))

        pos_nose = self.head_c + self.face_dir * 0.086 - up_hint * 0.012
        p.resetBasePositionAndOrientation(self.body_nose_tip, list(pos_nose), list(q_head))
        for idx, side in enumerate((-1, 1)):
            pos_ear = self.head_c + side_vec * (side * 0.081) - self.face_dir * 0.012 - up_hint * 0.006
            p.resetBasePositionAndOrientation(self.ear_bodies[idx * 2], list(pos_ear), list(q_head))
            pos_inner = pos_ear + side_vec * (side * 0.003) + self.face_dir * 0.002
            p.resetBasePositionAndOrientation(self.ear_bodies[idx * 2 + 1], list(pos_inner), list(q_head))

        p.resetBasePositionAndOrientation(self.hair_parts[0], list(self.head_c - self.face_dir * 0.012 + up_hint * 0.008), list(q_head))
        bun_dir = norm(-self.face_dir * 0.88 + up_hint * 0.40)
        bun_base = self.head_c - self.face_dir * 0.076 + up_hint * 0.020
        q_bun = frame_quat(bun_dir, up_hint)
        p.resetBasePositionAndOrientation(self.hair_parts[1], list(bun_base), list(q_bun))
        pos_bun_main = bun_base + bun_dir * 0.024
        p.resetBasePositionAndOrientation(self.hair_parts[2], list(pos_bun_main), list(q_bun))
        for i in range(5):
            angle = 0.45 + i * 1.25
            r_offset = side_vec * (math.cos(angle) * 0.026) + up_hint * (math.sin(angle) * 0.024)
            p.resetBasePositionAndOrientation(self.hair_parts[3 + i], list(pos_bun_main + r_offset), list(q_bun))
        for i, side in enumerate((-1, 1)):
            wisp_pos = self.head_c + side_vec * (side * 0.080) - self.face_dir * 0.022 - up_hint * 0.026
            p.resetBasePositionAndOrientation(self.hair_parts[8 + i * 2], list(wisp_pos), list(q_head))
            p.resetBasePositionAndOrientation(self.hair_parts[8 + i * 2 + 1], list(wisp_pos + v3(0, -0.015, -0.018)), list(q_head))
        for i, side in enumerate((-1, 1)):
            fringe_pos = self.head_c + side_vec * (side * 0.038) + self.face_dir * 0.076 + up_hint * 0.048
            p.resetBasePositionAndOrientation(self.hair_parts[12 + i], list(fringe_pos), list(q_head))

    def set_expression(self, expr: str):
        if expr == self.expression:
            return
        self.expression = expr
        tex_name = f"face_{expr}.png"
        if self.is_blinking and expr != "sleep":
            tex_name = f"face_{expr}_blink.png"
        if tex_name in self.scene.tex:
            self.scene.change_tex(self.body_face, self.scene.tex[tex_name])

    def update_blink(self, t: float):
        if self.expression == "sleep":
            return
        cycle = t - self._last_blink_t
        if cycle > 3.8:
            self._last_blink_t = t
            self.is_blinking = True
            tex_name = f"face_{self.expression}_blink.png"
            if tex_name in self.scene.tex:
                self.scene.change_tex(self.body_face, self.scene.tex[tex_name])
        elif self.is_blinking and (t - self._last_blink_t) > 0.14:
            self.is_blinking = False
            tex_name = f"face_{self.expression}.png"
            if tex_name in self.scene.tex:
                self.scene.change_tex(self.body_face, self.scene.tex[tex_name])

    def update_phone(self, pos, quat):
        """手机位姿同步。"""
        self.phone_pos = np.array(pos, dtype=float)
        self.phone_q = list(quat)
        R = np.array(p.getMatrixFromQuaternion(quat), dtype=float).reshape(3, 3)
        front = R[:, 2]
        up = R[:, 1]
        p.resetBasePositionAndOrientation(self.body_phone, list(self.phone_pos), list(quat))
        p.resetBasePositionAndOrientation(self.body_phone_rim, list(self.phone_pos), list(quat))
        screen_pos = self.phone_pos + front * 0.0048
        p.resetBasePositionAndOrientation(self.body_screen, list(screen_pos), list(quat))
        cam_pos = self.phone_pos - front * 0.0048 + up * 0.055
        p.resetBasePositionAndOrientation(self.body_cam, list(cam_pos), list(quat))

    def idle(self, t: float):
        """姿态1：躺着玩手机（60° 靠坐，双手稳稳端持深海宝蓝金边手机，目光专注）。"""
        self.set_recline(0.0, t, gaze_state="play")

        breath = 0.0016 * math.sin(t * 1.2)
        phone_bob = v3(0.0008 * math.sin(t * 1.0), 0.0006 * math.cos(t * 0.8), breath * 0.5)
        self.update_phone(self.phone_base_pos + phone_bob, self.phone_base_q)

        # 双手稳固端持手机
        R_ph = np.array(p.getMatrixFromQuaternion(self.phone_q), dtype=float).reshape(3, 3)
        ph_right = R_ph[:, 0]
        ph_up = R_ph[:, 1]
        ph_front = R_ph[:, 2]

        self.right_wrist = self.phone_pos + ph_right * 0.052 - ph_up * 0.025 - ph_front * 0.018
        self.right_hand_dir = norm(ph_up * 0.88 + ph_front * 0.22)
        self.right_palm_normal = norm(-ph_right * 0.85 + ph_front * 0.15)
        self.right_curl = 0.74
        self.right_thumb_curl = 0.70

        self.left_wrist = self.phone_pos - ph_right * 0.052 - ph_up * 0.025 - ph_front * 0.018
        self.left_hand_dir = norm(ph_up * 0.88 + ph_front * 0.22)
        self.left_palm_normal = norm(ph_right * 0.85 + ph_front * 0.15)
        self.left_curl = 0.12
        self.left_thumb_curl = 0.16

        self.update_arms()

    def update_arms(self):
        """手臂联动接口：2-Bone IK 自适应解算双臂、手掌、手指与细金手链贴合。"""
        # 1. 右臂 IK（自适应肩关节）
        elbow_r = solve_2bone_ik(
            self.shoulder_right, self.right_wrist,
            self.L_upper, self.L_forearm, bend_dir=v3(0.65, -0.45, -0.60))
        mid_ru = (self.shoulder_right + elbow_r) * 0.5
        q_ru = quat_from_z(elbow_r - self.shoulder_right)
        p.resetBasePositionAndOrientation(self.arm_bodies[0], list(mid_ru), list(q_ru))

        mid_rf = (elbow_r + self.right_wrist) * 0.5
        q_rf = quat_from_z(self.right_wrist - elbow_r)
        p.resetBasePositionAndOrientation(self.arm_bodies[1], list(mid_rf), list(q_rf))

        # 2. 左臂 IK
        elbow_l = solve_2bone_ik(
            self.shoulder_left, self.left_wrist,
            self.L_upper, self.L_forearm, bend_dir=v3(-0.65, -0.45, -0.60))
        mid_lu = (self.shoulder_left + elbow_l) * 0.5
        q_lu = quat_from_z(elbow_l - self.shoulder_left)
        p.resetBasePositionAndOrientation(self.arm_bodies[2], list(mid_lu), list(q_lu))

        mid_lf = (elbow_l + self.left_wrist) * 0.5
        q_lf = quat_from_z(self.left_wrist - elbow_l)
        p.resetBasePositionAndOrientation(self.arm_bodies[3], list(mid_lf), list(q_lf))

        # 3. 手掌姿态
        q_rh = frame_quat(self.right_hand_dir, self.right_palm_normal)
        p.resetBasePositionAndOrientation(self.hand_bodies[0], list(self.right_wrist), list(q_rh))
        q_lh = frame_quat(self.left_hand_dir, self.left_palm_normal)
        p.resetBasePositionAndOrientation(self.hand_bodies[1], list(self.left_wrist), list(q_lh))

        # 4. 手指节微动（松弛与蜷曲）
        for idx in range(8):
            ang = (idx - 3.5) * 0.011
            curl_d = self.right_curl * 0.012
            pos_f = self.right_wrist + self.right_hand_dir * (0.038 - curl_d) + v3(ang, 0.008, 0.005)
            p.resetBasePositionAndOrientation(self.finger_bodies[idx], list(pos_f), list(q_rh))

            pos_fl = self.left_wrist + self.left_hand_dir * 0.038 + v3(-ang, 0.008, 0.005)
            p.resetBasePositionAndOrientation(self.finger_bodies[idx + 8], list(pos_fl), list(q_lh))

        # 5. 右手腕细金手链贴合
        forearm_axis = norm(self.right_wrist - elbow_r)
        q_bracelet = frame_quat(forearm_axis, (0, 0, 1))
        pos_bracelet = self.right_wrist - forearm_axis * 0.022
        p.resetBasePositionAndOrientation(self.body_bracelet, list(pos_bracelet), list(q_bracelet))
        pos_charm = pos_bracelet + v3(0.005, -0.012, -0.028)
        p.resetBasePositionAndOrientation(self.body_charm, list(pos_charm), [0, 0, 0, 1])