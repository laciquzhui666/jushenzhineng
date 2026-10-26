#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""程序化高精度材质生成器 (textures.py) —— 东方温婉精致五官与清透少女美肤升级版

优化亮点：
1. 极致精细东方温婉五官：
   - 眉形：立体根根分明微弓自然眉（眉头柔淡、眉弓微扬、眉尾自然柔化收尖）
   - 眼部：精致天然开扇双眼皮折痕 + 细腻上眼线与卷翘睫毛 + 饱满立体“卧蚕”高光与光泽
   - 瞳孔：深琥珀/黑褐渐变瞳孔 + 手机冷白反光与床头暖黄双重清透眼神光
   - 鼻部：细腻山根与鼻梁立体微光影 + 精巧微翘小鼻尖 + 极细柔和鼻翼微影
   - 唇部：立体M型丘比特之弓唇峰 + 豆沙蜜桃渐变水光唇 + 唇珠柔润高光点与嘴角自然梨涡
2. 肤质与气色：
   - 白皙通透微泛暖晕的温润肤质（warm porcelain skin）+ 苹果肌通透自然的蜜桃淡粉腮红
3. 黑色蕾丝睡衣与顺滑发丝纹理
"""

from __future__ import annotations

import math
import os
import numpy as np
from PIL import Image, ImageDraw, ImageFilter

ROOT = os.path.dirname(os.path.abspath(__file__))
ASSETS = os.path.join(ROOT, "assets")
TEX_DIR = os.path.join(ASSETS, "textures")
CLOCK_DIR = os.path.join(ASSETS, "clock")
PHONE_DIR = os.path.join(ASSETS, "phone")
SPEECH_DIR = os.path.join(ASSETS, "speech")


def save_rgba(name: str, arr: np.ndarray):
    os.makedirs(TEX_DIR, exist_ok=True)
    path = os.path.join(TEX_DIR, name)
    img = Image.fromarray(np.clip(arr, 0, 255).astype(np.uint8), mode="RGBA")
    img.save(path)


def make_lace_black():
    """生成精致黑色蕾丝纹理（精美镂空花案与细腻网格织造）。"""
    w, h = 512, 512
    img = Image.new("RGBA", (w, h), (18, 16, 17, 240))
    draw = ImageDraw.Draw(img)

    step = 16
    for y in range(0, h, step):
        for x in range(0, w, step):
            shift = (step // 2) if (y // step) % 2 == 1 else 0
            cx, cy = (x + shift) % w, y
            draw.ellipse([cx - 4, cy - 4, cx + 4, cy + 4], fill=(12, 10, 11, 160))
            draw.line([(cx, cy - 6), (cx + 5, cy - 2), (cx + 5, cy + 3),
                       (cx, cy + 6), (cx - 5, cy + 3), (cx - 5, cy - 2), (cx, cy - 6)],
                      fill=(32, 28, 30, 255), width=1)

    for y in range(0, h, 64):
        for x in range(0, w, 64):
            draw.arc([x, y, x + 48, y + 48], 0, 180, fill=(45, 40, 42, 255), width=2)
            draw.arc([x + 12, y + 12, x + 36, y + 36], 180, 360, fill=(40, 35, 38, 255), width=2)
            draw.ellipse([x + 20, y + 20, x + 28, y + 28], fill=(30, 26, 28, 250))

    img = img.filter(ImageFilter.SMOOTH)
    save_rgba("lace_black.png", np.array(img))


def make_skin_warm():
    """白皙通透微泛暖晕的少女柔肤材质。"""
    w, h = 512, 512
    base = np.array([250, 234, 222, 255], dtype=np.float32)
    rng = np.random.default_rng(101)
    noise = rng.normal(0, 1.8, (h, w, 1)).astype(np.float32)
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
    glow = np.sin(xx * 0.02) * np.cos(yy * 0.02) * 1.8
    arr = base + noise
    arr[..., 0] += glow * 1.2
    arr[..., 1] += glow * 0.6
    arr[..., 3] = 255.0
    save_rgba("skin_warm.png", arr)


def make_hair_dark():
    """自然黑褐色发丝纹理与柔和高光。"""
    w, h = 512, 512
    arr = np.zeros((h, w, 4), dtype=np.float32)
    rng = np.random.default_rng(202)
    strands = rng.uniform(25, 45, (1, w, 1))
    fade = np.linspace(0.85, 1.15, h)[:, None, None]
    arr[..., :3] = strands * fade
    arr[..., 0] *= 1.12
    arr[..., 1] *= 0.98
    arr[..., 2] *= 0.92
    arr[..., 3] = 255.0
    save_rgba("hair_dark.png", arr)


def draw_face_canvas(expr: str, blink: bool = False) -> np.ndarray:
    """绘制高精细度唯美东方女性五官样貌（自然双眼皮、卧蚕、秀气小鼻、渐变水光唇）。"""
    w, h = 512, 512
    img = Image.new("RGBA", (w, h), (250, 234, 222, 255))
    draw = ImageDraw.Draw(img)

    # 1. 柔美温润通透腮红（苹果肌自然淡粉晕染）
    for side in (-1, 1):
        cx = int(w * (0.5 + side * 0.23))
        cy = int(h * 0.56)
        for r in range(54, 0, -3):
            alpha = int((1.0 - (r / 54.0) ** 1.3) * 32)
            draw.ellipse([cx - r, cy - int(r * 0.6), cx + r, cy + int(r * 0.6)], fill=(245, 138, 145, alpha))

    # 2. 细致唯美东方平眉 / 微弓野生眉（根根分明柔雾质感）
    for side in (-1, 1):
        # 眉形骨架点
        p0 = (w * (0.5 + side * 0.08), h * 0.355)
        p1 = (w * (0.5 + side * 0.21), h * 0.332)
        p2 = (w * (0.5 + side * 0.33), h * 0.368)

        if expr == "sad":
            # 眉头微蹙略向上抬，显露惹人怜惜的失落
            p0 = (w * (0.5 + side * 0.08), h * 0.345)
            p1 = (w * (0.5 + side * 0.20), h * 0.330)

        # 柔雾眉底色
        draw.line([p0, p1, p2], fill=(70, 52, 48, 160), width=6)
        # 精细毛流仿真
        for s_idx in range(12):
            u = s_idx / 11.0
            bx = p0[0] * (1 - u) ** 2 + 2 * (1 - u) * u * p1[0] + u ** 2 * p2[0]
            by = p0[1] * (1 - u) ** 2 + 2 * (1 - u) * u * p1[1] + u ** 2 * p2[1]
            draw.line([(bx, by + 2), (bx + side * 1.5, by - 3.5)], fill=(48, 36, 32, 210), width=2)

    # 3. 唯美灵动眼部系统（双眼皮折痕 + 卷翘睫毛 + 饱满卧蚕 + 清澈瞳孔）
    for side in (-1, 1):
        cx = int(w * (0.5 + side * 0.20))
        cy = int(h * 0.445)

        # (1) 饱满自然“卧蚕”（Aegyo Sal）：下方微暗阴影 + 饱满珠光提亮
        draw.arc([cx - 24, cy + 6, cx + 24, cy + 24], 25, 155, fill=(210, 175, 165, 120), width=2)
        draw.arc([cx - 20, cy + 2, cx + 20, cy + 16], 20, 160, fill=(255, 248, 240, 160), width=3)

        if blink or expr == "sleep":
            # 闭眼 / 熟睡：优雅柔和月牙长睫毛
            draw.arc([cx - 28, cy - 10, cx + 28, cy + 16], 15, 165, fill=(38, 28, 26, 255), width=4)
            # 睫毛下垂自然分簇
            for t_step in (-20, -12, -4, 4, 12, 20):
                draw.line([(cx + t_step, cy + 8), (cx + t_step + side * 3, cy + 16)], fill=(38, 28, 26, 210), width=2)
            # 双眼皮闭合时的浅浅褶皱
            draw.arc([cx - 26, cy - 18, cx + 26, cy + 8], 15, 165, fill=(200, 168, 156, 130), width=2)
        else:
            # 开眼：自然开扇双眼皮折痕
            draw.arc([cx - 26, cy - 25, cx + 26, cy + 8], 195, 345, fill=(195, 158, 146, 170), width=2)

            # 精致上眼线（眼尾微微上扬延长）
            pts_lash = [
                (cx - 28 * side, cy - 2),
                (cx - 14 * side, cy - 10),
                (cx, cy - 11),
                (cx + 18 * side, cy - 7),
                (cx + 31 * side, cy - 9),
            ]
            if side == 1:
                draw.line(pts_lash, fill=(32, 22, 20, 255), width=4)
            else:
                draw.line(list(reversed(pts_lash)), fill=(32, 22, 20, 255), width=4)

            # 外眼角纤长卷翘眼睫毛
            for lash_ang, lash_len in ((30, 7), (15, 9), (0, 8), (-15, 7)):
                rad = math.radians(lash_ang)
                tip_x = cx + 28 * side + math.cos(rad) * lash_len * side
                tip_y = cy - 8 - math.sin(rad) * lash_len
                draw.line([(cx + 27 * side, cy - 8), (tip_x, tip_y)], fill=(32, 22, 20, 230), width=2)

            # 下眼睑柔和轮廓
            draw.arc([cx - 24, cy - 10, cx + 24, cy + 14], 15, 165, fill=(75, 58, 54, 180), width=2)

            # 乌黑清澈瞳孔（虹膜渐变）
            pupil_offset_y = 2 if expr == "indifferent" else -1
            draw.ellipse([cx - 13, cy - 7 + pupil_offset_y, cx + 13, cy + 13 + pupil_offset_y], fill=(42, 28, 24, 255))
            draw.ellipse([cx - 10, cy - 4 + pupil_offset_y, cx + 10, cy + 10 + pupil_offset_y], fill=(22, 14, 12, 255))

            # 双重眼神高光（手机屏幕冷光点 + 床头灯暖黄色光斑）
            # 手机冷白眼神光（灵动清晰）
            draw.ellipse([cx - 6, cy - 3 + pupil_offset_y, cx - 1, cy + 2 + pupil_offset_y], fill=(245, 252, 255, 255))
            # 暖黄环境光光斑
            draw.ellipse([cx + 3, cy + 3 + pupil_offset_y, cx + 6, cy + 6 + pupil_offset_y], fill=(255, 238, 195, 200))

    # 4. 秀丽小巧鼻部（山根修容 + 鼻梁柔和高光 + 圆润小鼻尖 + 极细鼻翼）
    # (1) 鼻梁中心立体微光
    draw.line([(w * 0.5, h * 0.44), (w * 0.5, h * 0.58)], fill=(255, 248, 242, 190), width=3)
    # (2) 鼻侧温和柔影
    draw.line([(w * 0.5 - 7, h * 0.45), (w * 0.5 - 6, h * 0.57)], fill=(218, 185, 170, 110), width=3)
    draw.line([(w * 0.5 + 7, h * 0.45), (w * 0.5 + 6, h * 0.57)], fill=(218, 185, 170, 110), width=3)
    # (3) 秀气微翘小鼻尖
    draw.ellipse([w * 0.5 - 7, h * 0.59, w * 0.5 + 7, h * 0.59 + 8], fill=(242, 185, 175, 180))
    draw.ellipse([w * 0.5 - 3, h * 0.595, w * 0.5 + 3, h * 0.595 + 4], fill=(255, 245, 240, 220))
    # (4) 柔和精细鼻翼微影
    draw.arc([w * 0.5 - 15, h * 0.585, w * 0.5 - 5, h * 0.615], 70, 240, fill=(205, 165, 150, 150), width=2)
    draw.arc([w * 0.5 + 5, h * 0.585, w * 0.5 + 15, h * 0.615], 300, 470, fill=(205, 165, 150, 150), width=2)

    # 5. 精致立体水光唇彩（M型唇峰 + 豆沙蜜桃渐变 + 晶莹唇光）
    lip_c = (int(w * 0.5), int(h * 0.715))
    if expr == "sleep":
        # 熟睡恬淡柔美唇线
        draw.arc([lip_c[0] - 22, lip_c[1] - 8, lip_c[0] + 22, lip_c[1] + 12], 15, 165, fill=(215, 108, 120, 210), width=3)
        draw.ellipse([lip_c[0] - 18, lip_c[1] - 3, lip_c[0] + 18, lip_c[1] + 9], fill=(235, 128, 138, 150))
    elif expr == "sad":
        # 失落微撇小嘴（微向下垂嘴角，软萌委屈）
        draw.arc([lip_c[0] - 22, lip_c[1] + 3, lip_c[0] + 22, lip_c[1] + 23], 195, 345, fill=(208, 96, 108, 240), width=4)
        draw.ellipse([lip_c[0] - 16, lip_c[1] - 1, lip_c[0] + 16, lip_c[1] + 8], fill=(228, 118, 128, 170))
        # 嘴角小小暗影点
        draw.ellipse([lip_c[0] - 23, lip_c[1] + 10, lip_c[0] - 20, lip_c[1] + 13], fill=(160, 75, 85, 180))
        draw.ellipse([lip_c[0] + 20, lip_c[1] + 10, lip_c[0] + 23, lip_c[1] + 13], fill=(160, 75, 85, 180))
    else:
        # 平静专注的自然微合水光唇（精致丘比特弓M唇峰）
        # 上唇 M 弧线
        pts_upper = [
            (lip_c[0] - 22, lip_c[1] + 1),
            (lip_c[0] - 11, lip_c[1] - 7),
            (lip_c[0], lip_c[1] - 3),
            (lip_c[0] + 11, lip_c[1] - 7),
            (lip_c[0] + 22, lip_c[1] + 1),
        ]
        draw.line(pts_upper, fill=(218, 100, 112, 240), width=3)
        # 下唇饱满丰润轮廓
        draw.arc([lip_c[0] - 20, lip_c[1] - 4, lip_c[0] + 20, lip_c[1] + 13], 10, 170, fill=(218, 100, 112, 230), width=4)
        # 内部蜜桃色渐变饱满层
        draw.ellipse([lip_c[0] - 17, lip_c[1] - 2, lip_c[0] + 17, lip_c[1] + 9], fill=(238, 122, 134, 180))
        # 唇珠与下唇晶莹剔透水光高光点
        draw.ellipse([lip_c[0] - 5, lip_c[1] + 2, lip_c[0] + 5, lip_c[1] + 6], fill=(255, 232, 236, 230))
        draw.ellipse([lip_c[0] - 2, lip_c[1] - 4, lip_c[0] + 2, lip_c[1] - 2], fill=(255, 235, 238, 200))

    img = img.filter(ImageFilter.SMOOTH_MORE)
    return np.array(img)


def make_all_faces():
    for expr in ("indifferent", "sad", "sleep"):
        save_rgba(f"face_{expr}.png", draw_face_canvas(expr, blink=False))
        save_rgba(f"face_{expr}_blink.png", draw_face_canvas(expr, blink=True))


def make_environment():
    """生成木地板、床品、窗外夜色等背景材质。"""
    w, h = 512, 512
    wood = np.zeros((h, w, 4), dtype=np.uint8)
    wood[..., 0] = 110
    wood[..., 1] = 75
    wood[..., 2] = 50
    wood[..., 3] = 255
    wood[::64, :, :3] = (70, 45, 30)
    save_rgba("floor_wood.png", wood)

    wall = np.full((h, w, 4), 235, dtype=np.uint8)
    wall[..., 0] = 230
    wall[..., 1] = 222
    wall[..., 2] = 210
    wall[..., 3] = 255
    save_rgba("wall_plaster.png", wall)

    for name, col in (
        ("linen_cream.png", (245, 238, 225, 255)),
        ("linen_sheet.png", (248, 244, 236, 255)),
        ("duvet_oatmeal.png", (232, 218, 198, 255)),
        ("rug_weave.png", (210, 195, 175, 255)),
        ("art_warm_abstract.png", (220, 160, 110, 255)),
        ("night_window.png", (15, 20, 35, 255)),
    ):
        arr = np.full((h, w, 4), col, dtype=np.uint8)
        save_rgba(name, arr)


def make_all():
    os.makedirs(TEX_DIR, exist_ok=True)
    make_lace_black()
    make_skin_warm()
    make_hair_dark()
    make_all_faces()
    make_environment()


if __name__ == "__main__":
    make_all()
    print("全部高品质材质纹理生成完毕！")