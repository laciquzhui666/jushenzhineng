#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""睡前守护机器人 —— PyBullet 温馨卧室小剧场
（机器人真实抓取夺机动作 + 人物躺下双手拉被子盖住上半身 + 30°人物主体俯拍视角 + 字幕防遮挡高置布局 + 国际标准1:1闹钟跳秒 + 机器人2秒说话间隔 + 主体1.5倍放大 + 助眠联动拉窗帘 + 详尽注释版）

===============================================================================
【作品核心优化说明】
1. 机器人抢手机真实“抓取（Grasp）”动作优化：
   - 构建双侧多指节仿生包络机械抓指（含近端指节与远端防滑软胶指尖），与夹爪开合自由度实时联动；
   - 将夺机过程细化为四步真实抓取运动学：
     (a) 伸臂预抓取与张爪对准（20.0s~21.40s）：机械臂伸至手机前上方预抓取悬停点，腕部旋转对准手机，双爪大幅张开至 0.068；
     (b) 精准下探包络机身（21.40s~21.95s）：保持双爪大张，沿轴向下探使左右指节深入包络在手机机身两侧；
     (c) 双爪收紧抓取锁死（21.95s~22.45s）：在 0.50 秒内双爪从 0.068 有力收拢至 0.008，远端指尖向内弯曲扣紧手机边框，女主角手指同步受牵引松开；
     (d) 拔起抽离与收纳（22.45s~25.35s）：牢牢抓持手机沿五阶贝塞尔弧线拔起并送入侧面储物抽屉。
2. 人物躺下睡觉时“双手将被子拉起盖住上半身”动作优化：
   - 在床品构建中设计“下段固定护腿被体 + 上段可拉起盖身羽绒被芯 + 上段蓬松褶皱被面 + 胸肩立体贴合被面 + 象牙白反折被沿”多层动态被子机构；
   - 当机器人说完“将为您播放助眠曲”（34.5s）后，女主角执行三步拉被入睡动作：
     (a) 伸手抓被沿（34.5s~35.25s）：双手向前下方探出，手指弯曲握紧腿部被头反折边；
     (b) 平躺同时拉起被子盖住上半身（35.25s~37.25s）：躯干平滑躺落枕芯，双手紧握被沿同步向上拉至肩颈锁骨处（y=1.79），将腰腹、胸口与双肩完整严密盖住，仅露出枕上的头部与安详睡颜；
     (c) 掖好被角放松入眠（37.25s~37.85s）：双手松开被沿并自然安放于被窝内侧，彻底进入梦乡。
3. 气泡字幕防遮挡优化（30° 俯拍视角下绝不遮挡机器人）：
   - 将气泡字幕锚点置于机器人后上方高处 `speech_pos = (-0.48, 2.22, 1.46)`，在 30° 俯拍投影中位于机器人与机械臂正上方净空区，全程零遮挡。
4. 闹钟时间跳动改为国际规范快慢（1:1 国际标准秒 SI Second）+ 30° 人物主体俯拍视角 + 机器人说话间隔 2 秒 + 助眠联动拉窗帘 + 主体 1.5 倍等比例放大。
===============================================================================
"""

from __future__ import annotations

import argparse      # 命令行参数解析模块（用于解析窗口分辨率、离线渲染模式等）
import math          # 基础数学函数库（三角函数、弧度转换、指数衰减等）
import os            # 操作系统路径与文件管理模块
import queue         # 线程安全队列（用于机器人语音台词串行排队播放）
import subprocess    # 子进程调用模块（用于调用系统 PowerShell / say / espeak 合成语音）
import sys           # 系统运行环境与平台判断模块
import threading     # 多线程并发模块（用于后台非阻塞语音播报守护线程）
import time          # 高精度计时与帧率控制模块
import wave          # WAV 音频文件读写模块（用于程序化生成八音盒助眠曲）

import numpy as np   # 高性能多维数组与矩阵运算库（用于三维向量、四元数与图像后处理）

# ---------------------------------------------------------------------------
# 外部依赖库与项目子模块导入（带容错降级处理）
# ---------------------------------------------------------------------------
# 尝试导入 PyBullet 物理引擎及官方数据包路径
try:
    import pybullet as p
    import pybullet_data
except ImportError:
    p = None
    pybullet_data = None

# 尝试导入 Windows 原生音频播放模块 winsound（用于播放 WAV 语音和循环播放助眠曲）
try:
    import winsound
except ImportError:
    winsound = None

# 尝试导入程序化三维网格生成模块（生成球体、圆角矩形面板、褶皱被面、面部曲面等 OBJ 文件）
try:
    import meshgen
except ImportError:
    meshgen = None

# 尝试导入程序化纹理贴图生成模块（首次运行时自动绘制木纹、布纹、人脸表情、电子钟数字等 PNG）
try:
    import textures
except ImportError:
    textures = None

# 尝试导入 PNG 图像保存工具；若不存在则回退使用 PIL (Pillow) 库保存渲染帧
try:
    from pngutil import write_png
except ImportError:
    def write_png(filename, img_array):
        """回退图像保存函数：将 NumPy RGB 数组保存为 PNG 文件。"""
        from PIL import Image
        Image.fromarray(img_array).save(filename)

# 尝试导入床头守护机器人机械臂模块（含逆运动学求解、夹爪开合与侧边储物抽屉控制）
try:
    from robot_rig import RobotArm
except ImportError:
    RobotArm = None

# 尝试导入女主角人体骨骼与表情绑定模块（含脊柱倾角控制、双臂逆运动学、手指弯曲与眨眼控制）
try:
    from woman_rig import Woman
except ImportError:
    Woman = None

# ---------------------------------------------------------------------------
# 全局资源目录路径与台词常量定义
# ---------------------------------------------------------------------------
# ---------------------------------------------------------------------------
# 【核心修复】解决 PyBullet C++ 底层 (b3ResourcePath / fopen) 在中文路径或 EXE 下
# 无法加载任何 PNG 纹理贴图（导致字幕、闹钟、地板、人脸全部变灰白纯色块）的问题！
# ---------------------------------------------------------------------------
import shutil
import tempfile

def _init_ascii_runtime_root() -> str:
    """定位项目原始资源目录，并在路径含中文字符（或处于 EXE 打包环境）时，
    自动同步到纯英文（ASCII）高速缓存目录，确保 PyBullet C++ 底层 100% 加载成功。"""
    if getattr(sys, "frozen", False):
        exe_dir = os.path.dirname(os.path.abspath(sys.executable))
        mei_dir = getattr(sys, "_MEIPASS", exe_dir)
        # 依次查找：_internal/assets -> exe同级/assets -> exe上级/assets -> exe上两级/assets
        candidates = [
            mei_dir,
            exe_dir,
            os.path.dirname(exe_dir),
            os.path.dirname(os.path.dirname(exe_dir)),
        ]
        src_root = mei_dir
        for cand in candidates:
            if cand and os.path.exists(os.path.join(cand, "assets")):
                src_root = cand
                break
    else:
        src_root = os.path.dirname(os.path.abspath(__file__))

    # 判断路径是否包含非 ASCII 字符（如“夺机追梦者”或中文用户名）
    def _is_pure_ascii(s: str) -> bool:
        try:
            s.encode("ascii")
            return True
        except UnicodeEncodeError:
            return False

    # 在 Windows 下优先选取纯英文根路径（如 C:/Users/Public/sleep_guardian_cache）
    if sys.platform.startswith("win") and (getattr(sys, "frozen", False) or not _is_pure_ascii(src_root)):
        ascii_candidates = [
            os.path.join(os.environ.get("PUBLIC", "C:\\Users\\Public"), "sleep_guardian_cache"),
            "C:\\Temp\\sleep_guardian_cache",
            os.path.join(tempfile.gettempdir(), "sleep_guardian_cache"),
        ]
        cache_root = ascii_candidates[0]
        for ac in ascii_candidates:
            if _is_pure_ascii(ac):
                try:
                    os.makedirs(ac, exist_ok=True)
                    cache_root = ac
                    break
                except Exception:
                    continue
        # 将原始目录中的 assets 资源完整增量同步到纯英文缓存目录
        src_assets = os.path.join(src_root, "assets")
        dst_assets = os.path.join(cache_root, "assets")
        if os.path.exists(src_assets) and os.path.abspath(src_assets) != os.path.abspath(dst_assets):
            try:
                shutil.copytree(src_assets, dst_assets, dirs_exist_ok=True)
            except Exception as e:
                print("同步 assets 到纯英文缓存目录提示:", e)
        active_root = cache_root
    else:
        active_root = src_root

    os.makedirs(os.path.join(active_root, "assets"), exist_ok=True)
    try:
        os.chdir(active_root)
    except Exception:
        pass

    # 同步重定向所有子模块的 __file__ 与内部路径常量（ROOT / ASSETS / *_DIR），
    # 确保 textures.make_all() 和 meshgen 将文件直接生成在 active_root/assets 下！
    for mod_name in ("textures", "meshgen", "pngutil", "robot_rig", "woman_rig"):
        mod = sys.modules.get(mod_name)
        if mod is not None:
            try:
                mod.__file__ = os.path.join(active_root, f"{mod_name}.py")
                if hasattr(mod, "ROOT"):
                    mod.ROOT = active_root
                if hasattr(mod, "ASSETS"):
                    mod.ASSETS = os.path.join(active_root, "assets")
                if hasattr(mod, "TEX_DIR"):
                    mod.TEX_DIR = os.path.join(active_root, "assets", "textures")
                if hasattr(mod, "SPEECH_DIR"):
                    mod.SPEECH_DIR = os.path.join(active_root, "assets", "speech")
                if hasattr(mod, "CLOCK_DIR"):
                    mod.CLOCK_DIR = os.path.join(active_root, "assets", "clock")
                if hasattr(mod, "PHONE_DIR"):
                    mod.PHONE_DIR = os.path.join(active_root, "assets", "phone")
                if hasattr(mod, "MESH_DIR"):
                    mod.MESH_DIR = os.path.join(active_root, "assets", "meshes")
            except Exception:
                pass
    return active_root

ROOT = _init_ascii_runtime_root()                   # 纯英文安全运行根目录
ASSETS = os.path.join(ROOT, "assets")               # 静态与生成资源总目录
TEX_DIR = os.path.join(ASSETS, "textures")          # 材质贴图（木地板、墙面、床品、表情等）目录
SPEECH_DIR = os.path.join(ASSETS, "speech")         # 机器人气泡字幕贴图目录
CLOCK_DIR = os.path.join(ASSETS, "clock")           # 智能电子钟每秒数字贴图目录
PHONE_DIR = os.path.join(ASSETS, "phone")           # 智能手机屏幕贴图目录
VOICE_DIR = os.path.join(ASSETS, "voice")           # 机器人 TTS 合成语音 WAV 缓存目录

# 机器人四句字幕编号与对应中文语音台词文本映射表
SPEECH_LINES = {
    1: "不早了",
    2: "该睡觉了",
    3: "手机将于次日早上6点整归还",
    4: "将为您播放助眠曲",
}

# ---------------------------------------------------------------------------
# 机器人完整语音合成与不打断排队播放引擎
# ---------------------------------------------------------------------------
# 线程安全语音任务队列：存放待播报的台词编号（1~4）
_SPEECH_QUEUE: "queue.Queue[int | None]" = queue.Queue()
# 语音繁忙状态事件标志：当队列中有语音正在播报时置为 True，防止助眠曲提前抢播打断人声
_SPEECH_BUSY = threading.Event()


def ensure_speech_wavs():
    """首次运行时预生成全部四句台词的本地 WAV 音频文件，确保播放零延迟且发音完整。"""
    os.makedirs(VOICE_DIR, exist_ok=True)
    for idx, text in SPEECH_LINES.items():
        wav_path = os.path.join(VOICE_DIR, f"speech_{idx}.wav")
        # 若对应台词的 WAV 文件已存在且文件大小正常（>1KB），则跳过重复生成
        if os.path.exists(wav_path) and os.path.getsize(wav_path) > 1024:
            continue
        # 在 Windows 平台下调用内置 System.Speech.Synthesis 引擎将台词合成为本地 WAV 文件
        if sys.platform.startswith("win"):
            safe_path = wav_path.replace("'", "''")
            ps_script = (
                "$ErrorActionPreference = 'Stop'; "
                "Add-Type -AssemblyName System.Speech; "
                "$synth = New-Object System.Speech.Synthesis.SpeechSynthesizer; "
                "$synth.Rate = 0; "
                "$synth.Volume = 100; "
                f"$synth.SetOutputToWaveFile('{safe_path}'); "
                f"$synth.Speak('{text}'); "
                "$synth.Dispose();"
            )
            try:
                subprocess.run(
                    ["powershell", "-NoProfile", "-ExecutionPolicy", "Bypass", "-Command", ps_script],
                    check=True,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
                )
            except Exception as e:
                print(f"生成语音文件提示 ({text}): {e}")


def _speech_worker_loop():
    """后台串行语音播报守护线程：阻塞式依次播放队列中的每句台词，确保每句话 100% 完整读完、绝不中途截断。"""
    while True:
        # 从队列阻塞获取下一句待播报的台词编号
        idx = _SPEECH_QUEUE.get()
        if idx is None:
            _SPEECH_QUEUE.task_done()
            continue
        # 标记语音引擎处于繁忙播报状态
        _SPEECH_BUSY.set()
        try:
            text = SPEECH_LINES.get(idx, "")
            wav_path = os.path.join(VOICE_DIR, f"speech_{idx}.wav")
            # 优先使用 winsound 同步播放预合成的高清 WAV 文件（不带 SND_ASYNC，保证整句播完才进入下一句）
            if winsound is not None and os.path.exists(wav_path) and os.path.getsize(wav_path) > 1024:
                winsound.PlaySound(wav_path, winsound.SND_FILENAME)
            # 若 WAV 不存在，则在 Windows 下实时调用 PowerShell TTS 播报
            elif sys.platform.startswith("win"):
                ps_cmd = (
                    "Add-Type -AssemblyName System.Speech; "
                    "$s = New-Object System.Speech.Synthesis.SpeechSynthesizer; "
                    "$s.Rate = 0; "
                    f"$s.Speak('{text}');"
                )
                subprocess.run(
                    ["powershell", "-NoProfile", "-ExecutionPolicy", "Bypass", "-Command", ps_cmd],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
                )
            # macOS 平台回退使用系统 `say` 命令播报
            elif sys.platform == "darwin":
                subprocess.run(["say", text])
            # Linux 平台回退使用 `espeak` 中文语音播报
            else:
                subprocess.run(["espeak", "-v", "zh", text])
        except Exception as exc:
            print("语音播报异常:", exc)
        finally:
            # 若队列已空，清除繁忙标志，允许后续助眠曲安全切入
            if _SPEECH_QUEUE.empty():
                _SPEECH_BUSY.clear()
            _SPEECH_QUEUE.task_done()


# 创建并启动后台常驻语音播报守护线程（随主程序退出自动结束）
_WORKER_THREAD = threading.Thread(target=_speech_worker_loop, daemon=True)
_WORKER_THREAD.start()


def speak_subtitle(idx: int, enabled: bool = True):
    """将指定编号（1~4）的字幕台词推入语音队列，由后台线程排队完整朗读。"""
    if not enabled or idx not in SPEECH_LINES:
        return
    _SPEECH_BUSY.set()
    _SPEECH_QUEUE.put(idx)


def clear_speech_queue():
    """清空当前尚未播报的所有排队语音（在按 R 键重置剧目时调用）。"""
    while not _SPEECH_QUEUE.empty():
        try:
            _SPEECH_QUEUE.get_nowait()
            _SPEECH_QUEUE.task_done()
        except queue.Empty:
            break
    _SPEECH_BUSY.clear()


# ---------------------------------------------------------------------------
# 高阶三维数学、平滑样条曲线与四元数姿态插值工具函数
# ---------------------------------------------------------------------------
def v3(x, y=None, z=None):
    """快速构造三维浮点 NumPy 向量：支持传入单个序列 v3([x,y,z]) 或三个标量 v3(x, y, z)。"""
    if y is None:
        return np.asarray(x, dtype=float)
    return np.array([x, y, z], dtype=float)


def norm(v):
    """计算三维向量的单位归一化向量；若模长接近 0 则直接返回原向量以防止除零错误。"""
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    return v / n if n > 1e-9 else v


def lerp(a, b, t):
    """线性插值函数：根据权重 t ∈ [0, 1] 在向量/标量 a 与 b 之间按比例过渡。"""
    return np.asarray(a, dtype=float) * (1.0 - t) + np.asarray(b, dtype=float) * t


def smoothstep(t):
    """三阶 Hermite 平滑插值函数：3t^2 - 2t^3，使起止端速度平滑归零。"""
    t = np.clip(t, 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def ease_in_out(t):
    """五阶缓入缓出平滑函数：6t^5 - 15t^4 + 10t^3。"""
    t = np.clip(t, 0.0, 1.0)
    return t * t * t * (t * (t * 6 - 15) + 10)


def quintic_ease(u):
    """Minimum-Jerk（最小加加速度）五次多项式插值：
    公式：s(u) = 6u^5 - 15u^4 + 10u^3
    特性：一阶导数（速度）和二阶导数（加速度）在 u=0 与 u=1 处均严格为 0，
    用于机械臂抓取、人物拉被平躺、灯光渐暗与窗帘拉拢，彻底消除启停顿挫感。
    """
    u = np.clip(u, 0.0, 1.0)
    return u * u * u * (u * (u * 6.0 - 15.0) + 10.0)


def bezier5(p0, p1, p2, p3, p4, u):
    """五阶（4次）贝塞尔空间曲线插值：
    由 5 个空间控制点 p0~p4 生成高阶连续曲线，用于机械臂夹取手机后一气呵成地
    完成“拔起 -> 跨越空中高弧 -> 对准侧面储物格”的全程无停顿丝滑轨迹。
    """
    u = np.clip(u, 0.0, 1.0)
    inv = 1.0 - u
    return (inv**4 * np.asarray(p0) +
            4.0 * inv**3 * u * np.asarray(p1) +
            6.0 * inv**2 * u**2 * np.asarray(p2) +
            4.0 * inv * u**3 * np.asarray(p3) +
            u**4 * np.asarray(p4))


def bezier_curve(p0, p1, p2, p3, u):
    """三次贝塞尔空间曲线插值：
    由起点 p0、两个中间控制点 p1/p2 与终点 p3 构造平滑三维弧线轨迹。
    """
    u = np.clip(u, 0.0, 1.0)
    inv = 1.0 - u
    return (inv**3 * np.asarray(p0) +
            3.0 * inv**2 * u * np.asarray(p1) +
            3.0 * inv * u**2 * np.asarray(p2) +
            u**3 * np.asarray(p3))


def slerp_vector(v0, v1, t):
    """三维单位向量球面线性插值（SLERP）：
    使机械臂末端夹爪朝向向量在两个空间方向之间以恒定角速度平滑旋转，避免欧拉角万向节死锁或突变。
    """
    v0 = norm(v0)
    v1 = norm(v1)
    dot = float(np.clip(np.dot(v0, v1), -1.0, 1.0))
    # 若两向量夹角极小，退化为线性插值并归一化以避免除以 sin(0)
    if dot > 0.9995:
        return norm(lerp(v0, v1, t))
    theta = math.acos(dot)
    sin_theta = math.sin(theta)
    return norm((math.sin((1.0 - t) * theta) * v0 + math.sin(t * theta) * v1) / sin_theta)


def quat_from_matrix(R):
    """Shepperd 算法：将 3x3 正交旋转矩阵 R 稳定转换为 PyBullet 标准四元数 [x, y, z, w]。"""
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
    """基于 Gram-Schmidt 正交化构造局部坐标系四元数：
    使局部 +Z 轴严格对齐 z_axis，局部 +Y 轴尽可能对齐参考方向 y_hint。
    """
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
    """根据目标轴向 direction 构造旋转四元数（使圆柱体/胶囊体的局部 Z 轴指向 direction）。"""
    z = norm(direction)
    up = np.asarray(up_hint, dtype=float)
    # 若目标轴向与默认上方向近似共线，则切换备用参考轴防止叉积退化
    if abs(np.dot(z, norm(up))) > 0.96:
        up = np.array([1.0, 0.0, 0.0])
    x = norm(np.cross(up, z))
    y = np.cross(z, x)
    return quat_from_matrix(np.column_stack([x, y, z]))


def look_at_quat(position, camera, up_hint=(0.0, 0.0, 1.0)):
    """计算使位于 position 的面板正面（+Z 法线）始终朝向摄像机视点 camera 的四元数。"""
    return quat_from_z(np.asarray(camera) - np.asarray(position), up_hint)


def matrix_from_quat(q):
    """将四元数 [x, y, z, w] 转换为 3x3 旋转矩阵。"""
    if p is not None:
        return np.array(p.getMatrixFromQuaternion(q), dtype=float).reshape(3, 3)
    x, y, z, w = q
    return np.array([
        [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
        [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
    ])


def interp_quat(q0, q1, t):
    """四元数最短路径归一化线性插值（NLERP）：用于手机旋转姿态的平滑过渡。"""
    a = np.asarray(q0, dtype=float)
    b = np.asarray(q1, dtype=float)
    # 若点积为负，翻转其中一个四元数以保证走球面最短弧
    if np.dot(a, b) < 0.0:
        b = -b
    q = a * (1.0 - t) + b * t
    n = np.linalg.norm(q)
    return list(q / n) if n > 1e-9 else list(a)


def camera_eye_from_view(view_matrix):
    """从 PyBullet 返回的 4x4 列主序视图矩阵（View Matrix）反解摄像机在世界坐标系下的位置向量。"""
    M = np.array(view_matrix, dtype=float).reshape(4, 4).T
    R = M[:3, :3]
    t = M[:3, 3]
    return -R.T @ t


# ---------------------------------------------------------------------------
# 电影感光影后期处理管线（主体清晰 + 边缘柔焦 + 冷暖色调 + 睡眠暗调）
# ---------------------------------------------------------------------------
def apply_cinematic_postprocess(rgb_float: np.ndarray, sleep_dim: float = 0.0) -> np.ndarray:
    """对离线截取的渲染图像施加电影级后期调色与浅景深处理。

    参数：
      rgb_float: 原始 RGB 图像数组 (H, W, 3)
      sleep_dim: 睡眠暗光系数 (0.0=常亮模式, 1.0=助眠暗夜模式)
    """
    h, w, _ = rgb_float.shape
    img = rgb_float.astype(np.float32) / 255.0

    # 1. 构造椭圆径向景深遮罩：以画面中心偏右的主体区域 (0.55w, 0.46h) 为清晰焦点，边缘柔化
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
    fx, fy = w * 0.55, h * 0.46
    dist = np.sqrt(((xx - fx) / (w * 0.52)) ** 2 + ((yy - fy) / (h * 0.50)) ** 2)
    blur_mask = np.clip((dist - 0.68) / 0.55, 0.0, 1.0)[..., None]

    # 通过 4 倍下采样再上采样实现快速背景虚化，并按遮罩权重与原图融合
    down = img[::4, ::4, :]
    up = np.repeat(np.repeat(down, 4, axis=0), 4, axis=1)[:h, :w, :]
    img = img * (1.0 - blur_mask * 0.45) + up * (blur_mask * 0.45)

    # 2. 冷暖互补色调映射：根据像素亮度 luma，暗部偏温馨琥珀暖光，高光偏清透冷白
    luma = 0.299 * img[..., 0] + 0.587 * img[..., 1] + 0.114 * img[..., 2]
    warm_ambient = np.array([1.03, 0.98, 0.91], dtype=np.float32)
    clean_highlight = np.array([0.97, 1.00, 1.05], dtype=np.float32)
    img = img * (warm_ambient * (1.0 - luma[..., None] * 0.30) + clean_highlight * (luma[..., None] * 0.30))

    # 3. 睡眠模式暗调滤镜：随 sleep_dim 压暗整体曝光并注入静谧暖夜色调
    if sleep_dim > 0.0:
        darken = 1.0 - sleep_dim * 0.48
        night_tint = np.array([1.02, 0.96, 0.90], dtype=np.float32)
        img = img * (darken * night_tint)

    # 4. S 型曲线对比度柔化与微弱胶片颗粒注入（消除暗部色彩断层）
    img = np.clip(img, 0.0, 1.0)
    img = img * img * (3.0 - 2.0 * img) * 0.18 + img * 0.82
    rng = np.random.default_rng(42)
    grain = rng.normal(0.0, 0.008, size=img.shape).astype(np.float32)
    img = np.clip(img + grain, 0.0, 1.0)

    return (img * 255.0).astype(np.uint8)


# ---------------------------------------------------------------------------
# 程序化助眠音乐（八音盒摇篮曲）合成与播放控制
# ---------------------------------------------------------------------------
def _midi_hz(note):
    """将 MIDI 音高编号转换为标准十二平均律频率（A4 = 69 -> 440.0 Hz）。"""
    return 440.0 * (2.0 ** ((note - 69) / 12.0))


def make_lullaby(path):
    """程序化合成八音盒风格助眠摇篮曲并保存为 16-bit 单声道 WAV 文件。"""
    sr = 22050  # 采样率 22050 Hz
    # 主旋律音符序列：(MIDI音高, 持续时长秒数)
    melody = [
        (67, 0.55), (67, 0.55), (64, 0.55), (64, 0.55),
        (67, 0.55), (67, 0.55), (64, 1.10),
        (62, 0.55), (64, 0.55), (65, 0.55), (62, 0.55),
        (60, 0.55), (60, 0.55), (59, 1.10),
        (62, 0.55), (64, 0.55), (65, 0.55), (62, 0.55),
        (60, 0.55), (64, 0.55), (62, 1.10),
        (67, 0.55), (67, 0.55), (64, 0.55), (64, 0.55),
        (67, 0.55), (67, 0.55), (64, 1.10),
        (62, 0.55), (64, 0.55), (65, 0.55), (62, 0.55),
        (60, 1.65),
    ]
    total = sum(d for _, d in melody) + 3.0
    n = int(total * sr)
    out = np.zeros(n, dtype=np.float32)

    # 1. 合成八音盒清脆主旋律（基频 + 2/3/4倍微失谐泛音 + 快速起音与指数余音包络）
    t0 = 0.0
    for note, dur in melody:
        start = int(t0 * sr)
        length = int((dur + 1.4) * sr)
        t = np.arange(length, dtype=np.float32) / sr
        f = _midi_hz(note)
        env = (1.0 - np.exp(-t / 0.012)) * np.exp(-t / (dur * 0.95 + 0.45))
        tone = (np.sin(2 * np.pi * f * t)
                + 0.24 * np.sin(2 * np.pi * f * 2.005 * t)
                + 0.08 * np.sin(2 * np.pi * f * 3.01 * t)
                + 0.035 * np.sin(2 * np.pi * f * 4.02 * t))
        end = min(n, start + length)
        out[start:end] += (tone * env * 0.22)[:end - start]
        t0 += dur

    # 2. 合成温暖柔和的低音和弦铺底（C - G - Am - F 五度/八度叠加）
    chords = [(48, 0.0, 7.0), (43, 7.0, 7.0), (57, 14.0, 6.0), (53, 20.0, 6.0)]
    for root, start_s, dur in chords:
        start = int(start_s * sr)
        length = int((dur + 1.0) * sr)
        t = np.arange(length, dtype=np.float32) / sr
        env = (1.0 - np.exp(-t / 0.8)) * np.exp(-np.maximum(0, t - dur) / 0.6)
        pad = np.zeros(length, dtype=np.float32)
        for iv in (0, 7, 12):
            f = _midi_hz(root + iv) * 0.5
            pad += np.sin(2 * np.pi * f * t + iv * 0.4)
        pad *= 0.011 * env
        end = min(n, start + length)
        out[start:end] += pad[:end - start]

    # 3. 施加双抽头延迟混响（0.17s 与 0.34s 回声）及首尾 1.2s 平滑淡入淡出
    delay1, delay2 = int(0.17 * sr), int(0.34 * sr)
    wet = out.copy()
    wet[delay1:] += out[:-delay1] * 0.17
    wet[delay2:] += out[:-delay2] * 0.08
    out = wet
    fade = int(1.2 * sr)
    out[:fade] *= np.linspace(0, 1, fade, dtype=np.float32)
    out[-fade:] *= np.linspace(1, 0, fade, dtype=np.float32)

    # 4. 峰值归一化并量化为 16-bit 整型写入 WAV 文件
    peak = float(np.max(np.abs(out))) or 1.0
    out = out / peak * 0.58
    data = (out * 32767.0).astype(np.int16)
    with wave.open(path, "wb") as f:
        f.setnchannels(1)
        f.setsampwidth(2)
        f.setframerate(sr)
        f.writeframes(data.tobytes())
    return path


def play_lullaby(path):
    """调用 winsound 以异步循环模式（SND_ASYNC | SND_LOOP）播放助眠曲。"""
    if winsound is None:
        return
    try:
        winsound.PlaySound(
            path, winsound.SND_FILENAME | winsound.SND_ASYNC | winsound.SND_LOOP
        )
    except Exception as exc:
        print("音频播放失败:", exc)


def stop_lullaby():
    """停止当前正在播放的助眠曲音频。"""
    if winsound is None:
        return
    try:
        winsound.PlaySound(None, winsound.SND_PURGE)
    except Exception:
        pass


# ---------------------------------------------------------------------------
# Scene 场景构建与全剧动画时间线控制核心类
# ---------------------------------------------------------------------------
class Scene:
    """温馨卧室小剧场总控类：负责构建三维房间、家具、人物、机器人，并按时间轴调度动画与视听效果。"""

    # ==================== 【核心优化：人物主体 30° 俯拍视角配置】 ====================
    # CAM_TARGET 精准锁定在人物主体与床头交互区中心 (0.02, 1.42, 1.18)；
    # CAM_PITCH 设为 -30.0°（标准 30 度向下俯拍视角），摄像机高度约 2.36m，彻底摒弃原先的仰拍/平视视角！
    CAM_TARGET = (0.02, 1.42, 1.18)
    CAM_DIST = 2.35
    CAM_YAW = -26.0
    CAM_PITCH = -30.0
    CAM_FOV = 37.0

    # ==================== 【核心优化：主体部分 1.5 倍等比例缩放参数】 ====================
    # 将场景核心主体部分（女主角 + 守护机器人 + 电子钟 + 床铺床品 + 床头柜 + 台灯 + 气泡字幕）整体放大 1.5 倍
    SUBJECT_SCALE = 1.5
    # 三维相似变换地面基准锚点：z=0.0 保证放大 1.5 倍后床腿、床头柜腿、拖鞋依然严密贴合地面不悬空；
    # y=2.38 保证床头板依然贴近后墙不穿模；x=0.32 保证床体与右侧窗户窗帘之间留有自然通道。
    SUBJECT_SCALE_ORIGIN = v3(0.32, 2.38, 0.0)

    def __init__(self, gui=True, width=1920, height=1080, quiet=False,
                audio=True):
        """初始化 PyBullet 物理仿真器、加载材质贴图、构建全屋场景并启动剧目状态机。"""
        self.width = width
        self.height = height
        self.quiet = quiet
        self.audio_enabled = bool(audio)
        # 若开启音频，首次启动时确保四句台词 WAV 已合成就绪
        if self.audio_enabled:
            ensure_speech_wavs()

        # 连接 PyBullet 引擎（GUI 窗口模式或 DIRECT 后台无头渲染模式）
        opts = "--width=%d --height=%d" % (width, height)
        self.client = p.connect(p.GUI if gui else p.DIRECT, options=opts)
        if pybullet_data is not None:
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
        # 将纯英文资源根目录加入 PyBullet 内部搜索路径
        p.setAdditionalSearchPath(ROOT.replace("\\", "/"))
        p.setGravity(0, 0, -9.81)                         # 设置标准重力加速度
        p.setTimeStep(1.0 / 60.0)                         # 设置物理仿真步长为 60 FPS
        p.setPhysicsEngineParameter(numSolverIterations=20,
                                    numSubSteps=1,
                                    enableConeFriction=False)
        # 配置 OpenGL 可视化窗口：隐藏两侧调试控件，开启 4096 高清软阴影
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)
        p.configureDebugVisualizer(
            lightPosition=[-0.82, 1.68, 2.15],
            shadowMapResolution=4096,
            shadowMapIntensity=0.32,
        )
        # 初始化摄像机为人物主体 30° 俯拍视角
        p.resetDebugVisualizerCamera(self.CAM_DIST, self.CAM_YAW,
                                    self.CAM_PITCH, self.CAM_TARGET)
        if not quiet:
            print("PyBullet 卧室小剧场启动中（30°人物主体俯拍视角）...")

        # 场景资源与动态对象容器初始化
        self.meshes = {}                  # 存放程序化生成的 OBJ 网格文件路径
        self.tex = self._load_textures()  # 加载所有 PNG 纹理贴图并返回贴图 ID 字典
        self.bodies = {}                  # 存放具名关键刚体 ID（如电子钟外壳与屏幕）
        self.body_tex_map = {}            # 記錄每個剛體當前綁定的貼圖 ID，防止 changeVisualShape 調光時丟失貼圖
        self.unit_sphere = None           # 单位球体网格路径（用于缩放生成任意三轴椭球）
        self.unit_quad = None             # 单位平面网格路径（用于生成贴图卡片）
        self.scene_ambient_bodies = []    # 存放全屋参与睡眠调光的视觉刚体及初始材质颜色
        self.sleep_dimmable_items = []    # 存放额外调光对象记录
        self.nightlamp_bulb = None        # 床头台灯灯泡刚体 ID
        self.nightlamp_shade = None       # 床头台灯百褶灯罩刚体 ID
        self.bed_string_lights = []       # 床头 15 盏星光串灯刚体 ID 列表
        self.curtain_pleats = []          # 右侧窗户动态双开窗帘褶皱与滑环列表
        self.pull_duvet_parts = []        # 存放人物躺下时可向上拉起盖住上半身的动态被子组件列表
        self.gripper_claws = []           # 存放机器人末端双侧可开合弯曲的仿生抓取手指组件列表
        self.current_dim_factor = 0.0     # 当前全屋灯光暗化程度（0.0=常亮, 1.0=睡眠暗光）
        self.current_curtain_close = 0.0  # 当前窗帘拉合程度（0.0=敞开, 1.0=完全拉拢）
        self.current_duvet_cover = 0.0    # 当前被子拉起盖住上半身程度（0.0=腿部初始位, 1.0=拉起盖至肩颈）

        # 主体部分 1.5 倍等比例缩放控制状态与刚体集合
        self._subject_scaling_active = False
        self.scaled_subject_bodies = set()
        self._install_subject_scale_hooks()

        # 1. 生成基础几何网格并构建外围房间建筑、窗户、动态窗帘与背景陈设（保持房间原始尺寸）
        self._ensure_meshes()
        self._build_room()
        self._build_lounge_chair()
        self._build_indoor_plant()
        self._build_wall_decor()

        # 2. 开启【主体部分 1.5 倍等比例放大模式】，构建床铺、床头柜、灯光、闹钟、机器人、女主角与字幕
        self._subject_scaling_active = True
        try:
            self._build_bed()
            self._build_nightstand()
            self._build_nightstand_accessories()
            self._build_bedroom_slippers()
            self._build_bed_throw()
            self._build_warm_nightlight()
            self._build_clock()
            self._build_robot()
            self._build_woman()
            self._build_phone_face_glow()
            self._build_speech()
        finally:
            self._subject_scaling_active = False

        # 初始化剧情时间轴状态并重置到 0.0s 开场状态
        self.story_t = 0.0
        self.story_paused = False
        self.lullaby_on = False
        self.lullaby_path = os.path.join(ASSETS, "lullaby.wav")
        self.reset_story()
        self.update_camera_billboards()

    # -----------------------------------------------------------------------
    # 【核心优化实现】底层 PyBullet 三维相似变换拦截钩子（主体放大 1.5 倍）
    # -----------------------------------------------------------------------
    def _scale_subject_pos(self, pos):
        """将原始 1.0 倍主体空间坐标通过 1.5 倍三维相似变换映射到世界坐标：
        公式：P' = P_origin + SUBJECT_SCALE * (P - P_origin)
        """
        pos_arr = np.asarray(pos, dtype=float)
        # 若 z 坐标远低于地面（如 z = -100 用于隐藏字幕或手机屏幕辉光），则保持隐藏位置不变
        if pos_arr.shape == (3,) and pos_arr[2] < -10.0:
            return pos_arr
        return self.SUBJECT_SCALE_ORIGIN + self.SUBJECT_SCALE * (pos_arr - self.SUBJECT_SCALE_ORIGIN)

    def _install_subject_scale_hooks(self):
        """在 PyBullet 模块层包装 createVisualShape、createMultiBody 与 resetBasePositionAndOrientation：
        确保主体部分（包括 woman_rig.py 和 robot_rig.py 内部创建的每一个部件）在构建和运动更新时，
        几何尺寸与空间坐标均自动、单次、无遗漏地放大 1.5 倍。
        """
        if p is None:
            return
        # 保存 PyBullet 原始底层函数指针（避免重复包装）
        if not hasattr(p, "_orig_createVisualShape"):
            p._orig_createVisualShape = p.createVisualShape
            p._orig_createMultiBody = p.createMultiBody
            p._orig_resetBasePositionAndOrientation = p.resetBasePositionAndOrientation
            p._orig_changeVisualShape = p.changeVisualShape

        scene_ref = self
        s = float(self.SUBJECT_SCALE)

        def hooked_createVisualShape(shapeType, *args, **kwargs):
            """创建视觉几何体钩子：当处于主体构建阶段时，自动将网格、长方体、球体、圆柱、胶囊体尺寸放大 1.5 倍，并将网格路径转为纯 ASCII 相对路径以兼容中文目录。"""
            if shapeType == p.GEOM_MESH and "fileName" in kwargs and kwargs["fileName"]:
                kwargs["fileName"] = os.path.abspath(kwargs["fileName"]).replace("\\", "/")
            if scene_ref._subject_scaling_active:
                if shapeType == p.GEOM_MESH and "meshScale" in kwargs:
                    kwargs["meshScale"] = [float(v) * s for v in kwargs["meshScale"]]
                elif shapeType == p.GEOM_BOX and "halfExtents" in kwargs:
                    kwargs["halfExtents"] = [float(v) * s for v in kwargs["halfExtents"]]
                elif shapeType in (p.GEOM_SPHERE, p.GEOM_CYLINDER, p.GEOM_CAPSULE):
                    if "radius" in kwargs:
                        kwargs["radius"] = float(kwargs["radius"]) * s
                    if "length" in kwargs:
                        kwargs["length"] = float(kwargs["length"]) * s
            return p._orig_createVisualShape(shapeType, *args, **kwargs)

        def hooked_createMultiBody(baseMass=0, baseCollisionShapeIndex=-1,
                                   baseVisualShapeIndex=-1, basePosition=(0, 0, 0),
                                   baseOrientation=(0, 0, 0, 1), *args, **kwargs):
            """创建多体对象钩子：当处于主体构建阶段时，自动映射 1.5 倍初始位置并登记刚体 ID。"""
            if scene_ref._subject_scaling_active:
                basePosition = list(scene_ref._scale_subject_pos(basePosition))
            body_id = p._orig_createMultiBody(
                baseMass, baseCollisionShapeIndex, baseVisualShapeIndex,
                basePosition, baseOrientation, *args, **kwargs
            )
            if scene_ref._subject_scaling_active and body_id is not None and body_id != -1:
                scene_ref.scaled_subject_bodies.add(body_id)
            return body_id

        def hooked_resetBasePositionAndOrientation(bodyUniqueId, posObj, ornObj, *args, **kwargs):
            """更新刚体位姿钩子：凡属于主体集合的刚体，在每帧动画更新时自动将其坐标映射至 1.5 倍空间。"""
            if bodyUniqueId in scene_ref.scaled_subject_bodies:
                posObj = list(scene_ref._scale_subject_pos(posObj))
            return p._orig_resetBasePositionAndOrientation(bodyUniqueId, posObj, ornObj, *args, **kwargs)

        def hooked_changeVisualShape(objectUniqueId, linkIndex, *args, **kwargs):
            """修改视觉属性钩子：PyBullet 底层 C++ 在调用 changeVisualShape 修改颜色时，若未显式传入 textureUniqueId，
            会默认将 textureUniqueId 重置为 -1（导致闹钟、地板、床单等贴图瞬间丢失变灰白）！
            此钩子自动记忆并补齐该刚体已绑定的 textureUniqueId。"""
            key = (objectUniqueId, linkIndex)
            if "textureUniqueId" in kwargs and kwargs["textureUniqueId"] is not None:
                scene_ref.body_tex_map[key] = kwargs["textureUniqueId"]
            elif key in scene_ref.body_tex_map and scene_ref.body_tex_map[key] is not None:
                kwargs["textureUniqueId"] = scene_ref.body_tex_map[key]
            return p._orig_changeVisualShape(objectUniqueId, linkIndex, *args, **kwargs)

        # 挂载钩子函数
        p.createVisualShape = hooked_createVisualShape
        p.createMultiBody = hooked_createMultiBody
        p.resetBasePositionAndOrientation = hooked_resetBasePositionAndOrientation
        p.changeVisualShape = hooked_changeVisualShape

    # -----------------------------------------------------------------------
    # 贴图加载与基础网格生成
    # -----------------------------------------------------------------------
    def _ensure_speech_and_clock_pngs(self):
        """内置全量贴图兜底生成器：当 exe 环境中缺失字幕、闹钟、手机屏幕或基础贴图时，使用 PIL 实时绘制。"""
        try:
            from PIL import Image, ImageDraw, ImageFont
        except ImportError:
            return

        os.makedirs(TEX_DIR, exist_ok=True)
        os.makedirs(SPEECH_DIR, exist_ok=True)
        os.makedirs(CLOCK_DIR, exist_ok=True)
        os.makedirs(PHONE_DIR, exist_ok=True)

        # 尝试加载 Windows 中文字体（微软雅黑 / 黑体）与等宽数字字体
        def _get_font(size, bold=False):
            candidates = [
                "C:/Windows/Fonts/msyhbd.ttc" if bold else "C:/Windows/Fonts/msyh.ttc",
                "C:/Windows/Fonts/simhei.ttf",
                "C:/Windows/Fonts/simsun.ttc",
                "msyh.ttc",
                "simhei.ttf",
            ]
            for fp in candidates:
                if os.path.exists(fp):
                    try:
                        return ImageFont.truetype(fp, size)
                    except Exception:
                        pass
            return ImageFont.load_default()

        # 1. 检查并生成 4 张机器人气泡字幕贴图 (speech_plate_1.png ~ speech_plate_4.png)
        for idx, text in SPEECH_LINES.items():
            spath = os.path.join(SPEECH_DIR, f"speech_plate_{idx}.png")
            if not os.path.exists(spath):
                W, H = 1024, 256
                img = Image.new("RGBA", (W, H), (252, 248, 240, 255))
                draw = ImageDraw.Draw(img)
                # 绘制温润圆角边框
                draw.rounded_rectangle([10, 10, W - 11, H - 11], radius=36,
                                       fill=(254, 250, 242, 255), outline=(210, 185, 150, 255), width=6)
                font_sz = 64 if len(text) > 8 else 82
                font = _get_font(font_sz, bold=True)
                bbox = draw.textbbox((0, 0), text, font=font)
                tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
                tx = (W - tw) // 2 - bbox[0]
                ty = (H - th) // 2 - bbox[1]
                draw.text((tx, ty), text, font=font, fill=(45, 38, 32, 255))
                img.convert("RGB").save(spath)

        # 2. 检查并生成 21 张电子闹钟数字贴图 (11:59:40 ~ 12:00:00)
        clock_times = [(f"clock_11_59_{s:02d}.png", f"11:59:{s:02d}") for s in range(40, 60)]
        clock_times.append(("clock_12_00_00.png", "12:00:00"))
        font_clock = _get_font(128, bold=True)
        for fname, t_str in clock_times:
            cpath = os.path.join(CLOCK_DIR, fname)
            if not os.path.exists(cpath):
                W, H = 640, 320
                img = Image.new("RGB", (W, H), (14, 16, 22))
                draw = ImageDraw.Draw(img)
                draw.rounded_rectangle([8, 8, W - 9, H - 9], radius=24,
                                       fill=(18, 20, 28), outline=(45, 52, 68), width=4)
                bbox = draw.textbbox((0, 0), t_str, font=font_clock)
                tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
                tx = (W - tw) // 2 - bbox[0]
                ty = (H - th) // 2 - bbox[1]
                # 12:00:00 零点时刻高亮为柔和暖橙色，其余时刻为护眼暖琥珀白
                col = (255, 195, 110) if t_str == "12:00:00" else (245, 228, 190)
                draw.text((tx, ty), t_str, font=font_clock, fill=col)
                img.save(cpath)

        # 3. 检查并生成手机屏幕贴图 (phone_screen.png)
        ppath = os.path.join(PHONE_DIR, "phone_screen.png")
        if not os.path.exists(ppath):
            img = Image.new("RGB", (320, 640), (24, 30, 46))
            draw = ImageDraw.Draw(img)
            draw.rounded_rectangle([16, 28, 304, 210], radius=18, fill=(52, 78, 122))
            draw.rounded_rectangle([16, 230, 304, 410], radius=18, fill=(68, 56, 98))
            draw.rounded_rectangle([16, 430, 304, 600], radius=18, fill=(42, 88, 96))
            img.save(ppath)

        # 4. 检查并兜底生成 TEX_DIR 下的基础材质与人物表情贴图（防止 textures.py 未打包时缺失）
        base_fallbacks = {
            "floor_wood.png": (118, 78, 50),
            "wall_plaster.png": (216, 198, 176),
            "linen_cream.png": (244, 235, 218),
            "linen_sheet.png": (248, 242, 230),
            "duvet_oatmeal.png": (236, 224, 206),
            "rug_weave.png": (212, 196, 174),
            "lace_black.png": (28, 26, 30),
            "hair_dark.png": (26, 22, 20),
            "night_window.png": (18, 26, 48),
            "art_warm_abstract.png": (198, 134, 88),
            "skin_warm.png": (242, 206, 182),
        }
        for fname, rgb in base_fallbacks.items():
            fpath = os.path.join(TEX_DIR, fname)
            if not os.path.exists(fpath):
                Image.new("RGB", (256, 256), rgb).save(fpath)

        # 5. 检查并兜底生成 5 张女主角面部表情贴图
        for face_name in ("face_indifferent.png", "face_indifferent_blink.png",
                          "face_sad.png", "face_sad_blink.png", "face_sleep.png"):
            fpath = os.path.join(TEX_DIR, face_name)
            if not os.path.exists(fpath):
                img = Image.new("RGB", (512, 512), (242, 206, 182))
                d = ImageDraw.Draw(img)
                is_closed = ("blink" in face_name) or ("sleep" in face_name)
                is_sad = "sad" in face_name
                # 眉毛
                if is_sad:
                    d.line([(155, 195), (215, 180)], fill=(55, 38, 30), width=6)
                    d.line([(297, 180), (357, 195)], fill=(55, 38, 30), width=6)
                else:
                    d.line([(155, 185), (215, 182)], fill=(55, 38, 30), width=6)
                    d.line([(297, 182), (357, 185)], fill=(55, 38, 30), width=6)
                # 眼睛
                if is_closed:
                    d.arc([160, 215, 215, 242], start=10, end=170, fill=(45, 32, 26), width=5)
                    d.arc([297, 215, 352, 242], start=10, end=170, fill=(45, 32, 26), width=5)
                else:
                    d.ellipse([172, 212, 204, 246], fill=(36, 26, 22))
                    d.ellipse([308, 212, 340, 246], fill=(36, 26, 22))
                    d.ellipse([178, 216, 188, 226], fill=(255, 255, 255))
                    d.ellipse([314, 216, 324, 226], fill=(255, 255, 255))
                # 嘴巴
                if is_sad:
                    d.arc([232, 330, 280, 356], start=195, end=345, fill=(175, 95, 88), width=5)
                else:
                    d.arc([234, 320, 278, 344], start=15, end=165, fill=(185, 98, 90), width=5)
                img.save(fpath)

    def _load_textures(self):
        """检查并加载场景所需的全部 PNG 材质贴图（木纹、布艺、人物肤色、五种表情、电子钟与字幕）。"""
        need = [
            "floor_wood.png", "wall_plaster.png", "linen_cream.png",
            "linen_sheet.png", "duvet_oatmeal.png", "rug_weave.png",
            "lace_black.png", "hair_dark.png", "night_window.png",
            "art_warm_abstract.png", "skin_warm.png",
            "face_indifferent.png", "face_indifferent_blink.png",
            "face_sad.png", "face_sad_blink.png", "face_sleep.png",
        ]
        # 若基础贴图、气泡字幕贴图或电子钟贴图有任何一项缺失，均触发生成逻辑！
        missing_tex = [n for n in need if not os.path.exists(os.path.join(TEX_DIR, n))]
        missing_speech = [i for i in range(1, 5) if not os.path.exists(os.path.join(SPEECH_DIR, f"speech_plate_{i}.png"))]
        missing_clock = [s for s in range(40, 60) if not os.path.exists(os.path.join(CLOCK_DIR, f"clock_11_59_{s:02d}.png"))]
        if not os.path.exists(os.path.join(CLOCK_DIR, "clock_12_00_00.png")):
            missing_clock.append(60)

        if missing_tex or missing_speech or missing_clock:
            if not self.quiet:
                print("检测到贴图/字幕/闹钟资源缺失，正在生成程序化材质...")
            if textures is not None and hasattr(textures, "make_all"):
                try:
                    textures.make_all()
                except Exception as e:
                    print("调用 textures.make_all() 异常:", e)
            # 兜底保障：若气泡字幕或电子闹钟 PNG 仍不存在，使用内置 PIL 直接绘制生成！
            self._ensure_speech_and_clock_pngs()
        def _safe_load_tex(full_path):
            """安全加载纹理：从纯英文缓存目录（ROOT）以正斜杠绝对路径加载，确保 PyBullet OpenGL 100% 识别。"""
            if not full_path or not os.path.exists(full_path):
                return None
            norm_p = os.path.abspath(full_path).replace("\\", "/")
            tex_id = p.loadTexture(norm_p)
            if tex_id is not None and tex_id >= 0:
                return tex_id
            return None

        loaded = {}
        for n in need:
            path = os.path.join(TEX_DIR, n)
            loaded[n] = _safe_load_tex(path)

        # 加载手机屏幕贴图与机器人 1~4 号气泡字幕贴图
        phone_path = os.path.join(PHONE_DIR, "phone_screen.png")
        loaded["phone"] = _safe_load_tex(phone_path)
        for i in range(1, 5):
            spath = os.path.join(SPEECH_DIR, f"speech_plate_{i}.png")
            loaded[f"speech{i}"] = _safe_load_tex(spath)

        # 加载 11:59:40 至 12:00:00 的 21 张电子闹钟数字贴图
        for s in range(40, 60):
            key = "clock_11_59_%02d" % s
            path = os.path.join(CLOCK_DIR, "clock_11_59_%02d.png" % s)
            loaded[key] = _safe_load_tex(path)
        c12 = os.path.join(CLOCK_DIR, "clock_12_00_00.png")
        loaded["clock_12_00_00"] = _safe_load_tex(c12)
        return loaded

    def _ensure_meshes(self):
        """生成通用单位球、单位平面、圆角字幕板、圆角时钟外壳与女主角面部曲面贴片 OBJ 网格（带内置 UV 平面与面板兜底生成，确保 EXE 中闹钟和字幕 100% 正常显示贴图）。"""
        mesh_dir = os.path.join(ASSETS, "meshes")
        os.makedirs(mesh_dir, exist_ok=True)
        if meshgen is not None:
            try:
                self.unit_sphere = meshgen.uv_sphere_mesh("unit_sphere", rings=28, segments=40)
                self.unit_quad = meshgen.quad_mesh("unit_quad", 1.0, 1.0)
                self.meshes["speech_panel"] = meshgen.rounded_rect_panel(
                    "speech_panel", 1.0, 0.25, 0.039, thickness=0.006, segments=14)
                self.meshes["clock_panel"] = meshgen.rounded_rect_panel(
                    "clock_panel", 1.0, 0.50, 0.08, thickness=0.06, segments=10)
                self.meshes["head_face"] = meshgen.ellipsoid_patch_mesh(
                    "head_face", 0.080, 0.094, 0.089,
                    az_half=math.radians(64), el_half=math.radians(54),
                    nu=30, nv=30)
            except Exception as e:
                print("调用 meshgen 生成网格异常，启用内置网格兜底:", e)
                self.unit_sphere = ""
                self.unit_quad = ""
                self.meshes["speech_panel"] = ""
                self.meshes["clock_panel"] = ""
                self.meshes["head_face"] = ""
        else:
            self.unit_sphere = ""
            self.unit_quad = ""
            self.meshes["speech_panel"] = ""
            self.meshes["clock_panel"] = ""
            self.meshes["head_face"] = ""

        # 关键兜底：PyBullet 的 GEOM_BOX 不包含标准正面 0~1 UV 映射，会导致闹钟和字幕贴图无法显示！
        # 若 unit_quad 或 speech_panel 缺失，直接生成带标准 UV 坐标的 OBJ 网格文件。
        if not self.unit_quad or not os.path.exists(self.unit_quad):
            quad_obj = os.path.join(mesh_dir, "unit_quad.obj")
            with open(quad_obj, "w", encoding="utf-8") as f:
                f.write("# unit_quad with standard UV\n"
                        "v -0.5 -0.5 0.0\nv 0.5 -0.5 0.0\nv 0.5 0.5 0.0\nv -0.5 0.5 0.0\n"
                        "vt 0.0 0.0\nvt 1.0 0.0\nvt 1.0 1.0\nvt 0.0 1.0\n"
                        "vn 0.0 0.0 1.0\n"
                        "f 1/1/1 2/2/1 3/3/1\nf 1/1/1 3/3/1 4/4/1\n")
            self.unit_quad = quad_obj

        if not self.meshes.get("speech_panel") or not os.path.exists(self.meshes.get("speech_panel", "")):
            sp_obj = os.path.join(mesh_dir, "speech_panel.obj")
            with open(sp_obj, "w", encoding="utf-8") as f:
                f.write("# speech_panel quad (1.0 x 0.25) with standard UV\n"
                        "v -0.5 -0.125 0.0\nv 0.5 -0.125 0.0\nv 0.5 0.125 0.0\nv -0.5 0.125 0.0\n"
                        "vt 0.0 0.0\nvt 1.0 0.0\nvt 1.0 1.0\nvt 0.0 1.0\n"
                        "vn 0.0 0.0 1.0\n"
                        "f 1/1/1 2/2/1 3/3/1\nf 1/1/1 3/3/1 4/4/1\n")
            self.meshes["speech_panel"] = sp_obj

    def load_tex(self, key):
        """根据贴图键名获取已加载的 PyBullet 纹理 ID。"""
        return self.tex.get(key, None)

    def register_ambient_body(self, body, rgba=(1, 1, 1, 1), spec=(0.25, 0.25, 0.25)):
        """将刚体及其初始颜色/高光属性登记到环境光列表中，供助眠灯光暗化时按比例插值。"""
        if body is not None and body != -1:
            self.scene_ambient_bodies.append({
                "body": body,
                "base_rgba": [float(c) for c in rgba] if rgba is not None else [1.0, 1.0, 1.0, 1.0],
                "base_spec": [float(s) for s in spec] if spec is not None else [0.25, 0.25, 0.25],
            })
        return body

    # -----------------------------------------------------------------------
    # 场景基础图元创建方法（网格、长方体、圆柱、胶囊体、椭球、球体、平面卡、线段）
    # -----------------------------------------------------------------------
    def add_visual(self, vis, pos=(0, 0, 0), quat=(0, 0, 0, 1)):
        """由视觉形状索引 vis 创建质量为 0 的静态/运动学视觉刚体。"""
        return p.createMultiBody(0, -1, vis, list(pos), list(quat))

    def add_mesh(self, mesh_path, scale, pos, quat=(0, 0, 0, 1),
                rgba=(1, 1, 1, 1), tex=None, spec=(0.25, 0.25, 0.25),
                double_sided=False):
        """创建自定义 OBJ 网格刚体并绑定纹理贴图。"""
        flags = p.VISUAL_SHAPE_DOUBLE_SIDED if double_sided else 0
        if mesh_path and os.path.exists(mesh_path):
            vis = p.createVisualShape(
                p.GEOM_MESH, fileName=mesh_path, meshScale=list(scale),
                rgbaColor=list(rgba), specularColor=list(spec), flags=flags)
        else:
            vis = p.createVisualShape(
                p.GEOM_SPHERE, radius=scale[0],
                rgbaColor=list(rgba), specularColor=list(spec))
        body = self.add_visual(vis, pos, quat)
        if tex is not None:
            p.changeVisualShape(body, -1, textureUniqueId=tex, rgbaColor=list(rgba))
        return self.register_ambient_body(body, rgba, spec)

    def add_box(self, half, pos, quat=(0, 0, 0, 1), rgba=(1, 1, 1, 1),
                spec=(0.25, 0.25, 0.25), tex=None):
        """根据半长向量 half=(hx, hy, hz) 创建长方体视觉刚体。"""
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=list(half),
                                rgbaColor=list(rgba), specularColor=list(spec))
        body = self.add_visual(vis, pos, quat)
        if tex is not None:
            p.changeVisualShape(body, -1, textureUniqueId=tex, rgbaColor=list(rgba))
        return self.register_ambient_body(body, rgba, spec)

    def add_cylinder(self, radius, length, pos, quat=(0, 0, 0, 1),
                    rgba=(1, 1, 1, 1), spec=(0.25, 0.25, 0.25)):
        """根据半径 radius 与高度 length 创建圆柱体视觉刚体。"""
        vis = p.createVisualShape(p.GEOM_CYLINDER, radius=radius,
                                length=length, rgbaColor=list(rgba),
                                specularColor=list(spec))
        body = self.add_visual(vis, pos, quat)
        return self.register_ambient_body(body, rgba, spec)

    def add_capsule(self, radius, length, pos, quat=(0, 0, 0, 1),
                    rgba=(1, 1, 1, 1), spec=(0.25, 0.25, 0.25)):
        """根据半径与柱体段长度创建两端带半球的胶囊体刚体（常用于平滑衔接的四肢骨骼）。"""
        vis = p.createVisualShape(p.GEOM_CAPSULE, radius=radius,
                                length=length, rgbaColor=list(rgba),
                                specularColor=list(spec))
        body = self.add_visual(vis, pos, quat)
        return self.register_ambient_body(body, rgba, spec)

    def add_ellipsoid(self, radii, pos, quat=(0, 0, 0, 1),
                    rgba=(1, 1, 1, 1), tex=None,
                    spec=(0.25, 0.25, 0.25), double_sided=False):
        """通过对单位球体网格施加非均匀三轴缩放 radii=(rx, ry, rz) 创建椭球体刚体。"""
        if self.unit_sphere and os.path.exists(self.unit_sphere):
            return self.add_mesh(self.unit_sphere, radii, pos, quat,
                                rgba=rgba, tex=tex, spec=spec,
                                double_sided=double_sided)
        return self.add_sphere(radii[0], pos, quat=quat, rgba=rgba, spec=spec)

    def add_sphere(self, radius, pos, quat=(0, 0, 0, 1),
                rgba=(1, 1, 1, 1), spec=(0.25, 0.25, 0.25)):
        """创建标准球体视觉刚体。"""
        vis = p.createVisualShape(p.GEOM_SPHERE, radius=radius,
                                rgbaColor=list(rgba), specularColor=list(spec))
        body = self.add_visual(vis, pos, quat)
        return self.register_ambient_body(body, rgba, spec)

    def add_card(self, width, height, pos, normal, up_hint=(0, 0, 1),
                tex=None, rgba=(1, 1, 1, 1), spec=(0.15, 0.15, 0.15)):
        """创建指定法线朝向 normal 与宽高尺寸的平面薄卡片（用于地板、墙面、壁画、窗户与屏幕）。"""
        z = norm(normal)
        y = np.asarray(up_hint, dtype=float)
        y = norm(y - z * np.dot(y, z))
        if np.linalg.norm(y) < 1e-6:
            y = np.array([1.0, 0.0, 0.0])
        x = norm(np.cross(y, z))
        y = np.cross(z, x)
        quat = quat_from_matrix(np.column_stack([x, y, z]))
        if self.unit_quad and os.path.exists(self.unit_quad):
            vis = p.createVisualShape(
                p.GEOM_MESH, fileName=self.unit_quad,
                meshScale=[width, height, 1.0], rgbaColor=list(rgba),
                specularColor=list(spec), flags=p.VISUAL_SHAPE_DOUBLE_SIDED)
        else:
            vis = p.createVisualShape(
                p.GEOM_BOX, halfExtents=[width / 2, height / 2, 0.001],
                rgbaColor=list(rgba), specularColor=list(spec))
        body = self.add_visual(vis, pos, quat)
        if tex is not None:
            p.changeVisualShape(body, -1, textureUniqueId=tex, rgbaColor=list(rgba))
        return self.register_ambient_body(body, rgba, spec)

    def add_segment(self, p0, p1, radius, rgba=(1, 1, 1, 1),
                    kind="capsule", spec=(0.3, 0.3, 0.3)):
        """在空间两点 p0 与 p1 之间生成连接圆柱或胶囊体线段（用于灯线、植物茎秆、机械臂连杆等）。"""
        p0 = np.asarray(p0, dtype=float)
        p1 = np.asarray(p1, dtype=float)
        d = p1 - p0
        L = float(np.linalg.norm(d))
        if L < 1e-6:
            return None
        q = quat_from_z(d)
        mid = (p0 + p1) * 0.5
        if kind == "cylinder":
            return self.add_cylinder(radius, L, mid, q, rgba=rgba, spec=spec)
        length = max(0.004, L - 2.0 * radius)
        return self.add_capsule(radius, length, mid, q, rgba=rgba, spec=spec)

    def set_pose(self, body, pos, quat):
        """设置刚体的三维位置与四元数旋转姿态（若属于主体刚体，底层钩子会自动执行 1.5 倍坐标变换）。"""
        p.resetBasePositionAndOrientation(body, list(pos), list(quat))

    def change_tex(self, body, tex):
        """更换指定刚体的表面纹理贴图。"""
        if tex is not None:
            p.changeVisualShape(body, -1, textureUniqueId=tex, rgbaColor=[1, 1, 1, 1])

    # -----------------------------------------------------------------------
    # 房间建筑外壳、夜窗与【核心优化：动态可拉合窗帘】构建
    # -----------------------------------------------------------------------
    def _build_room(self):
        """构建卧室地面、四周墙体、天花板、踢脚线、右侧夜窗、动态双开窗帘、地毯与装饰画。"""
        dark_wood = (0.30, 0.19, 0.12, 1.0)    # 深胡桃木地板底座色
        plaster = (0.83, 0.75, 0.66, 1.0)      # 暖米灰石膏墙面色
        warm_white = (0.96, 0.92, 0.85, 1.0)   # 象牙白踢脚线与窗框色
        brass = (0.72, 0.54, 0.28, 1.0)        # 复古黄铜罗马杆色

        # 1. 铺设木纹地板卡片与底层厚木板
        self.add_card(6.4, 6.0, (0.0, 0.0, 0.001),
                    normal=(0, 0, 1), up_hint=(0, 1, 0),
                    tex=self.load_tex("floor_wood.png"),
                    spec=(0.22, 0.20, 0.17))
        self.add_box((3.2, 3.0, 0.05), (0, 0, -0.05), rgba=dark_wood)

        # 2. 构建后墙 (y=2.95)、左墙 (x=-3.15)、右墙 (x=3.15) 与天花板 (z=2.86)
        self.add_box((3.2, 0.08, 1.45), (0, 2.95, 1.45), rgba=plaster)
        self.add_box((0.08, 3.0, 1.45), (-3.15, 0, 1.45), rgba=plaster)
        self.add_box((0.08, 3.0, 1.45), (3.15, 0, 1.45), rgba=plaster)
        self.add_box((3.2, 3.0, 0.08), (0, 0, 2.86), rgba=(0.90, 0.85, 0.78, 1.0))

        # 后墙肌理贴图面片与四周墙角踢脚线、顶角线
        wall_card = self.add_card(6.2, 2.78, (0, 2.865, 1.42),
                    normal=(0, -1, 0), up_hint=(0, 0, 1),
                    tex=self.load_tex("wall_plaster.png"),
                    spec=(0.05, 0.05, 0.05))
        self.sleep_dimmable_items.append({"body": wall_card, "base_rgba": [1.0, 1.0, 1.0, 1.0], "target_rgba": [0.55, 0.52, 0.50, 1.0]})
        for args in (
            ((3.18, 0.055, 0.09), (0, 2.86, 0.09)),
            ((0.055, 3.0, 0.09), (-3.04, 0, 0.09)),
            ((0.055, 3.0, 0.09), (3.04, 0, 0.09)),
        ):
            self.add_box(args[0], args[1], rgba=warm_white, spec=(0.15, 0.15, 0.15))
        self.add_box((3.18, 0.05, 0.055), (0, 2.87, 2.75), rgba=warm_white)

        # 3. 构建右侧墙面（x ≈ 3.00）的星空夜景窗户与十字窗棂、窗台板
        win_y, win_z, win_w, win_h = 0.65, 1.58, 1.75, 1.42
        self.add_card(win_w, win_h, (3.035, win_y, win_z),
                    normal=(-1, 0, 0), up_hint=(0, 0, 1),
                    tex=self.load_tex("night_window.png"),
                    spec=(0.05, 0.05, 0.05))
        frame = (0.98, 0.95, 0.89, 1.0)
        for dz in (-win_h / 2, win_h / 2):
            self.add_box((0.035, win_w / 2 + 0.06, 0.045), (3.00, win_y, win_z + dz), rgba=frame)
        for dy in (-win_w / 2, win_w / 2):
            self.add_box((0.035, 0.045, win_h / 2 + 0.06), (3.00, win_y + dy, win_z), rgba=frame)
        self.add_box((0.025, 0.022, win_h / 2), (3.01, win_y, win_z), rgba=frame)
        self.add_box((0.025, win_w / 2, 0.022), (3.01, win_y, win_z), rgba=frame)
        self.add_box((0.12, win_w / 2 + 0.10, 0.035),
                    (2.98, win_y, win_z - win_h / 2 - 0.06), rgba=frame)

        # 4. 窗户上方的黄铜罗马窗帘杆与两端球形装饰头
        rod_y0, rod_y1 = win_y - win_w / 2 - 0.20, win_y + win_w / 2 + 0.20
        rod_z = win_z + win_h / 2 + 0.18
        self.add_cylinder(0.022, rod_y1 - rod_y0,
                        (2.92, (rod_y0 + rod_y1) / 2, rod_z),
                        quat=quat_from_z((0, 1, 0)), rgba=brass, spec=(0.6, 0.5, 0.3))
        for yy in (rod_y0, rod_y1):
            self.add_ellipsoid((0.032, 0.032, 0.032), (2.92, yy, rod_z),
                            rgba=brass, spec=(0.6, 0.5, 0.3))

        # ==================== 【核心优化：构建可动态拉合的双开多褶皱布艺窗帘】 ====================
        # 设计原理：
        # 在窗户左右两侧各布置 10 道竖直波浪褶皱柱（pleat_body）及顶部黄铜挂环（ring_body）。
        # - 初始敞开状态 (close_ratio = 0.0)：左右各 10 道褶皱紧密收拢在窗框外侧 0.20m 范围内，露出完整夜窗；
        # - 助眠拉拢状态 (close_ratio = 1.0)：左右褶皱沿罗马杆平滑滑向窗户中心线 win_y，相邻褶皱严密交叠，将窗户完全遮蔽。
        curtain_col = (0.88, 0.81, 0.71, 1.0)
        curtain_q = quat_from_z((0, 0, 1))
        ring_q = quat_from_z((0, 1, 0))
        self.curtain_pleats = []
        n_pleats_per_side = 10

        for side in (-1, 1):  # side = -1 为左半扇窗帘，side = +1 为右半扇窗帘
            y_outer = win_y + side * (win_w / 2 + 0.14)  # 外侧固定基准边
            for k in range(n_pleats_per_side):
                u_frac = k / float(n_pleats_per_side - 1)  # 褶皱序号归一化比例 [0.0 ~ 1.0]
                # 敞开时的 Y 坐标（收拢在窗边）与拉拢时的 Y 坐标（展开至窗户正中心 win_y）
                y_open = y_outer - side * (u_frac * 0.20)
                y_closed = y_outer - side * (u_frac * (win_w / 2 + 0.12))
                # 奇偶褶皱在 X 方向轻微前后交错，呈现真实布料波浪层次感
                x_pleat = 2.885 - (k % 2) * 0.028

                pleat_body = self.add_ellipsoid(
                    (0.055, 0.082, win_h / 2 + 0.18),
                    (x_pleat, y_open, win_z - 0.05),
                    quat=curtain_q,
                    rgba=curtain_col,
                    spec=(0.05, 0.05, 0.05)
                )
                ring_body = self.add_cylinder(
                    0.029, 0.012,
                    (2.92, y_open, rod_z),
                    quat=ring_q,
                    rgba=brass,
                    spec=(0.65, 0.52, 0.30)
                )
                self.curtain_pleats.append({
                    "pleat_body": pleat_body,
                    "ring_body": ring_body,
                    "x": x_pleat,
                    "y_open": y_open,
                    "y_closed": y_closed,
                    "z_pleat": win_z - 0.05,
                    "z_ring": rod_z,
                    "pleat_q": curtain_q,
                    "ring_q": ring_q,
                })

        # 5. 床前编织地毯与后墙高处暖色抽象装饰挂画
        if meshgen is not None:
            rug_path = meshgen.rounded_rect_panel(
                "rug_panel", 2.8, 1.95, 0.10, thickness=0.015, segments=10)
            self.add_mesh(rug_path, (1, 1, 1), (0.45, -0.85, 0.012),
                        tex=self.load_tex("rug_weave.png"),
                        rgba=(1, 1, 1, 1), spec=(0.05, 0.05, 0.05))

        art_c = (0.85, 2.855, 2.38)
        self.add_box((0.52, 0.02, 0.34), (art_c[0], art_c[1] + 0.01, art_c[2]),
                    rgba=(0.24, 0.14, 0.08, 1.0))
        self.add_card(0.90, 0.54, (art_c[0], art_c[1] - 0.012, art_c[2]),
                    normal=(0, -1, 0), up_hint=(0, 0, 1),
                    tex=self.load_tex("art_warm_abstract.png"),
                    spec=(0.05, 0.05, 0.05))

    def _update_curtains(self, close_ratio: float):
        """动态更新窗帘拉合位置：
        参数 close_ratio ∈ [0.0, 1.0]，0.0 表示窗帘完全敞开，1.0 表示左右窗帘在窗户中央完全合拢。
        """
        if p is None or not p.isConnected():
            return
        close_ratio = float(np.clip(close_ratio, 0.0, 1.0))
        self.current_curtain_close = close_ratio
        for item in getattr(self, "curtain_pleats", []):
            # 按当前拉合比例在 y_open 与 y_closed 之间线性插值每道褶皱与挂环的水平位置
            y_cur = lerp(item["y_open"], item["y_closed"], close_ratio)
            p.resetBasePositionAndOrientation(
                item["pleat_body"],
                [item["x"], float(y_cur), item["z_pleat"]],
                list(item["pleat_q"])
            )
            p.resetBasePositionAndOrientation(
                item["ring_body"],
                [2.92, float(y_cur), item["z_ring"]],
                list(item["ring_q"])
            )

    # -----------------------------------------------------------------------
    # 卧室家具与装饰物构建（以下主体组件在构建时均自动享受 1.5 倍等比例放大）
    # -----------------------------------------------------------------------
    def _build_bed(self):
        """构建双人床主体：含四根实木床腿、床框、软包床头板、15盏星光串灯、床垫、三只枕头，
        以及【核心优化：可在人物平躺入睡时向上拉起盖住上半身的分段动态羽绒被系统】。
        """
        sheet = self.load_tex("linen_sheet.png")
        linen = self.load_tex("linen_cream.png")
        duvet = self.load_tex("duvet_oatmeal.png")
        wood = (0.40, 0.27, 0.17, 1.0)
        dark = (0.25, 0.16, 0.10, 1.0)

        # 四角实木圆柱床腿与厚实木床框
        for x in (-0.05, 1.55):
            for y in (0.27, 2.18):
                self.add_cylinder(0.045, 0.22, (x, y, 0.11),
                                rgba=dark, spec=(0.15, 0.10, 0.06))
        self.add_box((0.94, 1.08, 0.09), (0.70, 1.22, 0.27),
                    rgba=wood, spec=(0.15, 0.1, 0.06))
        self.add_box((0.97, 0.08, 0.67), (0.70, 2.36, 0.97),
                    rgba=(0.33, 0.22, 0.14, 1.0))

        # 奶油色亚麻软包床头板
        if meshgen is not None:
            hb = meshgen.rounded_rect_panel("headboard_panel", 1.88, 1.22,
                                            0.13, thickness=0.12, segments=12)
            q_hb = quat_from_z((0, -1, 0))
            hb_body = self.add_mesh(hb, (1, 1, 1), (0.70, 2.285, 0.94), q_hb,
                        tex=linen, rgba=(0.96, 0.92, 0.85, 1.0),
                        spec=(0.08, 0.08, 0.08))
            self.sleep_dimmable_items.append({"body": hb_body, "base_rgba": [0.96, 0.92, 0.85, 1.0], "target_rgba": [0.52, 0.48, 0.44, 1.0]})

        # 悬挂于床头板上沿的 15 盏暖黄星光氛围灯串（自然垂链曲线分布）
        nlights = 15
        prev = None
        self.bed_string_lights = []
        for i in range(nlights):
            x = -0.12 + i * (1.64 / (nlights - 1))
            z = 1.545 - 0.045 * math.sin(math.pi * i / (nlights - 1)) ** 2
            pos = v3(x, 2.235, z)
            if prev is not None:
                self.add_segment(prev, pos, 0.0015,
                                rgba=(0.28, 0.21, 0.12, 1.0), kind="cylinder")
            b = self.add_ellipsoid((0.012, 0.012, 0.014), pos,
                            rgba=(1.0, 0.82, 0.48, 1.0), spec=(1.0, 0.9, 0.6))
            self.bed_string_lights.append(b)
            prev = pos

        # 饱满圆角床垫与顶层平整床单
        if meshgen is not None:
            mattress = meshgen.rounded_rect_panel("mattress", 1.78, 2.06, 0.12,
                                                thickness=0.26, segments=12)
            self.add_mesh(mattress, (1, 1, 1), (0.70, 1.22, 0.54),
                        tex=sheet, rgba=(0.98, 0.95, 0.90, 1.0),
                        spec=(0.08, 0.08, 0.08))
            top_sheet = meshgen.rounded_rect_panel("top_sheet", 1.75, 2.02, 0.10,
                                                    thickness=0.022, segments=10)
            self.add_mesh(top_sheet, (1, 1, 1), (0.70, 1.22, 0.665),
                        tex=sheet, rgba=(0.98, 0.95, 0.90, 1.0),
                        spec=(0.06, 0.06, 0.06))

        # 三只并排斜倚在床头的饱满羽绒枕（左侧枕头承载女主角平躺后的头部）
        for x, y, z, nrm, rz, rr in (
            (0.14, 2.01, 0.835, (-0.10, -0.93, 0.34), 0.10, (0.285, 0.170, 0.108)),
            (0.68, 2.03, 0.830, (0.04, -0.94, 0.32), -0.05, (0.305, 0.175, 0.110)),
            (1.25, 2.01, 0.820, (0.14, -0.93, 0.34), -0.12, (0.270, 0.155, 0.100)),
        ):
            q = frame_quat(nrm, (0, 0, 1))
            q = p.multiplyTransforms([0, 0, 0], q, [0, 0, 0],
                                    p.getQuaternionFromEuler([0, 0, rz]))[1]
            pillow_body = self.add_ellipsoid(rr, (x, y, z), q,
                            tex=linen, rgba=(0.98, 0.94, 0.88, 1.0),
                            spec=(0.04, 0.04, 0.04))
            self.sleep_dimmable_items.append({"body": pillow_body, "base_rgba": [0.98, 0.94, 0.88, 1.0], "target_rgba": [0.56, 0.53, 0.48, 1.0]})

        # ==================== 【核心优化：分段式可拉起盖住上半身的动态羽绒被系统】 ====================
        # 1. 下段固定护腿羽绒被（保持覆盖床尾与腿部 y ∈ [0.26, 1.30]，被子拉起时床尾不露底）
        self.pull_duvet_parts = []
        if meshgen is not None:
            duvet_panel = meshgen.rounded_rect_panel("duvet_panel", 1.86, 1.04,
                                                    0.12, thickness=0.16, segments=12)
            duvet_body = self.add_mesh(duvet_panel, (1, 1, 1), (0.70, 0.78, 0.79),
                        tex=duvet, rgba=(0.95, 0.90, 0.83, 1.0), spec=(0.04, 0.04, 0.04))
            self.sleep_dimmable_items.append({"body": duvet_body, "base_rgba": [0.95, 0.90, 0.83, 1.0], "target_rgba": [0.55, 0.50, 0.45, 1.0]})

            top_quilt = meshgen.quilt_mesh(
                "duvet_top_soft", 1.83, 1.02, nu=46, nv=30, amplitude=1.15, seed=3)
            top_quilt_body = self.add_mesh(top_quilt, (1, 1, 1), (0.70, 0.80, 0.878),
                        tex=duvet, rgba=(0.97, 0.93, 0.86, 1.0),
                        spec=(0.04, 0.04, 0.04), double_sided=True)
            self.sleep_dimmable_items.append({"body": top_quilt_body, "base_rgba": [0.97, 0.93, 0.86, 1.0], "target_rgba": [0.56, 0.52, 0.47, 1.0]})

            # 2. 上段可拉起盖住上半身的动态羽绒被芯与表面起伏褶皱层：
            #    - 初始状态 (cover_ratio = 0.0)：叠合收拢在腿部被窝内 (y = 0.92)，不遮挡靠坐的腰腹；
            #    - 睡眠拉起状态 (cover_ratio = 1.0)：随女主角双手向上拉起展开至 (y = 1.45, z = 0.855)，
            #      前端直达肩颈锁骨处 (y = 1.79)，严密盖住整个上半身（腰、腹、胸、双肩与上臂）！
            upper_duvet_panel = meshgen.rounded_rect_panel(
                "upper_duvet_panel", 1.85, 0.68, 0.11, thickness=0.165, segments=12)
            upper_duvet_body = self.add_mesh(
                upper_duvet_panel, (1, 1, 1), (0.70, 0.92, 0.785),
                tex=duvet, rgba=(0.95, 0.90, 0.83, 1.0), spec=(0.04, 0.04, 0.04))
            self.pull_duvet_parts.append({
                "body": upper_duvet_body,
                "pos_open": v3(0.70, 0.92, 0.785),
                "pos_covered": v3(0.70, 1.45, 0.855),
                "pitch_open": 0.0,
                "pitch_covered": math.radians(7.5),
            })

            upper_quilt = meshgen.quilt_mesh(
                "upper_duvet_quilt", 1.82, 0.66, nu=44, nv=26, amplitude=1.18, seed=7)
            upper_quilt_body = self.add_mesh(
                upper_quilt, (1, 1, 1), (0.70, 0.92, 0.875),
                tex=duvet, rgba=(0.97, 0.93, 0.86, 1.0),
                spec=(0.04, 0.04, 0.04), double_sided=True)
            self.pull_duvet_parts.append({
                "body": upper_quilt_body,
                "pos_open": v3(0.70, 0.92, 0.875),
                "pos_covered": v3(0.70, 1.45, 0.942),
                "pitch_open": 0.0,
                "pitch_covered": math.radians(7.5),
            })

            # 3. 象牙白被头反折边（随双手拉被动作从腿部 y=1.16 平滑拉至肩颈锁骨处 y=1.69）
            turned = meshgen.quilt_mesh(
                "turned_sheet", 1.72, 0.24, nu=32, nv=8, amplitude=0.35, seed=4)
            turned_body = self.add_mesh(turned, (1, 1, 1), (0.70, 1.16, 0.895),
                        tex=sheet, rgba=(0.985, 0.96, 0.92, 1.0),
                        spec=(0.04, 0.04, 0.04), double_sided=True)
            self.sleep_dimmable_items.append({"body": turned_body, "base_rgba": [0.985, 0.96, 0.92, 1.0], "target_rgba": [0.58, 0.55, 0.51, 1.0]})
            self.pull_duvet_parts.append({
                "body": turned_body,
                "pos_open": v3(0.70, 1.16, 0.895),
                "pos_covered": v3(0.70, 1.69, 0.968),
                "pitch_open": 0.0,
                "pitch_covered": math.radians(8.0),
            })
        else:
            # 无 meshgen 时的回退几何体构建
            upper_duvet_body = self.add_box(
                (0.92, 0.34, 0.082), (0.70, 0.92, 0.785),
                rgba=(0.95, 0.90, 0.83, 1.0), tex=duvet, spec=(0.04, 0.04, 0.04))
            turned_body = self.add_box(
                (0.86, 0.12, 0.015), (0.70, 1.16, 0.895),
                rgba=(0.985, 0.96, 0.92, 1.0), tex=sheet, spec=(0.04, 0.04, 0.04))
            self.pull_duvet_parts.append({
                "body": upper_duvet_body,
                "pos_open": v3(0.70, 0.92, 0.785),
                "pos_covered": v3(0.70, 1.45, 0.855),
                "pitch_open": 0.0,
                "pitch_covered": math.radians(7.5),
            })
            self.pull_duvet_parts.append({
                "body": turned_body,
                "pos_open": v3(0.70, 1.16, 0.895),
                "pos_covered": v3(0.70, 1.69, 0.968),
                "pitch_open": 0.0,
                "pitch_covered": math.radians(8.0),
            })

        # 4. 女主角胸肩专属蓬松拱形被窝包络层（确保被子拉起后立体包裹胸口与双肩，绝不穿模外露）
        torso_drape_body = self.add_ellipsoid(
            (0.38, 0.33, 0.078), (0.28, 0.90, 0.825),
            tex=duvet, rgba=(0.96, 0.92, 0.85, 1.0), spec=(0.04, 0.04, 0.04))
        self.pull_duvet_parts.append({
            "body": torso_drape_body,
            "pos_open": v3(0.28, 0.90, 0.825),
            "pos_covered": v3(0.28, 1.48, 0.918),
            "pitch_open": 0.0,
            "pitch_covered": math.radians(8.5),
        })

    def _update_duvet(self, cover_ratio: float):
        """动态更新被子拉起盖住人物上半身的位姿：
        参数 cover_ratio ∈ [0.0, 1.0]：
          - 0.0：初始靠坐状态，被子仅盖住腿部（y <= 1.28）；
          - 1.0：平躺入睡状态，被子随双手拉起至肩颈锁骨处（y ≈ 1.79），严密温柔地盖住整个上半身。
        """
        if p is None or not p.isConnected():
            return
        cover_ratio = float(np.clip(cover_ratio, 0.0, 1.0))
        self.current_duvet_cover = cover_ratio
        # 在拉被子中途（cover_ratio 约 0.5 时）附加一个微小的向上掀起弧度，模拟双手提拉被头的真实布料动态
        lift_arc = math.sin(math.pi * cover_ratio) * 0.024
        for item in getattr(self, "pull_duvet_parts", []):
            pos = lerp(item["pos_open"], item["pos_covered"], cover_ratio) + v3(0.0, 0.0, lift_arc)
            pitch = float(lerp(item["pitch_open"], item["pitch_covered"], cover_ratio))
            quat = p.getQuaternionFromEuler([pitch, 0.0, 0.0])
            p.resetBasePositionAndOrientation(item["body"], list(pos), list(quat))

    def _build_lounge_chair(self):
        """构建房间左后侧的单人休闲扶手椅（浅燕麦色布艺坐垫 + 胡桃木斜腿 + 焦糖披毯与姜黄腰枕）。"""
        fabric = (0.92, 0.88, 0.82, 1.0)
        cushion_col = (0.95, 0.92, 0.86, 1.0)
        walnut = (0.35, 0.22, 0.14, 1.0)
        throw_col = (0.74, 0.42, 0.28, 1.0)
        pillow_col = (0.84, 0.66, 0.32, 1.0)
        cx, cy = -1.95, 1.15
        # 四条外倾实木椅腿
        for dx, dy in ((-0.16, -0.16), (0.16, -0.16), (-0.16, 0.16), (0.16, 0.16)):
            q_leg = quat_from_z((dx * 0.18, dy * 0.18, 1.0))
            self.add_cylinder(0.018, 0.34, (cx + dx, cy + dy, 0.17), quat=q_leg,
                              rgba=walnut, spec=(0.2, 0.15, 0.1))
        # 椅座与厚坐垫
        self.add_box((0.24, 0.24, 0.045), (cx, cy, 0.35), rgba=fabric, spec=(0.06, 0.06, 0.06))
        self.add_box((0.22, 0.22, 0.055), (cx, cy - 0.01, 0.42), rgba=cushion_col, spec=(0.05, 0.05, 0.05))
        # 靠背与顶部圆滚边缘
        self.add_box((0.22, 0.055, 0.24), (cx, cy + 0.19, 0.62), rgba=fabric, spec=(0.06, 0.06, 0.06))
        self.add_cylinder(0.055, 0.44, (cx, cy + 0.19, 0.74), quat=quat_from_z((1, 0, 0)),
                          rgba=fabric, spec=(0.06, 0.06, 0.06))
        # 左右两侧扶手
        for side in (-1, 1):
            self.add_box((0.045, 0.22, 0.12), (cx + side * 0.22, cy, 0.50), rgba=fabric, spec=(0.06, 0.06, 0.06))
            self.add_cylinder(0.045, 0.44, (cx + side * 0.22, cy, 0.62), quat=quat_from_z((0, 1, 0)),
                              rgba=fabric, spec=(0.06, 0.06, 0.06))
        # 扶手披毯与腰枕
        self.add_box((0.09, 0.18, 0.018), (cx + 0.22, cy - 0.03, 0.61), rgba=throw_col, spec=(0.08, 0.06, 0.05))
        self.add_box((0.015, 0.18, 0.12), (cx + 0.28, cy - 0.03, 0.48), rgba=throw_col, spec=(0.08, 0.06, 0.05))
        self.add_ellipsoid((0.14, 0.07, 0.09), (cx, cy + 0.12, 0.48), rgba=pillow_col, spec=(0.10, 0.08, 0.05))

    def _build_indoor_plant(self):
        """构建房间左后角落的三脚木架陶土花盆与 6 片饱满舒展的龟背竹绿植叶片。"""
        wood = (0.65, 0.48, 0.32, 1.0)
        terracotta = (0.80, 0.54, 0.42, 1.0)
        soil_col = (0.22, 0.16, 0.12, 1.0)
        stem_col = (0.28, 0.45, 0.22, 1.0)
        leaf_col = (0.16, 0.34, 0.18, 1.0)
        px, py = -1.95, 2.15
        for ang in (0.0, 2.094, 4.188):
            dx = math.cos(ang) * 0.15
            dy = math.sin(ang) * 0.15
            self.add_cylinder(0.014, 0.36, (px + dx, py + dy, 0.18), rgba=wood, spec=(0.2, 0.15, 0.1))
        self.add_cylinder(0.14, 0.02, (px, py, 0.28), rgba=wood, spec=(0.2, 0.15, 0.1))
        self.add_cylinder(0.135, 0.26, (px, py, 0.44), rgba=terracotta, spec=(0.15, 0.12, 0.10))
        self.add_cylinder(0.145, 0.035, (px, py, 0.56), rgba=terracotta, spec=(0.18, 0.14, 0.12))
        self.add_cylinder(0.128, 0.015, (px, py, 0.55), rgba=soil_col, spec=(0.04, 0.04, 0.04))
        leaf_specs = [
            (0.55, 0.12, 0.72, (0.08, 0.08, 0.22), (0.07, 0.10, 0.014)),
            (0.56, -0.10, 0.76, (-0.09, 0.06, 0.24), (0.08, 0.11, 0.014)),
            (0.58, 0.02, 0.85, (0.02, 0.10, 0.30), (0.09, 0.12, 0.014)),
            (0.57, -0.06, 0.88, (-0.05, 0.08, 0.32), (0.085, 0.115, 0.014)),
            (0.59, 0.10, 0.94, (0.07, 0.04, 0.36), (0.08, 0.11, 0.014)),
            (0.60, -0.02, 0.98, (0.00, 0.06, 0.40), (0.075, 0.105, 0.014)),
        ]
        for z0, ox, oz, stem_vec, radii in leaf_specs:
            base_p = v3(px, py, z0)
            tip_p = base_p + v3(stem_vec[0], stem_vec[1], stem_vec[2])
            self.add_segment(base_p, tip_p, 0.004, rgba=stem_col, kind="cylinder", spec=(0.2, 0.3, 0.2))
            q_leaf = quat_from_z(norm(tip_p - base_p + v3(0, 0.05, 0.03)))
            self.add_ellipsoid(radii, tip_p, quat=q_leaf, rgba=leaf_col, spec=(0.35, 0.45, 0.30))

    def _build_wall_decor(self):
        """构建后墙高处的实木悬浮置物壁架及陶瓷多肉盆栽、黄铜相框、琥珀香薰蜡烛。"""
        oak = (0.76, 0.62, 0.48, 1.0)
        brass = (0.72, 0.54, 0.28, 1.0)
        shelf_x, shelf_y, shelf_z = -1.25, 2.78, 2.15
        self.add_box((0.36, 0.09, 0.018), (shelf_x, shelf_y, shelf_z), rgba=oak, spec=(0.18, 0.14, 0.1))
        for sx in (-0.26, 0.26):
            self.add_box((0.012, 0.07, 0.06), (shelf_x + sx, shelf_y, shelf_z - 0.045), rgba=brass, spec=(0.6, 0.5, 0.3))
        self.add_cylinder(0.028, 0.045, (shelf_x - 0.22, shelf_y, shelf_z + 0.038), rgba=(0.95, 0.95, 0.95, 1.0), spec=(0.3, 0.3, 0.3))
        self.add_ellipsoid((0.032, 0.032, 0.024), (shelf_x - 0.22, shelf_y, shelf_z + 0.072), rgba=(0.42, 0.62, 0.45, 1.0), spec=(0.2, 0.3, 0.2))
        self.add_box((0.055, 0.008, 0.075), (shelf_x - 0.05, shelf_y - 0.01, shelf_z + 0.09), rgba=brass, spec=(0.7, 0.55, 0.3))
        self.add_box((0.045, 0.006, 0.065), (shelf_x - 0.05, shelf_y - 0.014, shelf_z + 0.09), rgba=(0.96, 0.94, 0.90, 1.0), spec=(0.1, 0.1, 0.1))
        self.add_cylinder(0.026, 0.055, (shelf_x + 0.16, shelf_y, shelf_z + 0.045), rgba=(0.58, 0.36, 0.18, 0.92), spec=(0.8, 0.7, 0.5))
        self.add_cylinder(0.022, 0.010, (shelf_x + 0.16, shelf_y, shelf_z + 0.068), rgba=(0.96, 0.93, 0.85, 1.0), spec=(0.1, 0.1, 0.1))
        self.add_sphere(0.0035, (shelf_x + 0.16, shelf_y, shelf_z + 0.078), rgba=(1.0, 0.85, 0.45, 1.0), spec=(1.0, 0.9, 0.5))

    def _build_nightstand_accessories(self):
        """构建床头柜台面上的生活小物件：木质杯垫、玻璃冷水壶与水杯、无火藤条香薰瓶、睡前读物。"""
        glass = (0.88, 0.94, 0.98, 0.80)
        coaster_col = (0.40, 0.26, 0.16, 1.0)
        amber = (0.50, 0.32, 0.16, 0.92)
        brass = (0.75, 0.58, 0.30, 1.0)
        page_col = (0.97, 0.95, 0.90, 1.0)
        cover_col = (0.16, 0.22, 0.34, 1.0)
        # 1. 杯垫与玻璃水具
        self.add_cylinder(0.046, 0.008, (-0.46, 1.82, 0.560), rgba=coaster_col, spec=(0.2, 0.15, 0.1))
        self.add_cylinder(0.030, 0.100, (-0.46, 1.82, 0.615), rgba=glass, spec=(0.9, 0.95, 1.0))
        self.add_cylinder(0.024, 0.055, (-0.53, 1.78, 0.590), rgba=glass, spec=(0.9, 0.95, 1.0))
        # 2. 琥珀玻璃香薰瓶与 4 根扩香藤条
        self.add_box((0.024, 0.024, 0.038), (-0.48, 2.05, 0.594), rgba=amber, spec=(0.85, 0.75, 0.55))
        self.add_cylinder(0.012, 0.014, (-0.48, 2.05, 0.638), rgba=brass, spec=(0.8, 0.65, 0.35))
        diff_base = v3(-0.48, 2.05, 0.645)
        for dx, dy in ((-0.018, -0.012), (0.016, -0.014), (-0.012, 0.016), (0.015, 0.015)):
            self.add_segment(diff_base, diff_base + v3(dx * 2.2, dy * 2.2, 0.13), 0.0018,
                             rgba=(0.12, 0.10, 0.08, 1.0), kind="cylinder", spec=(0.2, 0.2, 0.2))
        # 3. 精装睡前书本
        bx, by, bz = -0.66, 1.68, 0.562
        self.add_box((0.075, 0.055, 0.005), (bx, by, bz), rgba=cover_col, spec=(0.2, 0.2, 0.25))
        self.add_box((0.070, 0.050, 0.012), (bx, by, bz + 0.008), rgba=page_col, spec=(0.1, 0.1, 0.1))

    def _build_bedroom_slippers(self):
        """构建床边地毯上自然摆放的一双毛绒居家拖鞋。"""
        fleece = (0.92, 0.88, 0.82, 1.0)
        sole = (0.42, 0.32, 0.24, 1.0)
        inner = (0.86, 0.81, 0.74, 1.0)
        for idx, (sx, sy, yaw) in enumerate([(-0.06, 1.32, 0.12), (0.06, 1.30, -0.08)]):
            q_s = p.getQuaternionFromEuler([0, 0, yaw])
            self.add_box((0.042, 0.110, 0.008), (sx, sy, 0.020), quat=q_s, rgba=sole, spec=(0.1, 0.08, 0.05))
            self.add_box((0.038, 0.100, 0.004), (sx, sy, 0.028), quat=q_s, rgba=inner, spec=(0.06, 0.06, 0.06))
            self.add_ellipsoid((0.046, 0.065, 0.032), (sx, sy - 0.035, 0.045), quat=q_s, rgba=fleece, spec=(0.08, 0.08, 0.08))

    def _build_bed_throw(self):
        """构建床尾折叠搭放的焦糖暖咖色针织盖毯。"""
        throw_col = (0.76, 0.58, 0.44, 1.0)
        self.add_box((0.82, 0.22, 0.028), (0.70, 0.52, 0.840), rgba=throw_col, spec=(0.06, 0.05, 0.04))
        self.add_box((0.78, 0.18, 0.022), (0.70, 0.52, 0.865), rgba=(0.80, 0.62, 0.48, 1.0), spec=(0.06, 0.05, 0.04))

    def _build_nightstand(self):
        """构建床头左侧的胡桃木双抽屉床头柜（含四条柜腿、柜体、顶板及两层黄铜拉手抽屉面）。"""
        walnut = (0.36, 0.23, 0.145, 1.0)
        brass = (0.72, 0.54, 0.28, 1.0)
        x0, x1 = -1.00, -0.31
        y0, y1 = 1.62, 2.24
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        for xx in (x0 + 0.05, x1 - 0.05):
            for yy in (y0 + 0.05, y1 - 0.05):
                self.add_cylinder(0.025, 0.42, (xx, yy, 0.21),
                                rgba=walnut, spec=(0.25, 0.18, 0.12))
        self.add_box(((x1 - x0) / 2 - 0.02, 0.28, 0.20), (cx, cy, 0.44),
                    rgba=walnut, spec=(0.22, 0.16, 0.11))
        self.add_box(((x1 - x0) / 2 + 0.015, 0.31, 0.025), (cx, cy, 0.555),
                    rgba=(0.42, 0.29, 0.19, 1.0), spec=(0.24, 0.18, 0.12))
        for zz in (0.38, 0.50):
            self.add_box(((x1 - x0) / 2 - 0.06, 0.012, 0.047),
                        (cx, y0 - 0.012, zz),
                        rgba=(0.40, 0.27, 0.17, 1.0), spec=(0.24, 0.18, 0.12))
            self.add_ellipsoid((0.016, 0.012, 0.016), (cx, y0 - 0.028, zz),
                            rgba=brass, spec=(0.7, 0.55, 0.3))

    def _build_warm_nightlight(self):
        """构建床头柜左后角的暖黄夜读台灯（保存灯泡与灯罩刚体 ID，用于助眠曲响起后平滑变暗）。"""
        base_pos = v3(-0.90, 2.02, 0.58)
        self.add_cylinder(0.052, 0.04, base_pos + v3(0, 0, 0.02),
                        rgba=(0.38, 0.24, 0.15, 1.0), spec=(0.3, 0.2, 0.1))
        self.add_cylinder(0.014, 0.14, base_pos + v3(0, 0, 0.09),
                        rgba=(0.72, 0.54, 0.28, 1.0), spec=(0.7, 0.55, 0.3))
        self.nightlamp_bulb = self.add_ellipsoid((0.048, 0.048, 0.055), base_pos + v3(0, 0, 0.165),
                        rgba=(1.0, 0.85, 0.50, 1.0), spec=(1.0, 0.9, 0.6))
        self.nightlamp_shade = self.add_cylinder(0.095, 0.13, base_pos + v3(0, 0, 0.175),
                        rgba=(0.99, 0.88, 0.65, 0.94), spec=(0.3, 0.25, 0.15))

    # -----------------------------------------------------------------------
    # 睡眠模式全屋动态调光系统
    # -----------------------------------------------------------------------
    def _apply_lighting(self, dim: float):
        """根据暗化系数 dim ∈ [0.0, 1.0] 平滑调节全卧室环境光照（0.0=睡前常亮, 1.0=深度睡眠微光）。"""
        if p is None or not p.isConnected():
            return
        dim = float(np.clip(dim, 0.0, 1.0))
        self.current_dim_factor = dim

        # 收集单独定制调光曲线的特殊发光体 ID，避免与普通漫反射物体重复处理
        special_bodies = set()
        if hasattr(self, "nightlamp_bulb") and self.nightlamp_bulb is not None:
            special_bodies.add(self.nightlamp_bulb)
        if hasattr(self, "nightlamp_shade") and self.nightlamp_shade is not None:
            special_bodies.add(self.nightlamp_shade)
        if hasattr(self, "bed_string_lights") and self.bed_string_lights:
            special_bodies.update(self.bed_string_lights)
        if "clock_screen" in self.bodies and self.bodies["clock_screen"] is not None:
            special_bodies.add(self.bodies["clock_screen"])
        if hasattr(self, "speech_body") and self.speech_body is not None:
            special_bodies.add(self.speech_body)
        if hasattr(self, "speech_tail") and self.speech_tail is not None:
            special_bodies.add(self.speech_tail)
        if hasattr(self, "phone_screen_glow") and self.phone_screen_glow is not None:
            special_bodies.add(self.phone_screen_glow)

        # 1. 全屋普通物体（地板、墙面、窗帘、床品、人物、机器人等）亮度从 100% 平滑降至 22%，并略带暖琥珀夜色偏移
        dark_factor = 1.0 - 0.78 * dim
        amber_shift = [1.03, 0.95, 0.84]

        for item in getattr(self, "scene_ambient_bodies", []):
            body = item["body"]
            if body in special_bodies:
                continue
            base_rgba = item["base_rgba"]
            cur_rgba = [
                base_rgba[0] * dark_factor * amber_shift[0],
                base_rgba[1] * dark_factor * amber_shift[1],
                base_rgba[2] * dark_factor * amber_shift[2],
                base_rgba[3]
            ]
            base_spec = item.get("base_spec", [0.2, 0.2, 0.2])
            cur_spec = [s * (1.0 - 0.85 * dim) for s in base_spec]
            p.changeVisualShape(body, -1, rgbaColor=cur_rgba, specularColor=cur_spec)

        # 2. 床头暖色台灯：灯泡与百褶灯罩由明亮暖黄渐变为微弱深邃的琥珀睡眠伴睡夜灯
        if hasattr(self, "nightlamp_bulb") and self.nightlamp_bulb is not None:
            bulb_rgba = lerp([1.0, 0.85, 0.50, 1.0], [0.20, 0.12, 0.04, 1.0], dim)
            bulb_spec = lerp([1.0, 0.9, 0.6], [0.12, 0.08, 0.03], dim)
            p.changeVisualShape(self.nightlamp_bulb, -1, rgbaColor=list(bulb_rgba), specularColor=list(bulb_spec))

        if hasattr(self, "nightlamp_shade") and self.nightlamp_shade is not None:
            shade_rgba = lerp([0.99, 0.88, 0.65, 0.94], [0.16, 0.11, 0.06, 0.94], dim)
            shade_spec = lerp([0.3, 0.25, 0.15], [0.03, 0.02, 0.01], dim)
            p.changeVisualShape(self.nightlamp_shade, -1, rgbaColor=list(shade_rgba), specularColor=list(shade_spec))

        # 3. 床头 15 盏星光氛围灯串：由明亮暖光淡化为微弱萤火
        if hasattr(self, "bed_string_lights") and self.bed_string_lights:
            fairy_rgba = lerp([1.0, 0.82, 0.48, 1.0], [0.14, 0.08, 0.02, 1.0], dim)
            fairy_spec = lerp([1.0, 0.9, 0.6], [0.06, 0.04, 0.01], dim)
            for light_body in self.bed_string_lights:
                p.changeVisualShape(light_body, -1, rgbaColor=list(fairy_rgba), specularColor=list(fairy_spec))

        # 4. 智能电子闹钟屏幕：自动进入深色免打扰护眼夜显模式
        if "clock_screen" in self.bodies and self.bodies["clock_screen"] is not None:
            clock_rgba = lerp([1.0, 1.0, 1.0, 1.0], [0.20, 0.25, 0.35, 1.0], dim)
            p.changeVisualShape(self.bodies["clock_screen"], -1, rgbaColor=list(clock_rgba))

        # 5. 全局阴影强度与渲染器背景底色同步沉降为深邃夜色
        cur_shadow = float(lerp(0.32, 0.03, dim))
        bg_color = list(lerp([0.65, 0.65, 0.68], [0.02, 0.02, 0.03], dim))
        p.configureDebugVisualizer(shadowMapIntensity=cur_shadow, rgbBackground=bg_color)

    # -----------------------------------------------------------------------
    # 核心交互角色与道具构建（手机屏幕辉光、电子闹钟、机械臂及抓取手指、女主角、气泡字幕）
    # -----------------------------------------------------------------------
    def _build_phone_face_glow(self):
        """构建贴附在手机屏幕正前方的半透明冷白辉光面片（表现深夜刷手机的屏幕发光感）。"""
        w = self.woman
        if w is None:
            return
        self.phone_screen_glow = self.add_card(
            0.084, 0.162,
            w.phone_base_pos + w.phone_front * 0.0065,
            normal=w.phone_front, up_hint=w.phone_up,
            rgba=(0.82, 0.93, 1.0, 0.16), spec=(0.9, 0.95, 1.0)
        )

    def _build_clock(self):
        """构建床头柜前侧的智能电子闹钟（适配 30° 俯拍视角微调屏幕仰角，使数字清晰正对镜头）。"""
        self.clock_center = v3(-0.82, 1.700, 0.725)
        # 法线向上仰起（z=0.45），使显示屏正对上方 30° 俯拍镜头
        self.clock_forward = norm(v3(-0.24, -0.86, 0.45))
        q = frame_quat(self.clock_forward, (0, 0, 1))
        if "clock_panel" in self.meshes and self.meshes["clock_panel"]:
            vis = p.createVisualShape(
                p.GEOM_MESH, fileName=self.meshes["clock_panel"],
                meshScale=[0.270, 0.155, 0.10], rgbaColor=[0.92, 0.88, 0.80, 1],
                specularColor=[0.25, 0.22, 0.18])
        else:
            vis = p.createVisualShape(
                p.GEOM_BOX, halfExtents=[0.135, 0.077, 0.05],
                rgbaColor=[0.92, 0.88, 0.80, 1], specularColor=[0.25, 0.22, 0.18])
        self.bodies["clock_body"] = self.add_visual(vis)
        p.resetBasePositionAndOrientation(
            self.bodies["clock_body"],
            list(self.clock_center - self.clock_forward * 0.032), q)
        self.add_card(0.256, 0.139,
                    self.clock_center + self.clock_forward * 0.024,
                    normal=self.clock_forward, up_hint=(0, 0, 1),
                    rgba=(0.035, 0.035, 0.045, 1.0), spec=(0.2, 0.2, 0.2))
        self.bodies["clock_screen"] = self.add_card(
            0.244, 0.124,
            self.clock_center + self.clock_forward * 0.026,
            normal=self.clock_forward, up_hint=(0, 0, 1),
            tex=self.load_tex("clock_11_59_40"), spec=(0.05, 0.05, 0.05))
        self.add_box((0.115, 0.060, 0.014), (-0.82, 1.710, 0.568),
                    rgba=(0.12, 0.11, 0.105, 1.0))
        self.add_box((0.060, 0.050, 0.060), (-0.82, 1.730, 0.610),
                    rgba=(0.18, 0.17, 0.16, 1.0))
        self.clock_time = "11:59:40"

    def set_clock(self, time_str, force=False):
        """切换电子钟数字纹理贴图（支持 force 强制刷新纹理）。"""
        if not force and time_str == self.clock_time:
            return
        self.clock_time = time_str
        key = "clock_" + time_str.replace(":", "_")
        if key in self.tex and self.tex[key] is not None and "clock_screen" in self.bodies:
            self.change_tex(self.bodies["clock_screen"], self.tex[key])

    def _build_robot(self):
        """实例化床头守护机器人（RobotArm），并在末端夹爪两侧构建【核心优化：多指节仿生包络抓取手指】。"""
        if RobotArm is not None:
            self.robot = RobotArm(self)
            # ==================== 【核心优化：构建双侧多指节仿生包络机械抓指】 ====================
            # 为使机器人抢夺手机时展现出极其清晰、有力的“张爪 -> 下探包络 -> 弯曲扣紧抓取”动作，
            # 在夹爪左右两侧各装配由“近端指节（proximal）+ 指间关节轴（knuckle）+ 远端内扣防滑指尖（distal）”组成的机械手指。
            self.gripper_claws = []
            claw_ivory = (0.94, 0.91, 0.86, 1.0)    # 象牙白机械指节外壳色
            claw_joint = (0.28, 0.30, 0.34, 1.0)    # 钛灰金属指间铰链色
            claw_pad = (0.16, 0.18, 0.22, 1.0)      # 深灰高摩擦防滑硅胶指垫色
            for side in (-1.0, 1.0):                # side = -1.0 为左抓指，+1.0 为右抓指
                prox_body = self.add_capsule(0.0065, 0.032, (0, 0, -100),
                                             rgba=claw_ivory, spec=(0.45, 0.45, 0.48))
                knuckle_body = self.add_sphere(0.0075, (0, 0, -100),
                                               rgba=claw_joint, spec=(0.6, 0.6, 0.65))
                dist_body = self.add_capsule(0.0058, 0.028, (0, 0, -100),
                                             rgba=claw_pad, spec=(0.25, 0.25, 0.30))
                self.gripper_claws.append({
                    "side": side,
                    "prox_body": prox_body,
                    "knuckle_body": knuckle_body,
                    "dist_body": dist_body,
                })
            self.robot.set_pose(self.robot.home_tcp, self.robot.home_approach,
                                jaw_open=0.032)
            self._update_gripper_claws(self.robot.home_tcp, self.robot.home_approach,
                                       jaw_open=0.032, grasp_curl=0.25)
        else:
            self.robot = None
            self.gripper_claws = []

    def _update_gripper_claws(self, tcp, approach, jaw_open: float, grasp_curl: float = 0.0):
        """实时解算并更新末端双侧仿生机械抓指的三维位姿：
        参数：
          tcp: 机械臂末端中心点坐标
          approach: 机械臂末端逼近方向向量
          jaw_open: 夹爪开合间距（越大则双爪向外张开越宽）
          grasp_curl: 远端指尖内扣弯曲度 ∈ [0.0, 1.0]（0.0=张开微曲预备态, 1.0=向内用力合拢扣死手机两侧）
        """
        if not getattr(self, "gripper_claws", None) or p is None or not p.isConnected():
            return
        tcp = np.asarray(tcp, dtype=float)
        fwd = norm(approach)
        # 构造夹爪局部正交坐标系：fwd 为手指延伸前向，side_ax 为左右开合轴，up_ax 为法向
        up_ref = v3(0.0, 0.0, 1.0)
        if abs(np.dot(fwd, up_ref)) > 0.95:
            up_ref = v3(1.0, 0.0, 0.0)
        side_ax = norm(np.cross(fwd, up_ref))
        up_ax = norm(np.cross(side_ax, fwd))

        grasp_curl = float(np.clip(grasp_curl, 0.0, 1.0))
        # 近端指节外张角：当 jaw_open 张大时向外舒展（+24°），当收紧抓取时向内聚拢（-10°）
        open_ratio = float(np.clip((jaw_open - 0.008) / (0.068 - 0.008), 0.0, 1.0))
        flare_ang = math.radians(lerp(-10.0, 24.0, open_ratio))
        # 远端指尖内扣角：随 grasp_curl 从 12° 弯曲至 48°，形成环抱扣紧手机两侧边框的抓取手型
        curl_ang = math.radians(lerp(12.0, 48.0, grasp_curl))

        for claw in self.gripper_claws:
            s_dir = claw["side"]  # -1.0 或 +1.0
            # 指根锚点随 jaw_open 左右平移
            base_pt = tcp - fwd * 0.018 + side_ax * (s_dir * (jaw_open * 0.85 + 0.012))
            # 近端指节轴向与末端指间关节位置
            prox_dir = norm(fwd * math.cos(flare_ang) + side_ax * (s_dir * math.sin(flare_ang)))
            knuckle_pt = base_pt + prox_dir * 0.034
            prox_mid = (base_pt + knuckle_pt) * 0.5
            q_prox = quat_from_z(prox_dir, up_hint=up_ax)

            # 远端指尖向夹爪中心线内扣，包络抓取手机侧边
            dist_dir = norm(fwd * math.cos(curl_ang) - side_ax * (s_dir * math.sin(curl_ang)))
            tip_pt = knuckle_pt + dist_dir * 0.030
            dist_mid = (knuckle_pt + tip_pt) * 0.5
            q_dist = quat_from_z(dist_dir, up_hint=up_ax)

            p.resetBasePositionAndOrientation(claw["prox_body"], list(prox_mid), list(q_prox))
            p.resetBasePositionAndOrientation(claw["knuckle_body"], list(knuckle_pt), [0, 0, 0, 1])
            p.resetBasePositionAndOrientation(claw["dist_body"], list(dist_mid), list(q_dist))

    def _build_woman(self):
        """实例化床上女主角模型（Woman），并初始化为 0.0s 靠坐刷手机姿态。"""
        if Woman is not None:
            self.woman = Woman(self)
            self.woman.idle(0.0)
        else:
            self.woman = None

    def _build_speech(self):
        """构建机器人后上方的圆角气泡字幕面板，默认加载第 1 句台词贴图（“不早了”）。"""
        self.speech_tex_keys = {
            1: "speech1", 2: "speech2", 3: "speech3", 4: "speech4",
        }
        self.speech_current = 1
        self.speech_visible = True
        # ==================== 【核心优化：气泡字幕高置防遮挡坐标】 ====================
        # 在 30° 俯拍视角下，将字幕置于机器人后方高处 (x=-0.48, y=2.22, z=1.46)，
        # 使其在屏幕投影中位于机器人与机械臂的正上方墙面净空区，绝不遮挡机器人机身！
        self.speech_pos = v3(-0.48, 2.22, 1.46)
        self.speech_time = 0.0
        panel = self.meshes.get("speech_panel", "")
        if panel and os.path.exists(panel):
            vis = p.createVisualShape(
                p.GEOM_MESH, fileName=panel, meshScale=[0.76, 0.76, 0.76],
                rgbaColor=[1, 1, 1, 1], specularColor=[0.12, 0.12, 0.12],
                flags=p.VISUAL_SHAPE_DOUBLE_SIDED)
        else:
            vis = p.createVisualShape(
                p.GEOM_BOX, halfExtents=[0.38, 0.095, 0.003],
                rgbaColor=[1, 1, 1, 1], specularColor=[0.12, 0.12, 0.12])
        self.speech_body = self.add_visual(vis, self.speech_pos)
        self.change_tex(self.speech_body, self.tex.get("speech1", None))
        self.speech_q = (0, 0, 0, 1)
        self.update_camera_billboards()

    def set_speech(self, index, force=False):
        """显示指定编号（1~4）的气泡字幕并切换对应文字贴图；若传入无效编号则隐藏字幕。"""
        if index not in self.speech_tex_keys:
            self.speech_visible = False
            return
        self.speech_visible = True
        if force or index != self.speech_current:
            self.speech_current = index
            tex_val = self.tex.get(self.speech_tex_keys[index], None)
            if tex_val is not None and hasattr(self, "speech_body"):
                self.change_tex(self.speech_body, tex_val)

    def hide_speech(self):
        """将字幕可见性标志置为 False，使其在下一帧更新时从画面中消失。"""
        self.speech_visible = False

    def update_camera_billboards(self):
        """每帧更新字幕面板位姿：
        - 若 `speech_visible` 为 False，将字幕面板移至地下 [0, 0, -100] 彻底隐藏；
        - 若 `speech_visible` 为 True，施加正弦上下微幅呼吸浮动，并计算朝向当前 30° 俯拍摄像机视点的旋转四元数。
        """
        if not hasattr(self, "speech_body"):
            return
        if not getattr(self, "speech_visible", False):
            p.resetBasePositionAndOrientation(
                self.speech_body, [0, 0, -100], [0, 0, 0, 1])
            return
        try:
            cam = p.getDebugVisualizerCamera()
            eye = camera_eye_from_view(cam[2])
        except Exception:
            eye = np.array([-0.87, -0.42, 2.36])
        bob = math.sin(self.speech_time * 1.6) * 0.008
        pos = self.speech_pos + v3(0, 0, bob)
        # 计算放大 1.5 倍后的真实世界坐标以精确对准 30° 俯拍摄像机视线
        world_pos = self._scale_subject_pos(pos)
        q = look_at_quat(world_pos, eye)
        p.resetBasePositionAndOrientation(self.speech_body, list(pos), list(q))
        self.speech_q = q

    # -----------------------------------------------------------------------
    # 【核心优化：1:1 国际标准时钟跳秒 + 机器人 2.0 秒说话间隔 + 真实抓取与拉被子时间轴】
    # -----------------------------------------------------------------------
    # 机器人每次说话之间的固定静默间隔（单位：国际标准秒）
    SPEECH_INTERVAL = 2.0

    # 阶段 1：闹钟从 11:59:40 严格按 1秒/格 跳动至 11:59:50 (0.0s ~ 10.0s)
    T_REMIND1_END = 8.0           # 第1句“不早了”显示结束时刻（预留 2.0 秒说话间隔至 10.0s）
    T_REMIND2 = 10.0              # 11:59:50（第 10.0 标准秒）机器人播报第2句："该睡觉了"
    T_REMIND2_END = 18.0          # 第2句“该睡觉了”显示结束时刻（预留 2.0 秒间隔至 20.0s 零点）

    # 阶段 2：闹钟抵达 12:00:00 (20.0s)，机械臂执行“预抓取张爪 -> 下探包络 -> 夹紧抓取 -> 拔起收纳”
    T_REACH_START = 20.0          # 12:00:00（第 20.0 标准秒）零点到达，机械臂出动并大幅张开双爪
    T_PRE_GRASP_END = 21.40       # 机械臂抵达手机斜上方预抓取悬停点，双爪完全张开至 0.068 对准手机
    T_REACH_END = 21.95           # 机械臂保持张爪姿态精准下探，使双侧抓指深入包络在手机机身左右两侧
    T_ATTACH = 22.45              # 双爪与指尖在 0.50s 内有力收拢夹紧（0.068 -> 0.008），完成真实抓取动作！
    T_PULL_END = 23.85            # 机械臂抓牢手机平稳拔起抽离至空中过渡高点
    T_STOW_END = 25.35            # 机械臂将手机运送至机器人底座侧面储物抽屉上方
    T_INSERT_END = 26.00          # 手机安放入储物抽屉，抓指松开并向上抬起退出
    T_HATCH_CLOSE_START = 26.0    # 机器人侧面储物抽屉开始平滑关闭
    T_HATCH_CLOSE_END = 26.7      # 储物抽屉完全闭合锁紧

    # 阶段 3：第3句与第4句台词之间严格相隔 2.0 秒（SPEECH_INTERVAL = 2.0s），说完第4句字幕消失
    T_SPEECH3 = 26.8              # 机器人开始播报第3句："手机将于次日早上6点整归还"
    T_SPEECH3_END = 30.0          # 第3句台词播报结束，进入 2.0 秒说话静默间隔 (30.0s ~ 32.0s)
    T_SPEECH4 = 32.0              # 间隔 2.0 秒后（30.0 + 2.0 = 32.0s），机器人播报第4句："将为您播放助眠曲"
    T_SPEECH4_END = 34.5          # 第4句“将为您播放助眠曲”播报完毕，气泡字幕立即随之消失！

    # 阶段 4：女主角伸手抓被沿 -> 平躺同时将被子拉起盖住上半身 -> 掖被角入睡 -> 助眠曲响起、灯光渐暗、窗帘拉拢
    T_LIE_DOWN_START = 34.5       # 第4句播报完毕瞬间，女主角开始伸手抓握腿部被沿
    T_QUILT_GRAB_END = 35.25      # 女主角双手握紧被头反折边，准备平躺拉起被子
    T_QUILT_PULL_END = 37.25      # 女主角平躺落于枕芯，同时双手将被子拉起至肩颈处完全盖住上半身
    T_LIE_DOWN_END = 37.85        # 女主角松开被角并将双手安放于被窝内，彻底安详入睡
    T_LULLABY = 38.0              # 助眠曲正式响起，同时卧室灯光渐暗、右侧窗帘同步拉拢

    def _eye_now(self):
        """获取当前帧摄像机在世界坐标系下的实时视点位置。"""
        try:
            cam = p.getDebugVisualizerCamera()
            return camera_eye_from_view(cam[2])
        except Exception:
            return np.array([-0.87, -0.42, 2.36])

    def reset_story(self):
        """将整个小剧场重置回 t = 0.0s 的初始状态（供启动或按 R 键重播调用）。"""
        stop_lullaby()
        clear_speech_queue()
        self.story_t = 0.0
        self.story_paused = False
        self._start_delay = 1.0       # 开场预留 1.0 秒缓冲，确保窗口稳定呈现后再推进计时
        self.lullaby_on = False
        self._lullaby_started = False
        self._logged = set()
        self._speech_idx = None
        self.phone_inserted = False
        self.set_clock("11:59:40", force=True)    # 强制刷新闹钟贴图为 11:59:40
        self.set_speech(1, force=True)            # 强制刷新字幕贴图为第 1 句“不早了”
        self.current_dim_factor = 0.0
        self.current_curtain_close = 0.0
        self.current_duvet_cover = 0.0
        self._apply_lighting(0.0)     # 恢复全屋常亮温馨光照
        self._update_curtains(0.0)    # 恢复窗帘完全敞开状态
        self._update_duvet(0.0)       # 恢复被子初始状态（叠放在腿部，未拉起盖住上半身）
        w = self.woman
        if w is not None:
            w.set_expression("indifferent")
            w.idle(0.0)
        if self.robot is not None:
            self.robot.set_open(0.0)
            self.robot.jaw_open = 0.032
            self.robot.set_pose(self.robot.home_tcp, self.robot.home_approach, jaw_open=0.032)
            self._update_gripper_claws(self.robot.home_tcp, self.robot.home_approach,
                                       jaw_open=0.032, grasp_curl=0.25)
        self.speech_time = 0.0
        self.update_camera_billboards()

    def update_story(self, dt):
        """按帧时间步长 dt 推进剧情时间 story_t 并应用当前帧状态。"""
        if self.story_paused:
            return
        if getattr(self, "_start_delay", 0.0) > 0.0:
            self._start_delay -= max(0.0, dt)
            return
        self.story_t += max(0.0, dt)
        self._apply_story(self.story_t)

    def set_story_time(self, t):
        """直接跳转到指定剧情时间点 t（用于离线批量渲染关键帧）。"""
        self.story_t = float(t)
        self._apply_story(self.story_t)

    def _log_once(self, key, message):
        """确保每个剧情节点提示日志在控制台只打印一次。"""
        if key not in self._logged:
            self._logged.add(key)
            if not self.quiet:
                print(message)

    def _apply_story(self, t):
        """核心时间线状态机：根据当前剧情时间 t 同步驱动时钟、字幕、女主角、被子、机械臂、抓指、手机、灯光、窗帘与音乐。"""
        w = self.woman
        robot = self.robot
        self.speech_time = t

        # -------------------------------------------------------------------
        # 1. 【核心优化：床头智能电子钟按 1:1 国际标准秒（SI Second）跳动】
        # 每流逝 1.0 秒真实时间，电子钟秒数严格 +1（0.0s~20.0s 对应 11:59:40 -> 12:00:00）
        # -------------------------------------------------------------------
        sim_sec = 40 + int(min(t, self.T_REACH_START) / 1.0)
        clock_text = "12:00:00" if sim_sec >= 60 else "11:59:%02d" % sim_sec
        self.set_clock(clock_text)

        # -------------------------------------------------------------------
        # 2. 【核心优化：机器人说话间隔设为 2 秒 + 播报完第4句后字幕随之消失】
        # -------------------------------------------------------------------
        if t < self.T_REMIND1_END:
            # 阶段 A (0.0s ~ 8.0s)：显示并播报第 1 句字幕“不早了”
            self._set_speech_index(1)
        elif t < self.T_REMIND2:
            # 间隔 1 (8.0s ~ 10.0s)：第 1 句与第 2 句之间的 2.0 秒说话静默间隔
            self._set_speech_index(None)
        elif t < self.T_REMIND2_END:
            # 阶段 B (10.0s ~ 18.0s)：11:59:50 显示并播报第 2 句字幕“该睡觉了”
            self._set_speech_index(2)
            self._log_once("remind2", "闹钟 11:59:50 —— 机器人：该睡觉了")
        elif t < self.T_SPEECH3:
            # 间隔 2 及夺机收纳阶段 (18.0s ~ 26.8s)：保持字幕隐藏
            self._set_speech_index(None)
        elif t < self.T_SPEECH3_END:
            # 阶段 C (26.8s ~ 30.0s)：显示并播报第 3 句字幕“手机将于次日早上6点整归还”
            self._set_speech_index(3)
            self._log_once("speech3", "机器人：手机将于次日早上6点整归还")
        elif t < self.T_SPEECH4:
            # 间隔 3 (30.0s ~ 32.0s)：第 3 句与第 4 句之间严格 2.0 秒说话间隔（SPEECH_INTERVAL = 2.0s）
            self._set_speech_index(None)
        elif t < self.T_SPEECH4_END:
            # 阶段 D (32.0s ~ 34.5s)：显示并播报第 4 句字幕“将为您播放助眠曲”
            self._set_speech_index(4)
            self._log_once("speech4", "机器人：将为您播放助眠曲")
        else:
            # 阶段 E (t >= 34.5s)：机器人说完“将为您播放助眠曲”后，立即传入 None 使字幕面板消失！
            self._set_speech_index(None)
            self._log_once("speech4_hide", "机器人播报完‘将为您播放助眠曲’，气泡字幕随之自动消失")

        camera_eye = self._eye_now()

        # -------------------------------------------------------------------
        # 3. 女主角体态、神情与【核心优化：躺下睡觉时双手将被子拉起盖住上半身】控制
        # -------------------------------------------------------------------
        if w is not None:
            if t < self.T_REACH_END:
                # 阶段 1A (t < 21.95s)：60° 仰角靠坐床头刷手机，被子保持在腿部初始位 (cover_ratio = 0.0)
                self._update_duvet(0.0)
                w.idle(t)
                w.set_expression("indifferent")
                self._log_once("remind1", "闹钟 11:59:40 —— 机器人：不早了")
                if hasattr(self, "phone_screen_glow"):
                    self.set_pose(
                        self.phone_screen_glow,
                        w.phone_pos + w.phone_front * 0.0065,
                        w.phone_q
                    )

            elif t < self.T_ATTACH:
                # 阶段 1B (21.95s ~ 22.45s)：机械臂双爪正在收拢抓紧手机！
                # 女主角保持持机位置定格，但双手手指随机械爪收紧产生被动松开反馈，强化真实抓取夺机感
                self._update_duvet(0.0)
                w.idle(self.T_REACH_END)
                u_clamp = (t - self.T_REACH_END) / (self.T_ATTACH - self.T_REACH_END)
                k_clamp = quintic_ease(u_clamp)
                w.right_curl = lerp(w.right_curl, 0.18, k_clamp)
                w.left_curl = lerp(w.left_curl, 0.12, k_clamp)
                w.update_arms()
                w.set_expression("sad" if u_clamp > 0.65 else "indifferent")
                if hasattr(self, "phone_screen_glow"):
                    self.set_pose(
                        self.phone_screen_glow,
                        w.phone_pos + w.phone_front * 0.0065,
                        w.phone_q
                    )

            elif t < self.T_LIE_DOWN_START:
                # 阶段 2 (22.45s ~ 34.5s)：手机被机器人抓走后至第 4 句台词播报完毕前
                # 关闭手机屏幕面部辉光，被子仍在腿部，女主角切换为委屈失落表情，双手在被抓瞬间微抬挽留后垂落腹前
                self._update_duvet(0.0)
                if hasattr(self, "phone_screen_glow"):
                    self.set_pose(self.phone_screen_glow, (0, 0, -100), (0, 0, 0, 1))

                w.set_recline(0.0, t, gaze_state="sad")
                w.set_expression("sad")

                # 手机刚被抓走的前 0.6s 内，双手从持机高度微抬后自然垂落到腹前
                settle_k = quintic_ease(min(1.0, (t - self.T_ATTACH) / 0.65))
                w.right_wrist = lerp(v3(0.37, 1.25, 1.03), v3(0.40, 1.30, 0.92), settle_k)
                w.right_hand_dir = norm(v3(-0.25, -0.90, -0.15))
                w.right_palm_normal = v3(0.0, 0.0, 1.0)
                w.right_curl = lerp(0.18, 0.35, settle_k)
                w.right_thumb_curl = lerp(0.20, 0.30, settle_k)

                w.left_wrist = lerp(v3(0.23, 1.24, 1.01), v3(0.24, 1.28, 0.92), settle_k)
                w.left_hand_dir = norm(v3(-0.14, -0.94, -0.10))
                w.left_palm_normal = v3(0.0, 0.0, 1.0)
                w.left_curl = 0.12
                w.left_thumb_curl = 0.16
                w.update_arms()

            else:
                # ==================== 【核心优化：人物躺下睡觉时双手将被子拉起盖住上半身】 ====================
                if hasattr(self, "phone_screen_glow"):
                    self.set_pose(self.phone_screen_glow, (0, 0, -100), (0, 0, 0, 1))

                # 子阶段 4A (34.50s ~ 35.25s)：双手向前下方探出，五指弯曲握住腿部被头反折边 (y ≈ 1.17)
                if t < self.T_QUILT_GRAB_END:
                    u_grab = (t - self.T_LIE_DOWN_START) / (self.T_QUILT_GRAB_END - self.T_LIE_DOWN_START)
                    k_grab = quintic_ease(u_grab)
                    # 上半身微倾放松准备躺下 (recline: 0.0 -> 0.16)，被子边缘被双手微微提起 (cover: 0.0 -> 0.06)
                    w.set_recline(0.16 * k_grab, t, gaze_state="sleep")
                    w.set_expression("sad")
                    self._update_duvet(0.06 * k_grab)

                    # 双手从腹前伸向被头边缘 (y=1.17, z=0.905)，手掌朝下扣紧被沿，手指握拢 (curl -> 0.82)
                    w.right_wrist = lerp(v3(0.40, 1.30, 0.92), v3(0.42, 1.17, 0.905), k_grab)
                    w.right_hand_dir = norm(lerp(v3(-0.25, -0.90, -0.15), v3(-0.12, -0.95, -0.22), k_grab))
                    w.right_palm_normal = norm(lerp(v3(0.0, 0.0, 1.0), v3(0.0, 0.30, -0.95), k_grab))
                    w.right_curl = lerp(0.35, 0.82, k_grab)
                    w.right_thumb_curl = lerp(0.30, 0.75, k_grab)

                    w.left_wrist = lerp(v3(0.24, 1.28, 0.92), v3(0.16, 1.17, 0.905), k_grab)
                    w.left_hand_dir = norm(lerp(v3(-0.14, -0.94, -0.10), v3(0.10, -0.95, -0.22), k_grab))
                    w.left_palm_normal = norm(lerp(v3(0.0, 0.0, 1.0), v3(0.0, 0.30, -0.95), k_grab))
                    w.left_curl = lerp(0.12, 0.82, k_grab)
                    w.left_thumb_curl = lerp(0.16, 0.75, k_grab)
                    w.update_arms()
                    self._log_once("grab_quilt", "女生伸出双手握住被沿，准备躺下盖被子")

                # 子阶段 4B (35.25s ~ 37.25s)：平躺落枕的同时，双手紧握被头将被子一路拉起盖住整个上半身！
                elif t < self.T_QUILT_PULL_END:
                    u_pull = (t - self.T_QUILT_GRAB_END) / (self.T_QUILT_PULL_END - self.T_QUILT_GRAB_END)
                    k_pull = quintic_ease(u_pull)
                    recline_val = lerp(0.16, 1.0, k_pull)
                    cover_val = lerp(0.06, 1.0, k_pull)

                    w.set_recline(recline_val, t, gaze_state="sleep")
                    w.set_expression("sleep" if recline_val >= 0.45 else "sad")
                    # 实时同步拉起被子，使其从腿部一路延伸至肩颈锁骨处 (y=1.79)，严密盖住上半身！
                    self._update_duvet(cover_val)

                    # 双手紧握被头反折边，随被子同步向上拉至胸肩上方 (y: 1.17 -> 1.65, z: 0.905 -> 0.975)
                    arc_z = math.sin(math.pi * k_pull) * 0.025
                    w.right_wrist = lerp(v3(0.42, 1.17, 0.905), v3(0.42, 1.65, 0.975), k_pull) + v3(0, 0, arc_z)
                    w.right_hand_dir = norm(lerp(v3(-0.12, -0.95, -0.22), v3(-0.18, -0.88, 0.15), k_pull))
                    w.right_palm_normal = norm(v3(0.0, 0.35, -0.94))
                    w.right_curl = 0.82
                    w.right_thumb_curl = 0.75

                    w.left_wrist = lerp(v3(0.16, 1.17, 0.905), v3(0.16, 1.65, 0.975), k_pull) + v3(0, 0, arc_z)
                    w.left_hand_dir = norm(lerp(v3(0.10, -0.95, -0.22), v3(0.15, -0.88, 0.15), k_pull))
                    w.left_palm_normal = norm(v3(0.0, 0.35, -0.94))
                    w.left_curl = 0.82
                    w.left_thumb_curl = 0.75
                    w.update_arms()

                # 子阶段 4C (t >= 37.25s)：被子已完全盖住上半身，双手松开被角自然安放于被窝内侧熟睡
                else:
                    u_rest = min(1.0, (t - self.T_QUILT_PULL_END) / (self.T_LIE_DOWN_END - self.T_QUILT_PULL_END))
                    k_rest = quintic_ease(u_rest)
                    w.set_recline(1.0, t, gaze_state="sleep")
                    w.set_expression("sleep")
                    self._update_duvet(1.0)

                    # 双手手指松开，柔和滑入被窝内侧安放
                    w.right_wrist = lerp(v3(0.42, 1.65, 0.975), v3(0.44, 1.50, 0.84), k_rest)
                    w.right_hand_dir = norm(lerp(v3(-0.18, -0.88, 0.15), v3(-0.20, -0.96, -0.05), k_rest))
                    w.right_palm_normal = norm(lerp(v3(0.0, 0.35, -0.94), v3(0.0, 0.0, 1.0), k_rest))
                    w.right_curl = lerp(0.82, 0.15, k_rest)
                    w.right_thumb_curl = lerp(0.75, 0.12, k_rest)

                    w.left_wrist = lerp(v3(0.16, 1.65, 0.975), v3(0.16, 1.48, 0.84), k_rest)
                    w.left_hand_dir = norm(lerp(v3(0.15, -0.88, 0.15), v3(-0.10, -0.98, -0.05), k_rest))
                    w.left_palm_normal = norm(lerp(v3(0.0, 0.35, -0.94), v3(0.0, 0.0, 1.0), k_rest))
                    w.left_curl = lerp(0.82, 0.15, k_rest)
                    w.left_thumb_curl = lerp(0.75, 0.12, k_rest)
                    w.update_arms()

                    if t >= self.T_LIE_DOWN_END:
                        self._log_once("sleeping", "女生将被子拉起严密盖住上半身，闭眼安详入睡")

        # -------------------------------------------------------------------
        # 4. 【核心优化：守护机器人四阶段真实“抓取（Grasp）”动作与收纳轨迹控制】
        # -------------------------------------------------------------------
        if robot is not None and w is not None:
            home = robot.home_tcp                    # 机械臂初始待机末端坐标
            A_home = robot.home_approach             # 机械臂初始待机夹爪朝向
            # 抓取手机时的精准对准朝向：使夹爪正对手机机身，左右双指对称包络手机两侧边框
            A_grab = norm(w.phone_base_pos - (robot.base + v3(0.0, 0.0, 0.36)))
            # 手机机身抓取点（深入至手机中心偏上 +0.014m 处，使机械爪真正包住手机而非停在顶端外侧）
            grab_tcp = w.phone_base_pos + v3(0.0, 0.0, 0.014)
            # 手机前上方预抓取悬停点（张开双爪在此对准后再精准下探）
            pre_grab_tcp = grab_tcp - A_grab * 0.085 + v3(0.0, 0.0, 0.045)
            pull_tcp = grab_tcp + v3(-0.18, 0.10, 0.18)     # 手机拔起后的空中过渡高点
            store_tcp = robot.store_tcp              # 储物抽屉上方的投放点
            store_A = robot.store_approach           # 投放手机时的夹爪朝向
            retreat_tcp = store_tcp + v3(0.02, -0.06, 0.17) # 投放完成后的抬升撤离点
            t_retreat_done = self.T_INSERT_END + 1.5        # 机械臂完全归位时刻 (27.5s)

            # 机械臂待机时的微幅拟人化呼吸浮动偏移
            bob = v3(0.005 * math.sin(t * 1.15), 0.003 * math.sin(t * 0.8 + 0.5),
                    0.005 * math.sin(t * 1.7))

            # (1) 待机阶段 (t < 20.0s)：保持在床头柜上方轻微呼吸浮动，储物抽屉关闭
            if t < self.T_REACH_START:
                cur_tcp = home + bob
                cur_jaw = 0.030 + 0.002 * math.sin(t * 1.6)
                robot.set_pose(cur_tcp, A_home, jaw_open=cur_jaw)
                self._update_gripper_claws(cur_tcp, A_home, jaw_open=cur_jaw, grasp_curl=0.25)
                robot.set_open(0.0)

            # (2A) 伸臂预抓取与大幅张爪阶段 (20.0s -> 21.40s)：
            # 机械臂沿弧线伸至手机斜上方预抓取悬停点 pre_grab_tcp，同时双爪大幅张开至 0.068 对准手机！
            elif t < self.T_PRE_GRASP_END:
                u = (t - self.T_REACH_START) / (self.T_PRE_GRASP_END - self.T_REACH_START)
                k = quintic_ease(u)
                cp1 = home + v3(0.08, -0.05, 0.15)
                cp2 = pre_grab_tcp + v3(-0.06, -0.03, 0.08)
                tcp = bezier_curve(home, cp1, cp2, pre_grab_tcp, k)
                A_cur = slerp_vector(A_home, A_grab, k)
                jaw = float(lerp(0.030, 0.068, k))
                curl = float(lerp(0.25, 0.0, k))
                robot.set_pose(tcp, A_cur, jaw_open=jaw)
                self._update_gripper_claws(tcp, A_cur, jaw_open=jaw, grasp_curl=curl)
                robot.set_open(0.0)
                if t < self.T_REACH_START + 0.2:
                    self._log_once("grab", "12:00:00 —— 机器人伸出机械臂并大幅张开双爪，对准手机准备抓取")

            # (2B) 精准下探包络手机机身阶段 (21.40s -> 21.95s)：
            # 保持夹爪最大张开度 0.068，从预抓取点沿对准轴向深入下探至 grab_tcp，使左右手指完整包住手机两侧
            elif t < self.T_REACH_END:
                u = (t - self.T_PRE_GRASP_END) / (self.T_REACH_END - self.T_PRE_GRASP_END)
                k = quintic_ease(u)
                tcp = lerp(pre_grab_tcp, grab_tcp, k)
                jaw = 0.068
                curl = float(lerp(0.0, 0.18, k))
                robot.set_pose(tcp, A_grab, jaw_open=jaw)
                self._update_gripper_claws(tcp, A_grab, jaw_open=jaw, grasp_curl=curl)
                robot.set_open(0.0)

            # (3) 双爪收紧抓取锁死阶段 (21.95s -> 22.45s，耗时 0.50s)：
            # 双爪从大张状态 0.068 有力收拢闭合至 0.008，仿生指尖向内弯曲 (curl -> 1.0) 牢牢扣紧手机两侧边框！
            elif t < self.T_ATTACH:
                u = (t - self.T_REACH_END) / (self.T_ATTACH - self.T_REACH_END)
                k = quintic_ease(u)
                jaw = float(lerp(0.068, 0.008, k))
                curl = float(lerp(0.18, 1.0, k))
                # 在夹紧瞬间伴随微小的下压包裹与向上蓄力提拉感
                tug_z = -0.005 * math.sin(math.pi * u) + (0.008 * k if u > 0.6 else 0.0)
                tcp = grab_tcp + v3(0.0, 0.0, tug_z)
                robot.set_pose(tcp, A_grab, jaw_open=jaw)
                self._update_gripper_claws(tcp, A_grab, jaw_open=jaw, grasp_curl=curl)
                robot.set_open(0.0)
                if u > 0.5:
                    self._log_once("clamped", "机器人机械爪合拢收紧，牢牢抓取住手机边框")

            # (4) 连续五阶贝塞尔拔起抽离与入柜阶段 (22.45s -> 25.35s)：
            # 机械爪保持紧握姿态 (jaw_open=0.008, grasp_curl=1.0) 将手机拔起并送入侧面储物抽屉上方
            elif t < self.T_STOW_END:
                u = (t - self.T_ATTACH) / (self.T_STOW_END - self.T_ATTACH)
                k = quintic_ease(u)
                p0 = grab_tcp + v3(0.0, 0.0, 0.008)
                p1 = grab_tcp + v3(-0.06, 0.04, 0.15)
                p2 = pull_tcp + v3(-0.05, 0.03, 0.08)
                p3 = store_tcp + v3(0.05, -0.05, 0.12)
                p4 = store_tcp
                tcp = bezier5(p0, p1, p2, p3, p4, k)
                A_current = slerp_vector(A_grab, store_A, k)
                robot.set_pose(tcp, A_current, jaw_open=0.008)
                self._update_gripper_claws(tcp, A_current, jaw_open=0.008, grasp_curl=1.0)
                # 机器人侧面储物抽屉在手机抽离中途提前平滑滑开
                hatch = quintic_ease(min(1.0, max(0.0, (t - (self.T_ATTACH + 0.55)) / 1.2)))
                robot.set_open(hatch)
                if t < self.T_ATTACH + 0.15:
                    self._log_once("taken", "机器人抓牢手机顺滑抽离并送往储物格")

            # (5) 松爪释放手机与上浮撤离阶段 (25.35s -> 26.00s)：张开机械爪与指尖释放手机，沿弧线向上抬起
            elif t < self.T_INSERT_END:
                u = (t - self.T_STOW_END) / (self.T_INSERT_END - self.T_STOW_END)
                k = quintic_ease(u)
                release_k = quintic_ease(min(1.0, u / 0.6))
                jaw = float(lerp(0.008, 0.036, release_k))
                curl = float(lerp(1.0, 0.20, release_k))
                tcp = bezier_curve(store_tcp, store_tcp + v3(0, 0, 0.06), retreat_tcp + v3(0, 0, 0.05), retreat_tcp, k)
                robot.set_pose(tcp, store_A, jaw_open=jaw)
                self._update_gripper_claws(tcp, store_A, jaw_open=jaw, grasp_curl=curl)
                robot.set_open(1.0)

            # (6) 归位与抽屉关闭阶段 (t >= 26.00s)：机械臂返回待机位，储物抽屉缓缓合上
            else:
                ret_u = (t - self.T_INSERT_END) / (t_retreat_done - self.T_INSERT_END)
                ret_k = quintic_ease(min(1.0, ret_u))
                cp1 = retreat_tcp + v3(0.04, -0.02, 0.10)
                cp2 = home + v3(-0.06, 0.04, 0.12)
                tcp_target = bezier_curve(retreat_tcp, cp1, cp2, home, ret_k)
                A_current = slerp_vector(store_A, A_home, ret_k)
                close_u = (t - self.T_HATCH_CLOSE_START) / (self.T_HATCH_CLOSE_END - self.T_HATCH_CLOSE_START)
                close_hatch = 1.0 - quintic_ease(close_u) if t > self.T_HATCH_CLOSE_START else 1.0

                if t >= t_retreat_done:
                    settle = math.exp(-2.5 * (t - t_retreat_done)) * math.sin(6.0 * (t - t_retreat_done)) * 0.002
                    cur_tcp = home + bob * 0.5 + v3(0, 0, settle)
                    robot.set_pose(cur_tcp, A_home, jaw_open=0.030)
                    self._update_gripper_claws(cur_tcp, A_home, jaw_open=0.030, grasp_curl=0.25)
                else:
                    robot.set_pose(tcp_target, A_current, jaw_open=0.032)
                    self._update_gripper_claws(tcp_target, A_current, jaw_open=0.032, grasp_curl=0.25)
                robot.set_open(max(0.0, close_hatch))

            # ---------------------------------------------------------------
            # 5. 手机刚体跟随机械爪抓取、拔起与滑入抽屉托盘的位姿同步
            # ---------------------------------------------------------------
            if t < self.T_REACH_END:
                # t < 21.95s 时手机握在女主角手中，已由 w.idle(t) 更新
                pass
            elif t < self.T_ATTACH:
                # 21.95s ~ 22.45s：双爪收紧抓取过程中，手机被夹爪轻轻向爪心归中锁紧并随蓄力微抬
                u = (t - self.T_REACH_END) / (self.T_ATTACH - self.T_REACH_END)
                k = quintic_ease(u)
                clamped_pos = robot.tcp - v3(0, 0, 0.014)
                w.update_phone(lerp(w.phone_base_pos, clamped_pos, k), w.phone_base_q)
            elif t < self.T_STOW_END:
                # 22.45s ~ 25.35s：手机被机械爪牢牢抓在爪心，随 robot.tcp 同步移动并旋转至水平入盒姿态
                u = (t - self.T_ATTACH) / (self.T_STOW_END - self.T_ATTACH)
                held_pos = robot.tcp - v3(0, 0, 0.014)
                Rc = np.column_stack([v3(0, -1, 0), v3(1, 0, 0), v3(0, 0, 1)])
                q_tray = quat_from_matrix(Rc)
                phone_q = interp_quat(w.phone_base_q, q_tray, quintic_ease(min(1.0, u * 1.1)))
                w.update_phone(held_pos, phone_q)
            elif t < self.T_INSERT_END:
                # 25.35s ~ 26.00s：机械爪张开，手机平稳落入机器人侧面的抽出式储物托盘中央
                k = quintic_ease((t - self.T_STOW_END) / (self.T_INSERT_END - self.T_STOW_END))
                start_pos = robot.store_tcp - v3(0, 0, 0.014)
                start_q = frame_quat(norm(camera_eye - start_pos), (0, 0, 1))
                tray_pos = robot.base + v3(0.0, -0.190, 0.105)
                Rc = np.column_stack([v3(0, -1, 0), v3(1, 0, 0), v3(0, 0, 1)])
                w.update_phone(lerp(start_pos, tray_pos, k), interp_quat(start_q, quat_from_matrix(Rc), k))
            else:
                # t >= 26.00s：手机随储物托盘一同向内推入机器人机身内部锁好
                close_u = (t - self.T_HATCH_CLOSE_START) / (self.T_HATCH_CLOSE_END - self.T_HATCH_CLOSE_START)
                close_factor = quintic_ease(close_u) if t > self.T_HATCH_CLOSE_START else 0.0
                tray_pos = robot.base + v3(0.0, -0.190 + 0.13 * close_factor, 0.105)
                Rc = np.column_stack([v3(0, -1, 0), v3(1, 0, 0), v3(0, 0, 1)])
                w.update_phone(tray_pos, quat_from_matrix(Rc))

            # 更新女主角自然周期性眨眼动画
            w.update_blink(t)

        # -------------------------------------------------------------------
        # 6. 【核心优化：播放助眠曲后（t >= 38.0s），灯光暗下来的同时将窗帘拉起来】
        # -------------------------------------------------------------------
        if t < self.T_LULLABY:
            dim = 0.0
        else:
            # 在 t = 38.0s ~ 41.0s 的 3.0 秒内，通过五次多项式曲线将 dim 从 0.0 平滑推进至 1.0
            u = (t - self.T_LULLABY) / 3.0
            dim = quintic_ease(min(1.0, u))

        # 当 dim 发生变化（或处于起止端点）时，同步调暗全屋灯光并同步拉拢右侧窗帘！
        if abs(dim - getattr(self, "current_dim_factor", -1.0)) > 0.005 or dim in (0.0, 1.0):
            self._apply_lighting(dim)
            self._update_curtains(dim)

        if t >= self.T_LULLABY:
            self._log_once("sleep_lighting", "助眠曲悠扬响起，卧室灯光缓缓调暗，右侧窗帘同步自动拉拢 ...")

        # -------------------------------------------------------------------
        # 7. 启动助眠八音盒摇篮曲（等待台词语音队列 100% 播报完毕且到达 T_LULLABY）
        # -------------------------------------------------------------------
        if (t >= self.T_LULLABY and self.audio_enabled
                and not self._lullaby_started
                and not _SPEECH_BUSY.is_set()):
            self._lullaby_started = True
            try:
                if not os.path.exists(self.lullaby_path):
                    make_lullaby(self.lullaby_path)
                play_lullaby(self.lullaby_path)
                self.lullaby_on = True
            except Exception as exc:
                if not self.quiet:
                    print("助眠曲启动失败:", repr(exc))
            else:
                if not self.quiet:
                    print("助眠曲开始播放 ...")

        # 刷新气泡字幕面板的摄像机朝向或隐藏位置
        self.update_camera_billboards()

    def _set_speech_index(self, idx):
        """内部字幕状态切换器：仅在目标字幕编号 idx 发生变化时触发纹理更新与语音入队。"""
        if self._speech_idx != idx:
            if idx is None:
                self.hide_speech()
                self._speech_idx = -1
            else:
                self.set_speech(idx)
                self.speech_current = idx
                self._speech_idx = idx
                speak_subtitle(idx, enabled=self.audio_enabled)

    # -----------------------------------------------------------------------
    # 交互式主循环与离线超采样渲染器
    # -----------------------------------------------------------------------
    def run(self):
        """启动 60 FPS 实时交互渲染主循环，监听键盘事件（SPACE 暂停/继续、R 重播、Q/ESC 退出）。"""
        # 在 Windows 下将系统定时器精度提升至 1ms，确保帧率稳定丝滑
        if sys.platform.startswith("win"):
            try:
                import ctypes
                ctypes.windll.winmm.timeBeginPeriod(1)
            except Exception:
                pass
        self.reset_story()
        last = time.perf_counter()
        if not self.quiet:
            print("窗口操作：鼠标拖动旋转 / 滚轮缩放；SPACE 暂停，R 重播，Q 或 ESC 退出")
        try:
            while p.isConnected():
                now = time.perf_counter()
                dt = min(0.1, now - last)
                last = now
                # 轮询键盘按键事件
                keys = p.getKeyboardEvents()
                # 空格键（ASCII 32）：切换暂停与继续播放
                if 32 in keys and (keys[32] & p.KEY_WAS_TRIGGERED):
                    self.story_paused = not self.story_paused
                    if self.story_paused:
                        stop_lullaby()
                    elif (self.audio_enabled and self._lullaby_started
                        and os.path.exists(self.lullaby_path)):
                        play_lullaby(self.lullaby_path)
                # R 键（ASCII 114）：从头重播整场剧目
                if 114 in keys and (keys[114] & p.KEY_WAS_TRIGGERED):
                    self.reset_story()
                # Q 键（ASCII 113）或 ESC 键（ASCII 27）：安全退出程序
                if ((113 in keys and (keys[113] & p.KEY_WAS_TRIGGERED)) or
                        (27 in keys and (keys[27] & p.KEY_WAS_TRIGGERED))):
                    break
                # 推进故事动画、更新字幕朝向并步进物理仿真
                self.update_story(dt)
                self.update_camera_billboards()
                p.stepSimulation()
                spent = time.perf_counter() - now
                time.sleep(max(0.0, 1.0 / 60.0 - spent))
        except p.error:
            pass
        finally:
            stop_lullaby()
            try:
                p.disconnect()
            except p.error:
                pass

    def capture_times(self, times, outdir=None, width=None, height=None, ssaa=2):
        """离线超采样快照渲染函数：依次跳转到指定时间点列表 times，渲染 SSAA 高清抗锯齿图像并输出 PNG。"""
        outdir = outdir or os.path.join(ROOT, "renders")
        os.makedirs(outdir, exist_ok=True)
        width = width or self.width
        height = height or self.height
        ssaa = max(1, int(ssaa))
        rw, rh = width * ssaa, height * ssaa
        p.resetDebugVisualizerCamera(self.CAM_DIST, self.CAM_YAW,
                                    self.CAM_PITCH, self.CAM_TARGET)
        view = p.computeViewMatrixFromYawPitchRoll(
            self.CAM_TARGET, self.CAM_DIST, self.CAM_YAW, self.CAM_PITCH, 0, 2)
        proj = p.computeProjectionMatrixFOV(
            self.CAM_FOV, width / height, 0.04, 20.0)
        files = []
        for t in times:
            self.set_story_time(t)
            self.update_camera_billboards()
            for _ in range(2):
                p.stepSimulation()
            img = p.getCameraImage(
                rw, rh, view, proj,
                renderer=p.ER_BULLET_HARDWARE_OPENGL,
                flags=p.ER_NO_SEGMENTATION_MASK, shadow=1)
            rgba = np.reshape(img[2], (rh, rw, 4))[:, :, :3]
            if ssaa > 1:
                rgba = rgba.reshape(height, ssaa, width, ssaa, 3).mean(axis=(1, 3))
            sleep_dim = float(quintic_ease(min(1.0, max(0.0, (t - self.T_LULLABY) / 3.0))))
            final_img = apply_cinematic_postprocess(rgba, sleep_dim=sleep_dim)
            fname = os.path.join(outdir, "story_%05.1fs.png" % t)
            write_png(fname, final_img)
            files.append(fname)
            print("wrote", fname)
        return files


# ---------------------------------------------------------------------------
# 脚本命令行主入口
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="睡前守护机器人 PyBullet 温馨卧室小剧场")
    parser.add_argument("--capture", action="store_true", help="开启离线关键帧截图导出模式")
    parser.add_argument("--times", type=float, nargs="*", default=None, help="指定导出的时间点序列（秒）")
    parser.add_argument("--width", type=int, default=1920, help="渲染窗口或输出图像宽度")
    parser.add_argument("--height", type=int, default=1080, help="渲染窗口或输出图像高度")
    parser.add_argument("--ssaa", type=int, default=2, help="离线截图的超采样抗锯齿倍数")
    args = parser.parse_args()
    if p is not None:
        scene = Scene(gui=True, width=args.width, height=args.height,
                    audio=not args.capture)
        if args.capture:
            times = args.times or [0.0, 10.2, 21.5, 22.4, 25.5, 27.0, 32.5, 35.0, 36.5, 37.8, 41.0]
            scene.capture_times(times, width=args.width, height=args.height, ssaa=args.ssaa)
            p.disconnect()
        else:
            scene.run()
    else:
        print("提示：当前环境未安装 pybullet 库，请在目标 Python 环境（如 Windows .venv）下运行此脚本。")