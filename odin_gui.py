#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
os.environ["ROS_LOCALHOST_ONLY"] = "1"
os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
import re
import shutil
import signal
import subprocess
import threading
import math
import time
from datetime import datetime
import tkinter as tk
from tkinter import messagebox

# ----------------------------- config ---------------------------------
RECORD_DIR = "/home/nuc11/odin_ws/src/odin_ros_driver/recorddata"

# odin_ros_driver 目录（用于执行 ./set_param.sh）
ODIN_DRIVER_DIR = "/home/nuc11/odin_ws/src/odin_ros_driver"
SET_PARAM_SCRIPT = os.path.join(ODIN_DRIVER_DIR, "set_param.sh")

# 触发清理阈值：>300GB
TRIGGER_GB = 300
TRIGGER_BYTES = TRIGGER_GB * 1024**3

# 清理目标：删到 <=200GB
TARGET_GB = 200
TARGET_BYTES = TARGET_GB * 1024**3

# 弹窗预览将删除的前 N 个文件夹
PREVIEW_N = 10

# 你的 launch 命令
ROS_LAUNCH_CMD = "ros2 launch odin_ros_driver odin1_ros2.launch.py"

# FAST-LIVO2 launch 命令
FAST_LIVO2_LAUNCH_CMD = "ros2 launch fast_livo mapping_odin1.launch.py"

# 需要 source 的 ROS 环境（按你的实际路径改）
ROS_ENV_CMD = (
    "source /opt/ros/humble/setup.bash && "
    "source /home/nuc11/odin_ws/install/setup.bash && "
    "export ROS_LOCALHOST_ONLY=1"
)

# FAST-LIVO2 ROS 环境
FAST_LIVO2_ENV_CMD = (
    "source /home/nuc11/fast_livo2/install/setup.bash && "
    "export ROS_LOCALHOST_ONLY=1"
)

# 时间戳文件夹名格式：YYYYMMDD_HHMMSS
TS_PATTERN = re.compile(r"^\d{8}_\d{6}$")

# Odom topic
ODOM_TOPIC = "/odin1/odometry"
ODOM_REFRESH_MS = 100  # GUI 刷新频率（ms）

# 保存地图后等待秒数
SAVE_WAIT_SEC = 15


# ----------------------------- utils ---------------------------------
def log_to(widget: tk.Text, msg: str):
    widget.insert(tk.END, msg + "\n")
    widget.see(tk.END)


def bytes_to_gb(b: int) -> float:
    return b / (1024**3)


def get_dir_size_bytes(path: str) -> int:
    """统计目录大小（优先 du -sb，快；失败回退到 walk）"""
    try:
        out = subprocess.check_output(["du", "-sb", path], text=True).strip()
        return int(out.split()[0])
    except Exception:
        total = 0
        for root, _, files in os.walk(path):
            for f in files:
                fp = os.path.join(root, f)
                try:
                    total += os.path.getsize(fp)
                except OSError:
                    pass
        return total


def get_path_size_bytes(path: str) -> int:
    """统计单个文件夹大小（优先 du -sb）"""
    try:
        out = subprocess.check_output(["du", "-sb", path], text=True).strip()
        return int(out.split()[0])
    except Exception:
        total = 0
        for root, _, files in os.walk(path):
            for f in files:
                fp = os.path.join(root, f)
                try:
                    total += os.path.getsize(fp)
                except OSError:
                    pass
        return total


def list_timestamp_folders(path: str):
    """返回 [(dt, foldername, fullpath)]，按 dt 从旧到新排序"""
    items = []
    if not os.path.isdir(path):
        return items

    for name in os.listdir(path):
        full = os.path.join(path, name)
        if os.path.isdir(full) and TS_PATTERN.match(name):
            try:
                dt = datetime.strptime(name, "%Y%m%d_%H%M%S")
            except ValueError:
                continue
            items.append((dt, name, full))

    items.sort(key=lambda x: x[0])  # oldest first
    return items


def build_deletion_plan(record_dir: str, current_bytes: int, target_bytes: int, log_cb=None):
    """
    从最旧时间戳文件夹开始，计算需要删除哪些，直到 <= target_bytes。
    返回 plan: [(name, fullpath, size_bytes), ...], 以及计划删除后大小 planned_final_bytes
    """
    folders = list_timestamp_folders(record_dir)
    plan = []
    planned_bytes = current_bytes

    for _, name, full in folders:
        if planned_bytes <= target_bytes:
            break
        if log_cb:
            log_cb(f"[Plan] 计算 {name} 大小...")
        sz = get_path_size_bytes(full)
        plan.append((name, full, sz))
        planned_bytes -= sz

    return plan, planned_bytes


def quat_to_rpy(qx, qy, qz, qw):
    """quaternion(xyzw) -> roll,pitch,yaw (radians)"""
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


# ----------------------------- ROS Odom Listener ---------------------------------
class OdomListener:
    """
    后台线程 spin，保存最新 odom（线程安全读取），并估计 Hz（EMA）。
    """
    def __init__(self, topic: str):
        self.topic = topic
        self._lock = threading.Lock()
        self._latest = None  # dict

        self._stop_evt = threading.Event()
        self._thread = None

        self._rclpy = None
        self._node = None
        self._executor = None

        self._last_recv_t = None
        self._hz_ema = None

    def start(self, log_cb=None):
        if self._thread and self._thread.is_alive():
            return True

        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
            from nav_msgs.msg import Odometry
        except Exception as e:
            if log_cb:
                log_cb(f"[Odom] 导入 ROS2 python 失败：{e}")
            return False

        self._rclpy = rclpy

        try:
            if not rclpy.ok():
                rclpy.init(args=None)
        except Exception:
            pass

        self._node = rclpy.create_node("odin_gui_odom_listener")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        def cb(msg: Odometry):
            now_t = time.monotonic()

            hz = None
            if self._last_recv_t is not None:
                dt = now_t - self._last_recv_t
                if dt > 1e-6:
                    hz_inst = 1.0 / dt
                    if self._hz_ema is None:
                        self._hz_ema = hz_inst
                    else:
                        alpha = 0.2
                        self._hz_ema = (1 - alpha) * self._hz_ema + alpha * hz_inst
                    hz = self._hz_ema
            self._last_recv_t = now_t

            try:
                stamp = msg.header.stamp
                stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
            except Exception:
                stamp_ns = None

            p = msg.pose.pose.position
            o = msg.pose.pose.orientation
            roll, pitch, yaw = quat_to_rpy(o.x, o.y, o.z, o.w)

            with self._lock:
                self._latest = {
                    "x": float(p.x), "y": float(p.y), "z": float(p.z),
                    "roll": float(roll), "pitch": float(pitch), "yaw": float(yaw),
                    "hz": None if hz is None else float(hz),
                    "stamp_ns": stamp_ns,
                }

        self._node.create_subscription(Odometry, self.topic, cb, qos)

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)

        self._stop_evt.clear()
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

        if log_cb:
            log_cb(f"[Odom] 已开始订阅 {self.topic}")
        return True

    def _spin(self):
        while not self._stop_evt.is_set():
            try:
                self._executor.spin_once(timeout_sec=0.1)
            except Exception:
                pass

    def get_latest(self):
        with self._lock:
            return None if self._latest is None else dict(self._latest)

    def stop(self, log_cb=None):
        self._stop_evt.set()

        try:
            if self._executor is not None:
                self._executor.shutdown()
        except Exception:
            pass

        try:
            if self._node is not None:
                self._node.destroy_node()
        except Exception:
            pass

        try:
            if self._rclpy is not None and self._rclpy.ok():
                self._rclpy.shutdown()
        except Exception:
            pass

        if log_cb:
            log_cb("[Odom] 已停止订阅")


# ----------------------------- GUI ---------------------------------
class OdinGUI:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Odin SLAM 控制面板（Demo）")

        self.proc = None  # ros2 launch 进程句柄
        self.fast_livo_proc = None  # FAST-LIVO2 launch 进程句柄

        # Odom monitor
        self.odom_listener = OdomListener(ODOM_TOPIC)
        self.odom_win = None
        self.odom_text = tk.StringVar(value="等待 /odin1/odometry ...")
        self._odom_ui_job = None

        # Buttons
        btn_frame = tk.Frame(root)
        btn_frame.pack(fill=tk.X, padx=10, pady=10)

        self.btn_start = tk.Button(btn_frame, text="启动 SLAM", command=self.on_start_slam)
        self.btn_start.pack(side=tk.LEFT, padx=5)

        self.btn_stop = tk.Button(
            btn_frame,
            text="保存地图 / 关闭 SLAM（可选）",
            command=self.on_stop_slam,
            state=tk.DISABLED,
        )
        self.btn_stop.pack(side=tk.LEFT, padx=5)

        self.btn_start_fast_livo = tk.Button(
            btn_frame,
            text="启动 FAST-LIVO2",
            command=self.on_start_fast_livo,
        )
        self.btn_start_fast_livo.pack(side=tk.LEFT, padx=5)

        self.btn_stop_fast_livo = tk.Button(
            btn_frame,
            text="关闭 FAST-LIVO2",
            command=self.on_stop_fast_livo,
            state=tk.DISABLED,
        )
        self.btn_stop_fast_livo.pack(side=tk.LEFT, padx=5)

        # Log window
        self.log = tk.Text(root, height=18, width=120)
        self.log.pack(fill=tk.BOTH, expand=True, padx=10, pady=(0, 10))

        log_to(self.log, f"[Init] RECORD_DIR = {RECORD_DIR}")
        log_to(self.log, f"[Init] Trigger clean if > {TRIGGER_GB} GB, clean down to <= {TARGET_GB} GB")
        log_to(self.log, f"[Init] Clean preview shows first {PREVIEW_N} folders (oldest first)")
        log_to(self.log, f"[Init] SET_PARAM_SCRIPT = {SET_PARAM_SCRIPT}")
        log_to(self.log, f"[Init] ODOM_TOPIC = {ODOM_TOPIC}")
        log_to(self.log, f"[Init] FAST_LIVO2_CMD = {FAST_LIVO2_LAUNCH_CMD}")
        log_to(self.log, "[Init] 先点“启动 SLAM”，再点“保存地图 / 关闭 SLAM（可选）”。")

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---------------- button1: start ----------------
    def on_start_slam(self):
        if self.proc is not None and self.proc.poll() is None:
            messagebox.showinfo("提示", "SLAM 已在运行中（进程未退出）。")
            return

        self.btn_start.config(state=tk.DISABLED)
        self.btn_stop.config(state=tk.DISABLED)
        threading.Thread(target=self._start_slam_flow, daemon=True).start()

    def _start_slam_flow(self):
        try:
            if not os.path.isdir(RECORD_DIR):
                self._ui_log(f"[ERR] 目录不存在：{RECORD_DIR}")
                return

            self._ui_log("[Check] 正在统计 recorddata 目录大小（可能需要一点时间）...")
            size_bytes = get_dir_size_bytes(RECORD_DIR)
            size_gb = bytes_to_gb(size_bytes)
            self._ui_log(f"[Check] 当前大小：{size_gb:.2f} GB")

            if size_bytes > TRIGGER_BYTES:
                self._ui_log(f"[Plan] 超过 {TRIGGER_GB} GB，生成删除计划（目标 <= {TARGET_GB} GB）...")
                plan, planned_bytes = build_deletion_plan(RECORD_DIR, size_bytes, TARGET_BYTES, log_cb=self._ui_log)

                if not plan:
                    self._ui_log("[Abort] 未生成可删除计划（可能没有符合时间戳格式的文件夹），取消启动。")
                    return

                planned_final_gb = bytes_to_gb(planned_bytes)
                ok = self._ask_user_confirm_clean(size_gb, planned_final_gb, plan, PREVIEW_N)
                if not ok:
                    self._ui_log("[Abort] 你取消了清理，本次不启动 SLAM。")
                    return

                self._ui_log(f"[Clean] 开始按计划删除 {len(plan)} 个文件夹...")
                for idx, (name, full, sz) in enumerate(plan, start=1):
                    self._ui_log(f"[Clean] ({idx}/{len(plan)}) 删除：{name}  ({bytes_to_gb(sz):.2f} GB)")
                    try:
                        shutil.rmtree(full)
                    except Exception as e:
                        self._ui_log(f"[ERR] 删除失败 {name}: {e}")
                        self._ui_log("[Abort] 清理未完成，为安全起见取消启动。")
                        return

                final_bytes = get_dir_size_bytes(RECORD_DIR)
                final_gb = bytes_to_gb(final_bytes)
                self._ui_log(f"[Clean] 清理后实际大小：{final_gb:.2f} GB")
                if final_bytes > TARGET_BYTES:
                    self._ui_log(f"[Abort] 清理后仍 > {TARGET_GB} GB，为安全起见取消启动。")
                    return

                self._ui_log("[Clean] 清理完成，准备启动 SLAM。")

            self._ui_log("[Run] 启动 SLAM（ros2 launch）...")
            cmd = f'{ROS_ENV_CMD} && {ROS_LAUNCH_CMD}'
            self._ui_log(f"[Run] CMD: {cmd}")

            self.proc = subprocess.Popen(
                ["bash", "-lc", cmd],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                universal_newlines=True,
                start_new_session=True,
            )

            self._ui_log(f"[Run] 已启动，PID = {self.proc.pid}")
            self._ui_log("[Run] 左上角将弹出 odom 位姿窗口。")

            self.root.after(0, lambda: self.btn_stop.config(state=tk.NORMAL))
            self.root.after(0, self._open_odom_window_and_start_listener)

            for line in self.proc.stdout:
                line = line.rstrip("\n")
                if line:
                    self._ui_log(line)

            rc = self.proc.wait()
            self._ui_log(f"[Run] 进程退出，return code = {rc}")

        except Exception as e:
            self._ui_log(f"[ERR] 启动流程异常：{e}")

        finally:
            self._ui_enable_buttons()

    # ---------------- FAST-LIVO2 start ----------------
    def on_start_fast_livo(self):
        if self.fast_livo_proc is not None and self.fast_livo_proc.poll() is None:
            messagebox.showinfo("提示", "FAST-LIVO2 已在运行中（进程未退出）。")
            return

        self.btn_start_fast_livo.config(state=tk.DISABLED)
        self.btn_stop_fast_livo.config(state=tk.DISABLED)
        threading.Thread(target=self._start_fast_livo_flow, daemon=True).start()

    def on_stop_fast_livo(self):
        if not self._fast_livo_running():
            messagebox.showinfo("提示", "FAST-LIVO2 当前没有运行。")
            self._ui_enable_buttons()
            return

        self.btn_stop_fast_livo.config(state=tk.DISABLED)
        threading.Thread(target=self._stop_fast_livo_flow, daemon=True).start()

    def _stop_fast_livo_flow(self):
        try:
            self._ui_log("[FAST-LIVO2] 开始关闭进程...")
            self._terminate_fast_livo_process()
        except Exception as e:
            self._ui_log(f"[ERR] FAST-LIVO2 关闭流程异常：{e}")
        finally:
            self._ui_enable_buttons()

    def _start_fast_livo_flow(self):
        try:
            self._ui_log("[FAST-LIVO2] 启动 mapping node...")
            cmd = f'{FAST_LIVO2_ENV_CMD} && {FAST_LIVO2_LAUNCH_CMD}'
            self._ui_log(f"[FAST-LIVO2] CMD: {cmd}")

            self.fast_livo_proc = subprocess.Popen(
                ["bash", "-lc", cmd],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                universal_newlines=True,
                start_new_session=True,
            )

            self._ui_log(f"[FAST-LIVO2] 已启动，PID = {self.fast_livo_proc.pid}")
            self._ui_enable_buttons()

            for line in self.fast_livo_proc.stdout:
                line = line.rstrip("\n")
                if line:
                    self._ui_log(f"[FAST-LIVO2] {line}")

            rc = self.fast_livo_proc.wait()
            self._ui_log(f"[FAST-LIVO2] 进程退出，return code = {rc}")

        except Exception as e:
            self._ui_log(f"[ERR] FAST-LIVO2 启动流程异常：{e}")

        finally:
            self._ui_enable_buttons()

    def _ask_user_confirm_clean(self, current_gb: float, planned_final_gb: float, plan, preview_n: int) -> bool:
        result_holder = {"ok": False}
        ev = threading.Event()

        def _show():
            lines = []
            for i, (name, _, sz) in enumerate(plan[:preview_n], start=1):
                lines.append(f"{i:02d}. {name}   ({bytes_to_gb(sz):.2f} GB)")
            total_del_gb = bytes_to_gb(sum(sz for _, _, sz in plan))
            more = ""
            if len(plan) > preview_n:
                more = f"\n... 还有 {len(plan) - preview_n} 个将被删除（未在此预览中展示）"

            msg = (
                f"检测到录制数据目录超过阈值：\n\n"
                f"当前大小：{current_gb:.2f} GB（> {TRIGGER_GB} GB）\n"
                f"清理目标：<= {TARGET_GB} GB\n"
                f"计划删除文件夹数：{len(plan)}\n"
                f"计划总删除量：约 {total_del_gb:.2f} GB\n"
                f"按计划清理后预计：{planned_final_gb:.2f} GB\n\n"
                f"将删除（按从旧到新）前 {min(preview_n, len(plan))} 个预览如下：\n"
                + "\n".join(lines)
                + more
                + f"\n\n是否继续清理并启动 SLAM？\n\n目录：{RECORD_DIR}"
            )
            result_holder["ok"] = messagebox.askokcancel("确认清理旧录制数据", msg)
            ev.set()

        self.root.after(0, _show)
        ev.wait()
        return result_holder["ok"]

    # ---------------- Odom window ----------------
    def _open_odom_window_and_start_listener(self):
        if self.odom_win is None or not self._window_exists(self.odom_win):
            self.odom_win = tk.Toplevel(self.root)
            self.odom_win.title("Odom Pose")
            self.odom_win.attributes("-topmost", True)
            self.odom_win.geometry("+20+20")  # 左上角

            frm = tk.Frame(self.odom_win, padx=10, pady=10)
            frm.pack(fill=tk.BOTH, expand=True)

            tk.Label(frm, text=f"Topic: {ODOM_TOPIC}", font=("Arial", 11, "bold")).pack(anchor="w")
            tk.Label(frm, textvariable=self.odom_text, justify="left", font=("Consolas", 11)).pack(anchor="w", pady=(6, 0))
            tk.Label(frm, text="单位：m / deg", font=("Arial", 9)).pack(anchor="w", pady=(6, 0))

            self.odom_win.protocol("WM_DELETE_WINDOW", self._close_odom_window)

        ok = self.odom_listener.start(log_cb=self._ui_log)
        if not ok:
            self.odom_text.set("无法订阅 odom：rclpy/nav_msgs 不可用\n请用 ROS 环境启动本 GUI")
            return

        self._schedule_odom_ui_update()

    def _schedule_odom_ui_update(self):
        if self._odom_ui_job is not None:
            try:
                self.root.after_cancel(self._odom_ui_job)
            except Exception:
                pass
        self._odom_ui_job = self.root.after(ODOM_REFRESH_MS, self._update_odom_ui_once)

    def _update_odom_ui_once(self):
        try:
            if self.odom_win is None or not self._window_exists(self.odom_win):
                return

            data = self.odom_listener.get_latest()
            if data is None:
                self.odom_text.set("等待 /odin1/odometry ...")
            else:
                x, y, z = data["x"], data["y"], data["z"]
                r, p, yw = data["roll"], data["pitch"], data["yaw"]
                rd, pd, ywd = math.degrees(r), math.degrees(p), math.degrees(yw)

                stamp_ns = data.get("stamp_ns", None)
                if stamp_ns is None:
                    stamp_str = "N/A"
                    local_str = "N/A"
                else:
                    sec = stamp_ns // 1_000_000_000
                    nsec = stamp_ns % 1_000_000_000
                    stamp_str = f"{sec}.{nsec:09d}"

                    # 小于 2000-01-01 多半不是 wall-clock（仿真/未知）
                    if sec < 946684800:
                        local_str = "SIM/UNKNOWN"
                    else:
                        ts = sec + (nsec / 1e9)
                        local_str = datetime.fromtimestamp(ts).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

                hz = data.get("hz", None)
                hz_str = "N/A" if hz is None else f"{hz:.2f}"

                self.odom_text.set(
                    f"stamp : {stamp_str}\n"
                    f"local : {local_str}\n"
                    f"hz    : {hz_str}\n"
                    f"x     : {x: .4f}\n"
                    f"y     : {y: .4f}\n"
                    f"z     : {z: .4f}\n"
                    f"roll  : {rd: .2f}\n"
                    f"pitch : {pd: .2f}\n"
                    f"yaw   : {ywd: .2f}"
                )
        finally:
            if self.odom_win is not None and self._window_exists(self.odom_win):
                self._schedule_odom_ui_update()

    def _close_odom_window(self):
        try:
            if self.odom_win is not None:
                self.odom_win.destroy()
        except Exception:
            pass
        self.odom_win = None

    @staticmethod
    def _window_exists(win):
        try:
            return win.winfo_exists() == 1
        except Exception:
            return False

    # ---------------- button2: save-map + optional close ----------------
    def on_stop_slam(self):
        # 不退出 GUI，只做“保存地图/关闭 slam”的交互
        self.btn_start.config(state=tk.DISABLED)
        self.btn_stop.config(state=tk.DISABLED)
        threading.Thread(target=self._stop_slam_flow, daemon=True).start()

    def _stop_slam_flow(self):
        try:
            running = self._slam_running()
            if not running:
                self._ui_log("[Info] 当前没有运行中的 SLAM（你可以直接启动）。")
                self._ui_enable_buttons()
                return

            # 1) 是否保存地图？
            save = self._ask_save_map()

            if save:
                if not os.path.isfile(SET_PARAM_SCRIPT):
                    self._ui_log(f"[ERR] 找不到脚本：{SET_PARAM_SCRIPT}")
                    self._ui_log("[Stop] 将继续等待并进入“是否关闭 SLAM”的选择。")
                else:
                    self._ui_log("[Stop] 选择：保存地图。发送命令：./set_param.sh save_map 1")
                    cmd = f'cd "{ODIN_DRIVER_DIR}" && chmod +x "{SET_PARAM_SCRIPT}" && ./set_param.sh save_map 1'
                    full_cmd = f'{ROS_ENV_CMD} && {cmd}'
                    self._ui_log(f"[Stop] CMD: {full_cmd}")
                    subprocess.run(["bash", "-lc", full_cmd], check=False)
            else:
                self._ui_log("[Stop] 选择：不保存地图。不发送任何命令。")

            # 2) 等待 15s（给保存地图留时间；不保存也按原需求等 15s）
            self._ui_log(f"[Stop] 等待 {SAVE_WAIT_SEC} 秒（用于保存地图/收尾）...")
            for i in range(SAVE_WAIT_SEC, 0, -1):
                self._ui_log(f"[Stop] 倒计时：{i}s")
                threading.Event().wait(1.0)

            # 3) 是否关闭 SLAM？
            close = self._ask_close_slam()
            if close:
                self._ui_log("[Stop] 选择：关闭 SLAM。开始结束进程...")
                self._terminate_slam_process()
                self._ui_log("[Stop] SLAM 已请求关闭（如仍有残留，可看日志/再点一次）。")
                # 关掉 odom 监控（slam 关掉后订阅也没意义）
                self._stop_odom_monitor()
            else:
                self._ui_log("[Stop] 选择：不关闭 SLAM。保持运行。")

        except Exception as e:
            self._ui_log(f"[ERR] 保存/关闭流程异常：{e}")

        finally:
            self._ui_enable_buttons()

    def _ask_save_map(self) -> bool:
        result_holder = {"ans": False}
        ev = threading.Event()

        def _show():
            msg = (
                "是否保存地图？\n\n"
                "选择“是”：发送 ./set_param.sh save_map 1\n"
                "选择“否”：不发送任何命令\n\n"
                f"之后会等待 {SAVE_WAIT_SEC} 秒，再询问是否关闭 SLAM。"
            )
            result_holder["ans"] = messagebox.askyesno("保存地图", msg)
            ev.set()

        self.root.after(0, _show)
        ev.wait()
        return result_holder["ans"]

    def _ask_close_slam(self) -> bool:
        result_holder = {"ans": False}
        ev = threading.Event()

        def _show():
            msg = "是否关闭 SLAM？\n\n选择“是”：结束 SLAM 进程\n选择“否”：保持 SLAM 继续运行"
            result_holder["ans"] = messagebox.askyesno("关闭 SLAM", msg)
            ev.set()

        self.root.after(0, _show)
        ev.wait()
        return result_holder["ans"]

    def _slam_running(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def _fast_livo_running(self) -> bool:
        return self.fast_livo_proc is not None and self.fast_livo_proc.poll() is None

    def _terminate_slam_process(self):
        if self.proc is None:
            self._ui_log("[Stop] 未检测到 SLAM 进程句柄。")
            return
        if self.proc.poll() is not None:
            self._ui_log("[Stop] SLAM 进程已退出，无需 kill。")
            return

        try:
            pgid = os.getpgid(self.proc.pid)
        except Exception:
            pgid = None

        # SIGINT（类似 Ctrl+C）
        try:
            if pgid is not None:
                self._ui_log(f"[Stop] 发送 SIGINT 到进程组：PGID={pgid}")
                os.killpg(pgid, signal.SIGINT)
            else:
                self._ui_log(f"[Stop] 发送 SIGINT 到 PID={self.proc.pid}")
                self.proc.send_signal(signal.SIGINT)
        except Exception as e:
            self._ui_log(f"[WARN] SIGINT 失败：{e}")

        # 等待最多 5 秒
        try:
            self.proc.wait(timeout=5)
            self._ui_log("[Stop] SLAM 已优雅退出。")
            return
        except Exception:
            pass

        # SIGTERM
        try:
            if pgid is not None:
                self._ui_log("[Stop] SIGINT 超时，发送 SIGTERM 到进程组...")
                os.killpg(pgid, signal.SIGTERM)
            else:
                self._ui_log("[Stop] SIGINT 超时，发送 SIGTERM 到进程...")
                self.proc.terminate()
        except Exception as e:
            self._ui_log(f"[WARN] SIGTERM 失败：{e}")

        try:
            self.proc.wait(timeout=5)
            self._ui_log("[Stop] SLAM 已退出。")
            return
        except Exception:
            pass

        # SIGKILL（兜底）
        try:
            if pgid is not None:
                self._ui_log("[Stop] SIGTERM 超时，发送 SIGKILL 到进程组（强制结束）...")
                os.killpg(pgid, signal.SIGKILL)
            else:
                self._ui_log("[Stop] SIGTERM 超时，发送 SIGKILL（强制结束）...")
                self.proc.kill()
        except Exception as e:
            self._ui_log(f"[WARN] SIGKILL 失败：{e}")

    def _terminate_fast_livo_process(self):
        if self.fast_livo_proc is None:
            self._ui_log("[FAST-LIVO2] 未检测到进程句柄。")
            return
        if self.fast_livo_proc.poll() is not None:
            self._ui_log("[FAST-LIVO2] 进程已退出，无需 kill。")
            return

        try:
            pgid = os.getpgid(self.fast_livo_proc.pid)
        except Exception:
            pgid = None

        try:
            if pgid is not None:
                self._ui_log(f"[FAST-LIVO2] 发送 SIGINT 到进程组：PGID={pgid}")
                os.killpg(pgid, signal.SIGINT)
            else:
                self._ui_log(f"[FAST-LIVO2] 发送 SIGINT 到 PID={self.fast_livo_proc.pid}")
                self.fast_livo_proc.send_signal(signal.SIGINT)
        except Exception as e:
            self._ui_log(f"[WARN] FAST-LIVO2 SIGINT 失败：{e}")

        try:
            self.fast_livo_proc.wait(timeout=5)
            self._ui_log("[FAST-LIVO2] 已优雅退出。")
            return
        except Exception:
            pass

        try:
            if pgid is not None:
                self._ui_log("[FAST-LIVO2] SIGINT 超时，发送 SIGTERM 到进程组...")
                os.killpg(pgid, signal.SIGTERM)
            else:
                self._ui_log("[FAST-LIVO2] SIGINT 超时，发送 SIGTERM 到进程...")
                self.fast_livo_proc.terminate()
        except Exception as e:
            self._ui_log(f"[WARN] FAST-LIVO2 SIGTERM 失败：{e}")

        try:
            self.fast_livo_proc.wait(timeout=5)
            self._ui_log("[FAST-LIVO2] 已退出。")
            return
        except Exception:
            pass

        try:
            if pgid is not None:
                self._ui_log("[FAST-LIVO2] SIGTERM 超时，发送 SIGKILL 到进程组（强制结束）...")
                os.killpg(pgid, signal.SIGKILL)
            else:
                self._ui_log("[FAST-LIVO2] SIGTERM 超时，发送 SIGKILL（强制结束）...")
                self.fast_livo_proc.kill()
        except Exception as e:
            self._ui_log(f"[WARN] FAST-LIVO2 SIGKILL 失败：{e}")

    def _stop_odom_monitor(self):
        if self._odom_ui_job is not None:
            try:
                self.root.after_cancel(self._odom_ui_job)
            except Exception:
                pass
            self._odom_ui_job = None

        if self.odom_win is not None and self._window_exists(self.odom_win):
            try:
                self.odom_win.destroy()
            except Exception:
                pass
        self.odom_win = None

        try:
            self.odom_listener.stop(log_cb=self._ui_log)
        except Exception:
            pass

    # ---------------- UI helpers ----------------
    def _ui_log(self, msg: str):
        self.root.after(0, lambda: log_to(self.log, msg))

    def _ui_enable_buttons(self):
        def _set():
            running = self._slam_running()
            fast_livo_running = self._fast_livo_running()
            self.btn_start.config(state=tk.DISABLED if running else tk.NORMAL)
            self.btn_stop.config(state=tk.NORMAL if running else tk.DISABLED)
            self.btn_start_fast_livo.config(state=tk.DISABLED if fast_livo_running else tk.NORMAL)
            self.btn_stop_fast_livo.config(state=tk.NORMAL if fast_livo_running else tk.DISABLED)
        self.root.after(0, _set)

    def _on_close(self):
        # 手动关窗口：停 odom，再尝试结束 slam
        try:
            self._stop_odom_monitor()
        except Exception:
            pass
        try:
            self._terminate_slam_process()
        except Exception:
            pass
        try:
            self._terminate_fast_livo_process()
        except Exception:
            pass
        self.root.quit()


def main():
    root = tk.Tk()
    _ = OdinGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
