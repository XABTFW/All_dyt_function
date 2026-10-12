#!/usr/bin/env python3
"""Plot measured PX4 flight data from ULog files.

All figures produced by this script are labelled as measured log data.  It does
not extrapolate flight envelopes or synthesize unrecorded high-speed results.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np
from pyulog import ULog

_CJK_FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc"
font_manager.fontManager.addfont(_CJK_FONT)
plt.rcParams["font.family"] = [font_manager.FontProperties(fname=_CJK_FONT).get_name(), "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False


def dataset(ulog: ULog, name: str):
    matches = [d for d in ulog.data_list if d.name == name and d.multi_id == 0]
    if not matches:
        raise RuntimeError(f"missing required topic: {name}")
    return matches[0].data


def time_s(data: dict[str, np.ndarray], start_us: int) -> np.ndarray:
    return (np.asarray(data["timestamp"], dtype=float) - start_us) / 1e6


def quat_to_euler_deg(q0, q1, q2, q3):
    # PX4 quaternion ordering is w, x, y, z.
    sinr = 2.0 * (q0 * q1 + q2 * q3)
    cosr = 1.0 - 2.0 * (q1 * q1 + q2 * q2)
    roll = np.arctan2(sinr, cosr)
    sinp = np.clip(2.0 * (q0 * q2 - q3 * q1), -1.0, 1.0)
    pitch = np.arcsin(sinp)
    siny = 2.0 * (q0 * q3 + q1 * q2)
    cosy = 1.0 - 2.0 * (q2 * q2 + q3 * q3)
    yaw = np.arctan2(siny, cosy)
    return np.rad2deg(np.column_stack((roll, pitch, yaw)))


def finite_limits(t: np.ndarray, speed: np.ndarray) -> tuple[float, float]:
    valid = np.isfinite(t) & np.isfinite(speed)
    moving = valid & (speed > 0.5)
    idx = np.flatnonzero(moving)
    if idx.size:
        lo = max(float(t[idx[0]]) - 10.0, float(t[valid][0]))
        hi = min(float(t[idx[-1]]) + 10.0, float(t[valid][-1]))
        return lo, hi
    return float(t[valid][0]), float(t[valid][-1])


def style_axis(ax, title: str, ylabel: str, xlim: tuple[float, float]):
    ax.set_title(title)
    ax.set_ylabel(ylabel)
    ax.set_xlim(*xlim)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=8, ncol=2)


def save_plot(fig, path: Path):
    fig.suptitle("实测 ULog 数据（非高速外推、非拦截仿真）", fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, 0.965))
    fig.savefig(path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def plot_one(log_path: Path, out_dir: Path) -> dict[str, object]:
    topics = [
        "vehicle_local_position",
        "vehicle_attitude",
        "vehicle_angular_velocity",
        "vehicle_acceleration",
    ]
    ulog = ULog(str(log_path), message_name_filter_list=topics)
    lp = dataset(ulog, "vehicle_local_position")
    att = dataset(ulog, "vehicle_attitude")
    rates = dataset(ulog, "vehicle_angular_velocity")
    acc = dataset(ulog, "vehicle_acceleration")

    tp = time_s(lp, ulog.start_timestamp)
    ta = time_s(att, ulog.start_timestamp)
    tr = time_s(rates, ulog.start_timestamp)
    tc = time_s(acc, ulog.start_timestamp)
    xyz = np.column_stack((lp["x"], lp["y"], lp["z"]))
    vel = np.column_stack((lp["vx"], lp["vy"], lp["vz"]))
    speed = np.linalg.norm(vel, axis=1)
    xlim = finite_limits(tp, speed)
    euler = quat_to_euler_deg(att["q[0]"], att["q[1]"], att["q[2]"], att["q[3]"])
    omega = np.rad2deg(np.column_stack((rates["xyz[0]"], rates["xyz[1]"], rates["xyz[2]"])))
    accel = np.column_stack((acc["xyz[0]"], acc["xyz[1]"], acc["xyz[2]"]))
    stem = log_path.stem

    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    for i, label in enumerate(("North x", "East y", "Down z")):
        axes[i].plot(tp, xyz[:, i], lw=0.8, label=label)
        style_axis(axes[i], f"位置分量：{label}", "m", xlim)
    axes[-1].set_xlabel("日志相对时间 (s)")
    save_plot(fig, out_dir / f"{stem}_01_position.png")

    fig, ax = plt.subplots(figsize=(12, 5.8))
    for i, label in enumerate(("vx", "vy", "vz")):
        ax.plot(tp, vel[:, i], lw=0.7, label=label)
    ax.plot(tp, speed, color="black", lw=1.1, label="speed magnitude")
    imax = int(np.nanargmax(speed))
    ax.scatter([tp[imax]], [speed[imax]], color="red", zorder=5)
    ax.annotate(f"实测最大值 {speed[imax]:.2f} m/s\nt={tp[imax]:.1f}s",
                (tp[imax], speed[imax]), xytext=(8, 10), textcoords="offset points")
    style_axis(ax, "速度分量与速度大小", "m/s", xlim)
    ax.set_xlabel("日志相对时间 (s)")
    save_plot(fig, out_dir / f"{stem}_02_velocity.png")

    fig, ax = plt.subplots(figsize=(12, 5.8))
    for i, label in enumerate(("ax", "ay", "az")):
        ax.plot(tc, accel[:, i], lw=0.7, label=label)
    style_axis(ax, "机体系/估计加速度（vehicle_acceleration）", "m/s²", xlim)
    ax.set_xlabel("日志相对时间 (s)")
    save_plot(fig, out_dir / f"{stem}_03_acceleration.png")

    fig, ax = plt.subplots(figsize=(12, 5.8))
    for i, label in enumerate(("roll", "pitch", "yaw")):
        ax.plot(ta, euler[:, i], lw=0.7, label=label)
    style_axis(ax, "姿态欧拉角", "deg", xlim)
    ax.set_xlabel("日志相对时间 (s)")
    save_plot(fig, out_dir / f"{stem}_04_attitude.png")

    fig, ax = plt.subplots(figsize=(12, 5.8))
    for i, label in enumerate(("p", "q", "r")):
        ax.plot(tr, omega[:, i], lw=0.65, label=label)
    style_axis(ax, "机体系姿态角速度", "deg/s", xlim)
    ax.set_xlabel("日志相对时间 (s)")
    save_plot(fig, out_dir / f"{stem}_05_angular_velocity.png")

    valid = np.isfinite(speed)
    moving = valid & (speed > 0.5)
    return {
        "log": log_path.name,
        "duration_s": (ulog.last_timestamp - ulog.start_timestamp) / 1e6,
        "plot_start_s": xlim[0],
        "plot_end_s": xlim[1],
        "max_speed_mps": float(np.nanmax(speed)),
        "p95_speed_mps": float(np.nanpercentile(speed[valid], 95)),
        "mean_moving_speed_mps": float(np.nanmean(speed[moving])) if np.any(moving) else float("nan"),
        "max_abs_accel_mps2": float(np.nanmax(np.abs(accel))),
        "max_abs_roll_deg": float(np.nanmax(np.abs(euler[:, 0]))),
        "max_abs_pitch_deg": float(np.nanmax(np.abs(euler[:, 1]))),
        "max_abs_body_rate_dps": float(np.nanmax(np.abs(omega))),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("logs", nargs="+", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    summaries = [plot_one(path, args.output) for path in args.logs]
    with (args.output / "measured_log_summary.csv").open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=list(summaries[0]))
        writer.writeheader()
        writer.writerows(summaries)
    with (args.output / "README.txt").open("w", encoding="utf-8") as f:
        f.write(
            "本目录图表仅来自两个 ULog 的实测记录。\n"
            "未进行高速外推，未生成目标轨迹、末制导指令、脱靶量或拦截概率。\n"
            "图中 z/vz 采用 PX4 NED 坐标约定，向下为正。\n"
            "绘图窗口按速度大于 0.5 m/s 的活动区间自动裁剪，并在两端保留 10 秒。\n"
        )


if __name__ == "__main__":
    main()
