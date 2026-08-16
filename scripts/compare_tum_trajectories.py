#!/usr/bin/env python3

# Copyright 2026 Caio Mauro
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Caio Mauro nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import math
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def load_tum(path: Path):
    data = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) != 8:
                continue
            vals = [float(x) for x in parts]
            data.append(vals)
    if not data:
        raise RuntimeError(f"no TUM entries found in {path}")
    arr = np.asarray(data, dtype=np.float64)
    return arr[:, 0], arr[:, 1:4]


def match_by_time(gt_t, gt_p, est_t, est_p, max_dt=0.02):
    matched_gt = []
    matched_est = []
    matched_t = []
    j = 0
    for i, t in enumerate(est_t):
        while j + 1 < len(gt_t) and gt_t[j + 1] <= t:
            j += 1
        candidates = [j]
        if j + 1 < len(gt_t):
            candidates.append(j + 1)
        best = min(candidates, key=lambda idx: abs(gt_t[idx] - t))
        dt = abs(gt_t[best] - t)
        if dt <= max_dt:
            matched_gt.append(gt_p[best])
            matched_est.append(est_p[i])
            matched_t.append(t)
    if len(matched_t) < 3:
        raise RuntimeError("too few matched samples after timestamp association")
    return np.asarray(matched_t), np.asarray(matched_gt), np.asarray(matched_est)


def rigid_align(src, dst):
    src_mean = src.mean(axis=0)
    dst_mean = dst.mean(axis=0)
    src_c = src - src_mean
    dst_c = dst - dst_mean
    cov = src_c.T @ dst_c / src.shape[0]
    u, _, vt = np.linalg.svd(cov)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:
        vt[-1, :] *= -1
        r = vt.T @ u.T
    t = dst_mean - r @ src_mean
    return r, t


def metrics(err):
    abs_err = np.linalg.norm(err, axis=1)
    return {
        "rmse": math.sqrt(float(np.mean(abs_err ** 2))),
        "mae": float(np.mean(abs_err)),
        "max": float(np.max(abs_err)),
        "latest": float(abs_err[-1]),
        "bias_x": float(np.mean(err[:, 0])),
        "bias_y": float(np.mean(err[:, 1])),
        "bias_z": float(np.mean(err[:, 2])),
        "rmse_x": math.sqrt(float(np.mean(err[:, 0] ** 2))),
        "rmse_y": math.sqrt(float(np.mean(err[:, 1] ** 2))),
        "rmse_z": math.sqrt(float(np.mean(err[:, 2] ** 2))),
    }


def plot_overlay(times, gt, est_aligned, out_path: Path):
    fig, axes = plt.subplots(2, 2, figsize=(11, 8))
    ax_xy = axes[0, 0]
    ax_xy.plot(gt[:, 0], gt[:, 1], label="ground truth", color="#00bcd4", linewidth=2)
    ax_xy.plot(est_aligned[:, 0], est_aligned[:, 1], label="basalt", color="#ff9800", linewidth=2)
    ax_xy.set_title("XY Trajectory")
    ax_xy.set_xlabel("x [m]")
    ax_xy.set_ylabel("y [m]")
    ax_xy.axis("equal")
    ax_xy.legend()
    ax_xy.grid(True, alpha=0.3)

    labels = ["x", "y", "z"]
    for ax, idx in zip([axes[0, 1], axes[1, 0], axes[1, 1]], range(3)):
        ax.plot(times - times[0], gt[:, idx], label="ground truth", color="#00bcd4", linewidth=1.8)
        ax.plot(times - times[0], est_aligned[:, idx], label="basalt", color="#ff9800", linewidth=1.8)
        ax.set_title(f"{labels[idx]} vs Time")
        ax.set_xlabel("time [s]")
        ax.set_ylabel(f"{labels[idx]} [m]")
        ax.grid(True, alpha=0.3)
    plt.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def plot_error(times, err, out_path: Path):
    dist = np.linalg.norm(err, axis=1)
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    axes[0].plot(times - times[0], dist, color="#ef5350", linewidth=2)
    axes[0].set_title("Translational Error Magnitude")
    axes[0].set_ylabel("error [m]")
    axes[0].grid(True, alpha=0.3)

    colors = ["#42a5f5", "#66bb6a", "#ffa726"]
    labels = ["dx", "dy", "dz"]
    for idx in range(3):
        axes[1].plot(times - times[0], err[:, idx], color=colors[idx], label=labels[idx], linewidth=1.7)
    axes[1].set_title("Per-Axis Error")
    axes[1].set_xlabel("time [s]")
    axes[1].set_ylabel("error [m]")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)
    plt.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def main():
    if len(sys.argv) != 5:
        print(
            "usage: compare_tum_trajectories.py <groundtruth.tum> <estimate.tum> <traj_overlay.pdf> <error_plot.pdf>",
            file=sys.stderr,
        )
        return 2

    gt_path = Path(sys.argv[1])
    est_path = Path(sys.argv[2])
    traj_plot = Path(sys.argv[3])
    err_plot = Path(sys.argv[4])

    gt_t, gt_p = load_tum(gt_path)
    est_t, est_p = load_tum(est_path)
    times, gt_match, est_match = match_by_time(gt_t, gt_p, est_t, est_p)
    r, t = rigid_align(est_match, gt_match)
    est_aligned = (r @ est_match.T).T + t
    err = est_aligned - gt_match
    m = metrics(err)

    print()
    print("Summary view:")
    print(f"  matched samples: {len(times)}")
    print(f"  RMSE:            {m['rmse']:.6f} m")
    print(f"  mean abs error:  {m['mae']:.6f} m")
    print(f"  max error:       {m['max']:.6f} m")
    print(f"  latest error:    {m['latest']:.6f} m")
    print(
        f"  mean axis bias:  ({m['bias_x']:.6f}, {m['bias_y']:.6f}, {m['bias_z']:.6f}) m"
    )
    print(
        f"  axis RMSE:       ({m['rmse_x']:.6f}, {m['rmse_y']:.6f}, {m['rmse_z']:.6f}) m"
    )

    traj_plot.parent.mkdir(parents=True, exist_ok=True)
    plot_overlay(times, gt_match, est_aligned, traj_plot)
    plot_error(times, err, err_plot)

    print()
    print("Plot view:")
    print(f"  trajectory overlay: {traj_plot}")
    print(f"  error plot:         {err_plot}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
