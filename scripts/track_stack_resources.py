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


import argparse
import base64
import csv
import html
import io
import json
import os
import shutil
import signal
import subprocess
import time
from collections import defaultdict
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib-codex")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import psutil


TARGETS = {
    "px4": [
        "px4_sitl_default/bin/px4",
        "/bin/px4",
    ],
    "basalt": [
        "ros2 launch basalt_wrapper basalt_node.launch.py",
        "/basalt_wrapper/lib/basalt_wrapper/basalt_node",
    ],
    "map_location": [
        "ros2 run geo_correction_tools map_location_node",
        "/geo_correction_tools/lib/geo_correction_tools/map_location_node",
    ],
    "geo_px4_bridge": [
        "ros2 launch px4_basalt_bridge px4_basalt_bridge.launch.py",
        "px4_basalt_bridge_node",
    ],
    "ground_truth_localizer": [
        "gz_ground_truth_odometry.py",
    ],
    "px4_imu_bridge": [
        "px4_sensor_combined_to_imu.py",
    ],
}

EXCLUDE_PATTERNS = (
    "gz sim",
    "gazebo",
    "rviz2",
    "ros_gz_image",
    "ros_gz_bridge",
    "MicroXRCEAgent",
    "track_stack_resources.py",
    "record_geo_mapping_dataset.sh",
    "run_geo_mapping_bag_replay.sh",
)


@dataclass
class SampleRow:
    t_sec: float
    group: str
    pid_count: int
    cpu_pct: float
    rss_mib: float
    gpu_mem_mib: float
    gpu_sm_pct: float | None
    read_mibps: float
    write_mibps: float


def parse_args():
    parser = argparse.ArgumentParser(
        description="Track CPU/GPU/MEM/storage usage for PX4/VIO/geo-localization processes."
    )
    parser.add_argument(
        "--out-dir",
        default=None,
        help="Output directory for CSV, summary JSON, and HTML report.",
    )
    parser.add_argument(
        "--interval-sec",
        type=float,
        default=1.0,
        help="Sampling interval in seconds.",
    )
    parser.add_argument(
        "--title",
        default="PX4 / VIO / Geo Resource Report",
        help="Title shown in the HTML dashboard.",
    )
    return parser.parse_args()


def default_out_dir():
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path.home() / "basalt_wrapper" / "resource_reports" / f"stack_resources_{stamp}"


def process_cmdline(proc: psutil.Process) -> str:
    try:
        cmdline = proc.cmdline()
        if cmdline:
            return " ".join(cmdline)
        return proc.name()
    except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
        return ""


def classify_process(cmdline: str) -> str | None:
    lowered = cmdline.lower()
    if lowered.strip() == "px4":
        return "px4"
    for pattern in EXCLUDE_PATTERNS:
        if pattern.lower() in lowered:
            return None
    for group, patterns in TARGETS.items():
        for pattern in patterns:
            if pattern.lower() in lowered:
                return group
    return None


def safe_io_counters(proc: psutil.Process):
    try:
        io = proc.io_counters()
        return int(io.read_bytes), int(io.write_bytes)
    except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess, AttributeError):
        return 0, 0


def query_gpu_by_pid():
    if shutil.which("nvidia-smi") is None:
        return {}

    gpu_mem = {}
    gpu_sm = {}

    try:
        result = subprocess.run(
            [
                "nvidia-smi",
                "--query-compute-apps=pid,used_gpu_memory",
                "--format=csv,noheader,nounits",
            ],
            check=False,
            capture_output=True,
            text=True,
        )
        if result.returncode == 0:
            for line in result.stdout.splitlines():
                parts = [p.strip() for p in line.split(",")]
                if len(parts) >= 2 and parts[0].isdigit():
                    gpu_mem[int(parts[0])] = float(parts[1])
    except Exception:
        pass

    try:
        result = subprocess.run(
            ["nvidia-smi", "pmon", "-c", "1", "-s", "um"],
            check=False,
            capture_output=True,
            text=True,
        )
        if result.returncode == 0:
            for line in result.stdout.splitlines():
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                parts = line.split()
                if len(parts) < 6:
                    continue
                pid = parts[1]
                sm = parts[3]
                if pid.isdigit() and sm not in ("-", "N/A"):
                    try:
                        gpu_sm[int(pid)] = float(sm)
                    except ValueError:
                        pass
    except Exception:
        pass

    out = {}
    for pid in set(gpu_mem) | set(gpu_sm):
        out[pid] = {
            "gpu_mem_mib": gpu_mem.get(pid, 0.0),
            "gpu_sm_pct": gpu_sm.get(pid),
        }
    return out


def as_png_data_uri(fig):
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=150, bbox_inches="tight")
    plt.close(fig)
    encoded = base64.b64encode(buf.getvalue()).decode("ascii")
    return f"data:image/png;base64,{encoded}"


def build_chart(rows, title, y_label, keys, colors):
    fig, ax = plt.subplots(figsize=(10, 4.2))
    if not rows:
        ax.text(0.5, 0.5, "No samples", ha="center", va="center")
        ax.axis("off")
        return as_png_data_uri(fig)

    x = [row["t_sec"] for row in rows]
    for key, color in zip(keys, colors):
        ax.plot(x, [row.get(key, 0.0) for row in rows], linewidth=1.8, color=color, label=key)
    ax.set_title(title)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel(y_label)
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best")
    fig.tight_layout()
    return as_png_data_uri(fig)


def fmt(value, suffix=""):
    if value is None:
        return "n/a"
    return f"{value:.2f}{suffix}"


def summarize_group(rows):
    if not rows:
        return None
    cpu = [r.cpu_pct for r in rows]
    rss = [r.rss_mib for r in rows]
    gpu_mem = [r.gpu_mem_mib for r in rows]
    gpu_sm = [r.gpu_sm_pct for r in rows if r.gpu_sm_pct is not None]
    read = [r.read_mibps for r in rows]
    write = [r.write_mibps for r in rows]
    pids = [r.pid_count for r in rows]
    return {
        "samples": len(rows),
        "cpu_avg_pct": sum(cpu) / len(cpu),
        "cpu_peak_pct": max(cpu),
        "rss_peak_mib": max(rss),
        "gpu_mem_peak_mib": max(gpu_mem),
        "gpu_sm_peak_pct": max(gpu_sm) if gpu_sm else None,
        "read_peak_mibps": max(read),
        "write_peak_mibps": max(write),
        "pid_peak_count": max(pids),
    }


def render_report(out_dir: Path, title: str, rows: list[SampleRow], meta: dict):
    report_dir = out_dir / "report"
    report_dir.mkdir(parents=True, exist_ok=True)
    rows_by_group = defaultdict(list)
    totals = []

    grouped_times = defaultdict(
        lambda: {
            "cpu_pct": 0.0,
            "rss_mib": 0.0,
            "gpu_mem_mib": 0.0,
            "gpu_sm_pct": 0.0,
            "read_mibps": 0.0,
            "write_mibps": 0.0,
        }
    )

    for row in rows:
        rows_by_group[row.group].append(row)
        bucket = grouped_times[row.t_sec]
        bucket["cpu_pct"] += row.cpu_pct
        bucket["rss_mib"] += row.rss_mib
        bucket["gpu_mem_mib"] += row.gpu_mem_mib
        bucket["gpu_sm_pct"] += row.gpu_sm_pct or 0.0
        bucket["read_mibps"] += row.read_mibps
        bucket["write_mibps"] += row.write_mibps

    for t_sec in sorted(grouped_times):
        entry = {"t_sec": t_sec}
        entry.update(grouped_times[t_sec])
        totals.append(entry)

    cpu_chart = build_chart(
        totals,
        "Tracked CPU Usage",
        "CPU [% of one core summed across tracked processes]",
        ["cpu_pct"],
        ["#2563eb"],
    )
    mem_chart = build_chart(
        totals,
        "Tracked Memory Usage",
        "RSS [MiB]",
        ["rss_mib", "gpu_mem_mib"],
        ["#16a34a", "#7c3aed"],
    )
    io_chart = build_chart(
        totals,
        "Tracked Disk Throughput",
        "MiB/s",
        ["read_mibps", "write_mibps"],
        ["#d97706", "#dc2626"],
    )

    group_cards = []
    for group in TARGETS:
        summary = summarize_group(rows_by_group.get(group, []))
        if summary is None:
            continue
        group_cards.append(
            f"""
            <tr>
              <th>{html.escape(group)}</th>
              <td>{summary['samples']}</td>
              <td>{fmt(summary['cpu_avg_pct'], '%')}</td>
              <td>{fmt(summary['cpu_peak_pct'], '%')}</td>
              <td>{fmt(summary['rss_peak_mib'], ' MiB')}</td>
              <td>{fmt(summary['gpu_mem_peak_mib'], ' MiB')}</td>
              <td>{fmt(summary['gpu_sm_peak_pct'], '%')}</td>
              <td>{fmt(summary['read_peak_mibps'], ' MiB/s')}</td>
              <td>{fmt(summary['write_peak_mibps'], ' MiB/s')}</td>
              <td>{summary['pid_peak_count']}</td>
            </tr>
            """
        )

    html_text = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>{html.escape(title)}</title>
  <style>
    body {{
      font-family: ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: linear-gradient(180deg, #f6f7fb 0%, #eef2ff 100%);
      color: #111827;
      margin: 0;
      padding: 24px;
    }}
    .wrap {{
      max-width: 1200px;
      margin: 0 auto;
    }}
    .hero {{
      background: white;
      border-radius: 18px;
      padding: 24px;
      box-shadow: 0 12px 34px rgba(15, 23, 42, 0.08);
      margin-bottom: 20px;
    }}
    .meta {{
      color: #475569;
      font-size: 14px;
      display: flex;
      gap: 18px;
      flex-wrap: wrap;
    }}
    .grid {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
      gap: 18px;
      margin-bottom: 18px;
    }}
    .card {{
      background: white;
      border-radius: 18px;
      padding: 18px;
      box-shadow: 0 12px 34px rgba(15, 23, 42, 0.08);
    }}
    table {{
      width: 100%;
      border-collapse: collapse;
      font-size: 14px;
    }}
    th, td {{
      padding: 10px 8px;
      border-bottom: 1px solid #e5e7eb;
      text-align: left;
    }}
    th {{
      color: #334155;
    }}
    img {{
      width: 100%;
      border-radius: 12px;
      background: #fff;
    }}
    code {{
      background: #eef2ff;
      padding: 2px 6px;
      border-radius: 6px;
    }}
  </style>
</head>
<body>
  <div class="wrap">
    <div class="hero">
      <h1>{html.escape(title)}</h1>
      <div class="meta">
        <span>started: {html.escape(meta['started_at'])}</span>
        <span>ended: {html.escape(meta['ended_at'])}</span>
        <span>interval: {meta['interval_sec']:.2f}s</span>
        <span>csv: <code>{html.escape(str(out_dir / 'resource_samples.csv'))}</code></span>
      </div>
      <p>Tracked groups only include PX4, Basalt/VIO, geo correction, geo feedback bridge, and related localization helpers. Gazebo, RViz, image bridges, and XRCE agent are intentionally excluded.</p>
    </div>

    <div class="grid">
      <div class="card"><h2>CPU</h2><img alt="CPU usage" src="{cpu_chart}"></div>
      <div class="card"><h2>Memory / GPU Memory</h2><img alt="Memory usage" src="{mem_chart}"></div>
      <div class="card"><h2>Disk IO</h2><img alt="Disk IO" src="{io_chart}"></div>
    </div>

    <div class="card">
      <h2>Per-Group Summary</h2>
      <table>
        <thead>
          <tr>
            <th>Group</th>
            <th>Samples</th>
            <th>CPU Avg</th>
            <th>CPU Peak</th>
            <th>RSS Peak</th>
            <th>GPU Mem Peak</th>
            <th>GPU SM Peak</th>
            <th>Read Peak</th>
            <th>Write Peak</th>
            <th>Max PIDs</th>
          </tr>
        </thead>
        <tbody>
          {''.join(group_cards) if group_cards else '<tr><td colspan="10">No tracked processes were captured.</td></tr>'}
        </tbody>
      </table>
    </div>
  </div>
</body>
</html>
"""
    (report_dir / "index.html").write_text(html_text)

    summary = {
        "title": title,
        "started_at": meta["started_at"],
        "ended_at": meta["ended_at"],
        "interval_sec": meta["interval_sec"],
        "groups": {
            group: summarize_group(rows_by_group.get(group, []))
            for group in TARGETS
            if summarize_group(rows_by_group.get(group, [])) is not None
        },
    }
    (out_dir / "resource_summary.json").write_text(json.dumps(summary, indent=2))


def main():
    args = parse_args()
    out_dir = Path(args.out_dir).expanduser() if args.out_dir else default_out_dir()
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / "resource_samples.csv"

    start_wall = datetime.now()
    start_mono = time.monotonic()
    stop_requested = False
    samples: list[SampleRow] = []
    previous_io = {}

    def handle_signal(_signum, _frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    print(f"Tracking stack resources into: {out_dir}")
    print("Press Ctrl+C after landing to finalize the HTML dashboard.")

    while not stop_requested:
        elapsed = time.monotonic() - start_mono
        gpu_info = query_gpu_by_pid()
        grouped = defaultdict(
            lambda: {
                "pid_count": 0,
                "cpu_pct": 0.0,
                "rss_mib": 0.0,
                "gpu_mem_mib": 0.0,
                "gpu_sm_pct": 0.0,
                "has_gpu_sm": False,
                "read_bytes": 0,
                "write_bytes": 0,
            }
        )

        for proc in psutil.process_iter(["pid", "name", "cmdline"]):
            cmdline = process_cmdline(proc)
            group = classify_process(cmdline)
            if group is None:
                continue
            try:
                cpu_pct = proc.cpu_percent(interval=None)
                mem = proc.memory_info().rss / (1024.0 * 1024.0)
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                continue

            read_bytes, write_bytes = safe_io_counters(proc)
            entry = grouped[group]
            entry["pid_count"] += 1
            entry["cpu_pct"] += float(cpu_pct)
            entry["rss_mib"] += float(mem)
            entry["read_bytes"] += read_bytes
            entry["write_bytes"] += write_bytes

            if proc.pid in gpu_info:
                entry["gpu_mem_mib"] += float(gpu_info[proc.pid].get("gpu_mem_mib", 0.0))
                gpu_sm = gpu_info[proc.pid].get("gpu_sm_pct")
                if gpu_sm is not None:
                    entry["gpu_sm_pct"] += float(gpu_sm)
                    entry["has_gpu_sm"] = True

        for group, entry in grouped.items():
            last_read, last_write, last_t = previous_io.get(group, (entry["read_bytes"], entry["write_bytes"], elapsed))
            dt = max(elapsed - last_t, 1e-6)
            read_mibps = max(entry["read_bytes"] - last_read, 0) / (1024.0 * 1024.0 * dt)
            write_mibps = max(entry["write_bytes"] - last_write, 0) / (1024.0 * 1024.0 * dt)
            previous_io[group] = (entry["read_bytes"], entry["write_bytes"], elapsed)
            samples.append(
                SampleRow(
                    t_sec=elapsed,
                    group=group,
                    pid_count=entry["pid_count"],
                    cpu_pct=entry["cpu_pct"],
                    rss_mib=entry["rss_mib"],
                    gpu_mem_mib=entry["gpu_mem_mib"],
                    gpu_sm_pct=entry["gpu_sm_pct"] if entry["has_gpu_sm"] else None,
                    read_mibps=read_mibps,
                    write_mibps=write_mibps,
                )
            )

        time.sleep(args.interval_sec)

    with csv_path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "t_sec",
                "group",
                "pid_count",
                "cpu_pct",
                "rss_mib",
                "gpu_mem_mib",
                "gpu_sm_pct",
                "read_mibps",
                "write_mibps",
            ]
        )
        for row in samples:
            writer.writerow(
                [
                    f"{row.t_sec:.3f}",
                    row.group,
                    row.pid_count,
                    f"{row.cpu_pct:.3f}",
                    f"{row.rss_mib:.3f}",
                    f"{row.gpu_mem_mib:.3f}",
                    "" if row.gpu_sm_pct is None else f"{row.gpu_sm_pct:.3f}",
                    f"{row.read_mibps:.6f}",
                    f"{row.write_mibps:.6f}",
                ]
            )

    render_report(
        out_dir,
        args.title,
        samples,
        {
            "started_at": start_wall.isoformat(timespec="seconds"),
            "ended_at": datetime.now().isoformat(timespec="seconds"),
            "interval_sec": args.interval_sec,
        },
    )

    print(f"Wrote resource CSV: {csv_path}")
    print(f"Wrote HTML report: {out_dir / 'report' / 'index.html'}")


if __name__ == "__main__":
    main()
