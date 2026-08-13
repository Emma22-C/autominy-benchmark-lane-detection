#!/usr/bin/env python3
# Copyright 2026 J. E. Vidal-Cuevas, F. R. Trejo-Macotela et al.
# Licensed under the Apache License, Version 2.0 (the "License").

"""Generate comparison plots between N algorithm runs.

Usage:
    ./plot_comparison.py \\
        --csv FCM=/tmp/fcm.csv \\
        --csv "Sliding Windows"=/tmp/sw.csv \\
        --csv Hough=/tmp/hough.csv \\
        --output-dir plots/

Produces (under output-dir):
    time_boxplot.{pdf,png}
    time_hist.{pdf,png}
    dev_boxplot.{pdf,png}
    dev_timeseries.{pdf,png}
    angle_timeseries.{pdf,png}
    detection_status.{pdf,png}
    trajectory_map.{pdf,png}        (if gt_x_m and gt_y_m are available)
"""

import argparse
import os
import sys
from collections import OrderedDict
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


plt.rcParams.update({
    "font.family": "serif",
    "font.size": 10,
    "axes.titlesize": 11,
    "axes.labelsize": 10,
    "legend.fontsize": 9,
    "xtick.labelsize": 9,
    "ytick.labelsize": 9,
    "figure.dpi": 100,
    "savefig.dpi": 200,
    "savefig.bbox": "tight",
})

# Paleta IEEE-friendly: rojo, azul, verde, naranja, púrpura, gris.
# Funciona para hasta 6 algoritmos y mantiene contraste en grayscale.
PALETTE = [
    "#C0392B",  # rojo (FCM por convención del proyecto)
    "#2874A6",  # azul (Sliding Windows)
    "#27AE60",  # verde (Hough)
    "#E67E22",  # naranja
    "#7D3C98",  # púrpura
    "#566573",  # gris
]


def parse_csv_arg(value: str):
    if "=" not in value:
        raise argparse.ArgumentTypeError(
            f"--csv must be LABEL=PATH (got: {value!r})")
    label, path = value.split("=", 1)
    label = label.strip()
    path = path.strip()
    if not label or not path:
        raise argparse.ArgumentTypeError(
            f"--csv must have non-empty LABEL and PATH (got: {value!r})")
    if not os.path.isfile(path):
        raise argparse.ArgumentTypeError(f"CSV not found: {path}")
    return (label, path)


def load(path: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    numeric_cols = [
        "t_sec", "status", "center_deviation_px", "angle_deg", "time_ms",
        "gt_yaw_deg", "gt_x_m", "gt_y_m", "angle_error_deg",
        "xb_left", "xb_right", "fpc_left", "fpc_right",
    ]
    for c in numeric_cols:
        if c in df.columns:
            df[c] = pd.to_numeric(df[c], errors="coerce")
    return df


def save_fig(fig, output_dir: Path, name: str) -> None:
    for ext in ("pdf", "png"):
        path = output_dir / f"{name}.{ext}"
        fig.savefig(path)
        print(f"  -> {path}")
    plt.close(fig)


def colors_for(labels):
    return [PALETTE[i % len(PALETTE)] for i in range(len(labels))]


def _boxplot_compat(ax, data, labels, **kwargs):
    """Wrapper para boxplot que funciona en matplotlib viejo y nuevo.

    En matplotlib >= 3.9, el kwarg correcto es `tick_labels`.
    En versiones anteriores, era `labels`. matplotlib 3.9+ deprecó
    `labels` pero todavía lo acepta; matplotlib < 3.9 no conoce
    `tick_labels` y falla.
    """
    try:
        return ax.boxplot(data, tick_labels=labels, **kwargs)
    except TypeError:
        return ax.boxplot(data, labels=labels, **kwargs)


# --------------------------------------------------------------------
# Plots
# --------------------------------------------------------------------

def plot_time_boxplot(dfs: "OrderedDict", output_dir: Path) -> None:
    labels = list(dfs.keys())
    data = [dfs[lbl]["time_ms"].dropna().values for lbl in labels]
    cols = colors_for(labels)
    fig, ax = plt.subplots(figsize=(5.5, 3.5))
    bp = _boxplot_compat(
        ax, data, labels=labels,
        patch_artist=True, widths=0.5,
    )
    for patch, c in zip(bp["boxes"], cols):
        patch.set_facecolor(c)
        patch.set_alpha(0.6)
    ax.set_ylabel("Processing time per frame (ms)")
    ax.set_title("Per-frame processing time")
    ax.grid(axis="y", linestyle=":", alpha=0.5)
    save_fig(fig, output_dir, "time_boxplot")


def plot_time_hist(dfs: "OrderedDict", output_dir: Path) -> None:
    labels = list(dfs.keys())
    cols = colors_for(labels)
    all_t = np.concatenate(
        [dfs[lbl]["time_ms"].dropna().values for lbl in labels])
    if all_t.size == 0:
        print("WARN: no time data; skipping time_hist")
        return
    bins = np.linspace(np.percentile(all_t, 1), np.percentile(all_t, 99), 40)

    fig, ax = plt.subplots(figsize=(6.0, 3.5))
    for lbl, c in zip(labels, cols):
        t = dfs[lbl]["time_ms"].dropna().values
        if t.size == 0:
            continue
        ax.hist(
            t, bins=bins, alpha=0.5, color=c, label=lbl,
            edgecolor="black", linewidth=0.3)
    ax.set_xlabel("Processing time per frame (ms)")
    ax.set_ylabel("Frame count")
    ax.set_title("Distribution of processing time")
    ax.legend()
    ax.grid(axis="y", linestyle=":", alpha=0.5)
    save_fig(fig, output_dir, "time_hist")


def plot_dev_boxplot(dfs: "OrderedDict", output_dir: Path) -> None:
    labels = list(dfs.keys())
    cols = colors_for(labels)
    data = [
        dfs[lbl].loc[dfs[lbl]["status"] != 0, "center_deviation_px"]
            .abs().dropna().values
        for lbl in labels
    ]
    fig, ax = plt.subplots(figsize=(5.5, 3.5))
    bp = _boxplot_compat(
        ax, data, labels=labels,
        patch_artist=True, widths=0.5, showfliers=True,
    )
    for patch, c in zip(bp["boxes"], cols):
        patch.set_facecolor(c)
        patch.set_alpha(0.6)
    ax.set_ylabel("$|$Center deviation$|$ (px, IPM)")
    ax.set_title("Lateral deviation from camera center")
    ax.grid(axis="y", linestyle=":", alpha=0.5)
    save_fig(fig, output_dir, "dev_boxplot")


def plot_dev_timeseries(dfs: "OrderedDict", output_dir: Path) -> None:
    labels = list(dfs.keys())
    cols = colors_for(labels)
    fig, ax = plt.subplots(figsize=(7.0, 3.5))
    for lbl, c in zip(labels, cols):
        df = dfs[lbl]
        ax.plot(
            df["t_sec"], df["center_deviation_px"].abs(),
            color=c, label=lbl, linewidth=0.9, alpha=0.85)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("$|$Center deviation$|$ (px)")
    ax.set_title("Lateral deviation over time")
    ax.legend()
    ax.grid(linestyle=":", alpha=0.5)
    save_fig(fig, output_dir, "dev_timeseries")


def plot_angle_timeseries(dfs: "OrderedDict", output_dir: Path) -> None:
    labels = list(dfs.keys())
    cols = colors_for(labels)
    fig, ax = plt.subplots(figsize=(7.0, 3.5))
    for lbl, c in zip(labels, cols):
        df = dfs[lbl]
        ax.plot(
            df["t_sec"], df["angle_deg"],
            color=c, label=lbl, linewidth=0.9, alpha=0.85)
    ax.axhline(
        90.0, color="black", linestyle="--", linewidth=0.7, alpha=0.5,
        label="Straight (90$^\\circ$)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Estimated angle (deg)")
    ax.set_title("Lane heading estimate over time")
    ax.legend()
    ax.grid(linestyle=":", alpha=0.5)
    save_fig(fig, output_dir, "angle_timeseries")


def plot_detection_status(dfs: "OrderedDict", output_dir: Path) -> None:
    labels = list(dfs.keys())
    n = len(labels)
    labels_status = {0: "None", 1: "Left only", 2: "Right only", 3: "Both"}
    colors_status = {0: "#888888", 1: "#F39C12", 2: "#27AE60", 3: "#2E86C1"}

    def counts(df):
        c = df["status"].value_counts(normalize=True).to_dict()
        return [c.get(k, 0.0) * 100.0 for k in (0, 1, 2, 3)]

    by_algo = [counts(dfs[lbl]) for lbl in labels]

    fig, ax = plt.subplots(figsize=(max(5.5, 1.8 * n + 2), 3.5))
    x_pos = np.arange(n)
    width = 0.5
    bottoms = np.zeros(n)
    for k in (0, 1, 2, 3):
        heights = np.array([by_algo[i][k] for i in range(n)])
        ax.bar(
            x_pos, heights, width, bottom=bottoms,
            color=colors_status[k], edgecolor="black", linewidth=0.4,
            label=labels_status[k])
        bottoms += heights
    ax.set_xticks(x_pos)
    ax.set_xticklabels(labels)
    ax.set_ylabel("Frames (%)")
    ax.set_title("Detection status distribution")
    ax.legend(loc="center left", bbox_to_anchor=(1.02, 0.5))
    ax.grid(axis="y", linestyle=":", alpha=0.5)
    save_fig(fig, output_dir, "detection_status")


def plot_trajectory_map(dfs: "OrderedDict", output_dir: Path) -> None:
    """Plot the GT trajectory of the car colored by each algorithm's
    |center_deviation|. Addresses Reviewer K's request that 'the
    vehicle's trajectory will be recorded over a sufficient distance to
    evaluate the shape of its path'."""
    labels = list(dfs.keys())

    have_gt = all(
        ("gt_x_m" in dfs[lbl].columns) and ("gt_y_m" in dfs[lbl].columns) and
        dfs[lbl][["gt_x_m", "gt_y_m"]].dropna().shape[0] > 5
        for lbl in labels
    )
    if not have_gt:
        print("INFO: ground-truth (x,y) not available in all CSVs; "
              "skipping trajectory_map")
        return

    n = len(labels)
    fig, axes = plt.subplots(1, n, figsize=(4.5 * n, 4.0), squeeze=False)
    for ax, lbl in zip(axes[0], labels):
        df = dfs[lbl].dropna(subset=["gt_x_m", "gt_y_m"])
        if df.empty:
            ax.set_title(f"{lbl} (no GT data)")
            continue
        # Color por magnitud de desviación
        c = df["center_deviation_px"].abs().fillna(0.0).values
        sc = ax.scatter(
            df["gt_x_m"], df["gt_y_m"],
            c=c, s=4, cmap="viridis", vmin=0,
            vmax=np.percentile(c[c > 0], 95) if (c > 0).any() else 1.0)
        ax.set_xlabel("$x$ (m)")
        ax.set_ylabel("$y$ (m)")
        ax.set_title(f"{lbl}: trajectory")
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(linestyle=":", alpha=0.3)
        fig.colorbar(sc, ax=ax, label="$|$dev$|$ (px)", shrink=0.8)

    fig.suptitle("Ground-truth trajectory colored by detection error", y=1.02)
    fig.tight_layout()
    save_fig(fig, output_dir, "trajectory_map")


# --------------------------------------------------------------------
# Main
# --------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument(
        "--csv", action="append", type=parse_csv_arg, required=True,
        help="LABEL=PATH (repeatable).")
    ap.add_argument("--output-dir", default="plots",
                    help="Output directory for plots")
    args = ap.parse_args()

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    dfs = OrderedDict()
    for label, path in args.csv:
        dfs[label] = load(path)
        print(f"Loaded {label}: {len(dfs[label])} rows")

    print(f"Generating plots in {output_dir}/...")
    plot_time_boxplot(dfs, output_dir)
    plot_time_hist(dfs, output_dir)
    plot_dev_boxplot(dfs, output_dir)
    plot_dev_timeseries(dfs, output_dir)
    plot_angle_timeseries(dfs, output_dir)
    plot_detection_status(dfs, output_dir)
    plot_trajectory_map(dfs, output_dir)
    print("Done.")


if __name__ == "__main__":
    main()
