#!/usr/bin/env python3
# Copyright 2026 J. E. Vidal-Cuevas, F. R. Trejo-Macotela et al.
# Licensed under the Apache License, Version 2.0 (the "License").

"""Generate the LaTeX comparison table from N algorithm CSVs.

Usage:
    ./generate_latex_table.py \\
        --csv FCM=/tmp/fcm.csv \\
        --csv "Sliding Windows"=/tmp/sw.csv \\
        --csv Hough=/tmp/hough.csv \\
        --output table_comparison.tex

Each --csv argument is LABEL=PATH. The label is used as the column
header in the generated table. Order in the command line is the order
of columns.

Statistical tests:
  * With 2 algorithms: paired Wilcoxon signed-rank test (FCM vs each
    other algorithm).
  * With 3+ algorithms: Friedman test across all algorithms, and
    pairwise Wilcoxon FCM-vs-X with Bonferroni correction.

The table reports mean, std and P95 of processing time; mean and std
of |center deviation| and angle; detection rate; both-sides-detection
rate; and (only for the FCM column) Xie-Beni and FPC.
"""

import argparse
import os
import sys
from collections import OrderedDict

import numpy as np
import pandas as pd
from scipy import stats


# --------------------------------------------------------------------
# CSV parsing & helpers
# --------------------------------------------------------------------

def parse_csv_arg(value: str):
    """Parse a 'LABEL=PATH' argument."""
    if "=" not in value:
        raise argparse.ArgumentTypeError(
            f"--csv must be in the form LABEL=PATH (got: {value!r})")
    label, path = value.split("=", 1)
    label = label.strip()
    path = path.strip()
    if not label or not path:
        raise argparse.ArgumentTypeError(
            f"--csv must have non-empty LABEL and PATH (got: {value!r})")
    if not os.path.isfile(path):
        raise argparse.ArgumentTypeError(f"CSV not found: {path}")
    return (label, path)


def load_csv(path: str) -> pd.DataFrame:
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


def _safe_series(series: pd.Series) -> pd.Series:
    s = pd.to_numeric(series, errors="coerce")
    return s.replace([np.inf, -np.inf], np.nan).dropna()


def safe_mean(s: pd.Series) -> float:
    s = _safe_series(s)
    return float(s.mean()) if not s.empty else float("nan")


def safe_std(s: pd.Series) -> float:
    s = _safe_series(s)
    return float(s.std(ddof=1)) if len(s) > 1 else float("nan")


def safe_percentile(s: pd.Series, q: float) -> float:
    s = _safe_series(s)
    return float(s.quantile(q)) if not s.empty else float("nan")


def detection_rate(df: pd.DataFrame) -> float:
    if "status" not in df.columns or df.empty:
        return float("nan")
    return float((df["status"] != 0).mean() * 100.0)


def both_sides_rate(df: pd.DataFrame) -> float:
    if "status" not in df.columns or df.empty:
        return float("nan")
    return float((df["status"] == 3).mean() * 100.0)


def compute_summary(df: pd.DataFrame, name: str) -> dict:
    s = {"name": name, "n_frames": len(df)}
    s["detection_rate_pct"] = detection_rate(df)
    s["both_sides_pct"] = both_sides_rate(df)
    s["time_mean_ms"] = safe_mean(df.get("time_ms", pd.Series(dtype=float)))
    s["time_std_ms"] = safe_std(df.get("time_ms", pd.Series(dtype=float)))
    s["time_p95_ms"] = safe_percentile(df.get("time_ms", pd.Series(dtype=float)), 0.95)
    s["fps_mean"] = (
        1000.0 / s["time_mean_ms"]
        if isinstance(s["time_mean_ms"], float) and s["time_mean_ms"] > 0
        else float("nan")
    )
    s["center_dev_mean_abs"] = safe_mean(
        df.get("center_deviation_px", pd.Series(dtype=float)).abs())
    s["center_dev_std"] = safe_std(
        df.get("center_deviation_px", pd.Series(dtype=float)))
    s["angle_mean"] = safe_mean(df.get("angle_deg", pd.Series(dtype=float)))
    s["angle_std"] = safe_std(df.get("angle_deg", pd.Series(dtype=float)))
    s["xb_left_mean"] = safe_mean(df.get("xb_left", pd.Series(dtype=float)))
    s["xb_right_mean"] = safe_mean(df.get("xb_right", pd.Series(dtype=float)))
    s["fpc_left_mean"] = safe_mean(df.get("fpc_left", pd.Series(dtype=float)))
    s["fpc_right_mean"] = safe_mean(df.get("fpc_right", pd.Series(dtype=float)))
    return s


# --------------------------------------------------------------------
# Statistical tests
# --------------------------------------------------------------------

def align_runs(dfs: OrderedDict, column: str) -> np.ndarray:
    """Return an array of shape (N_min_frames, N_algos) aligned by
    sequence index, restricted to rows where status != 0 in all algos.

    Aligning by sequence index assumes the same rosbag was replayed for
    each algorithm. This is the recommended way to collect data.
    """
    arrays = []
    for label, df in dfs.items():
        a = df[df["status"] != 0][[column]].dropna().reset_index(drop=True)
        arrays.append(a[column].astype(float).values)
    if not arrays:
        return np.zeros((0, 0))
    n_min = min(len(a) for a in arrays)
    if n_min == 0:
        return np.zeros((0, len(arrays)))
    aligned = np.column_stack([a[:n_min] for a in arrays])
    return aligned


def friedman_test(dfs: OrderedDict, column: str):
    """Friedman test across N >= 3 algorithms on a given column."""
    aligned = align_runs(dfs, column)
    if aligned.shape[1] < 3 or aligned.shape[0] < 5:
        return {"stat": float("nan"), "p": float("nan"),
                "n": aligned.shape[0]}
    try:
        stat, p = stats.friedmanchisquare(*[aligned[:, i] for i in range(aligned.shape[1])])
    except (ValueError, IndexError):
        return {"stat": float("nan"), "p": float("nan"),
                "n": aligned.shape[0]}
    return {"stat": float(stat), "p": float(p), "n": aligned.shape[0]}


def pairwise_wilcoxon_vs_first(dfs: OrderedDict, column: str):
    """Wilcoxon paired tests: first algorithm in dfs vs each other.
    Bonferroni-corrected p-values."""
    labels = list(dfs.keys())
    if len(labels) < 2:
        return {}
    aligned = align_runs(dfs, column)
    if aligned.shape[0] < 5:
        return {labels[i]: {"stat": float("nan"), "p": float("nan"),
                            "p_bonf": float("nan"), "n": aligned.shape[0]}
                for i in range(1, len(labels))}

    n_comparisons = len(labels) - 1
    out = {}
    a_ref = aligned[:, 0]
    for i in range(1, len(labels)):
        b = aligned[:, i]
        diff = a_ref - b
        if np.all(np.abs(diff) < 1e-12):
            out[labels[i]] = {"stat": 0.0, "p": 1.0, "p_bonf": 1.0,
                              "n": len(a_ref)}
            continue
        try:
            stat, p = stats.wilcoxon(a_ref, b)
        except ValueError:
            out[labels[i]] = {"stat": float("nan"), "p": float("nan"),
                              "p_bonf": float("nan"), "n": len(a_ref)}
            continue
        p_bonf = min(1.0, p * n_comparisons)
        out[labels[i]] = {
            "stat": float(stat),
            "p": float(p),
            "p_bonf": float(p_bonf),
            "n": len(a_ref),
        }
    return out


# --------------------------------------------------------------------
# LaTeX rendering
# --------------------------------------------------------------------

def fmt(value, decimals: int = 3) -> str:
    if value is None or (isinstance(value, float) and (np.isnan(value) or np.isinf(value))):
        return "--"
    return f"{value:.{decimals}f}"


def fmt_p(p) -> str:
    if p is None or (isinstance(p, float) and np.isnan(p)):
        return "--"
    if p < 0.001:
        return r"$<$0.001"
    return f"{p:.3f}"


def write_latex_table(summaries: OrderedDict, friedman: dict, output: str) -> None:
    labels = list(summaries.keys())
    n_algos = len(labels)
    # Columnas: Metric | algo1 | algo2 | ... | (omnibus column si N>=3)
    if n_algos >= 3:
        col_spec = "l" + "c" * n_algos + "c"
        header_extra = " & " + r"\textbf{Friedman $p$}"
    else:
        col_spec = "l" + "c" * n_algos + "c"
        header_extra = " & " + r"\textbf{Wilcoxon $p$}"

    L = []
    L.append(r"% Auto-generated by generate_latex_table.py")
    L.append(r"% Comparison of " + ", ".join(labels) + " on identical input.")
    L.append(r"\begin{table*}[!t]")
    L.append(r"\renewcommand{\arraystretch}{1.2}")
    L.append(r"\caption{Quantitative comparison of " + " vs ".join(labels) +
             r" for lane detection on the AutoMiny v4.0 simulated environment. "
             r"All algorithms receive identical input (same IPM, same Canny, "
             r"same ROIs). For $N\geq 3$ algorithms the omnibus column reports "
             r"the Friedman test $p$-value across all algorithms; pairwise "
             r"differences against the first column are tested with "
             r"Bonferroni-corrected Wilcoxon signed-rank tests (shown as "
             r"superscripts where applicable).}")
    L.append(r"\label{tab:comparison}")
    L.append(r"\centering")
    L.append(r"\begin{tabular}{" + col_spec + "}")
    L.append(r"\hline")
    L.append(
        r"\textbf{Metric} & " +
        " & ".join(rf"\textbf{{{lbl}}}" for lbl in labels) +
        header_extra + r" \\")
    L.append(r"\hline")

    def row(metric_name, values, p_value):
        cells = [metric_name] + [v for v in values] + [fmt_p(p_value)]
        return " & ".join(cells) + r" \\"

    # Tiempo
    L.append(row(
        "Processing time mean (ms)",
        [fmt(summaries[lbl]["time_mean_ms"], 2) for lbl in labels],
        friedman["time_ms"]["p"]))
    L.append(row(
        "Processing time std (ms)",
        [fmt(summaries[lbl]["time_std_ms"], 2) for lbl in labels],
        None))
    L.append(row(
        "Processing time P95 (ms)",
        [fmt(summaries[lbl]["time_p95_ms"], 2) for lbl in labels],
        None))
    L.append(row(
        "Mean FPS",
        [fmt(summaries[lbl]["fps_mean"], 1) for lbl in labels],
        None))
    L.append(r"\hline")
    # Detección
    L.append(row(
        r"Detection rate (\%)",
        [fmt(summaries[lbl]["detection_rate_pct"], 1) for lbl in labels],
        None))
    L.append(row(
        r"Both lanes detected (\%)",
        [fmt(summaries[lbl]["both_sides_pct"], 1) for lbl in labels],
        None))
    L.append(r"\hline")
    # Desviación / ángulo
    L.append(row(
        r"Mean $|$center deviation$|$ (px)",
        [fmt(summaries[lbl]["center_dev_mean_abs"], 2) for lbl in labels],
        friedman["center_deviation_px"]["p"]))
    L.append(row(
        r"Center deviation std (px)",
        [fmt(summaries[lbl]["center_dev_std"], 2) for lbl in labels],
        None))
    L.append(row(
        r"Mean estimated angle (deg)",
        [fmt(summaries[lbl]["angle_mean"], 2) for lbl in labels],
        friedman["angle_deg"]["p"]))
    L.append(row(
        r"Angle std (deg)",
        [fmt(summaries[lbl]["angle_std"], 2) for lbl in labels],
        None))
    L.append(r"\hline")
    # Métricas internas: solo FCM
    fcm_label = None
    for lbl in labels:
        if "fcm" in lbl.lower():
            fcm_label = lbl
            break

    def fcm_cell(field, decimals=3):
        if fcm_label is None:
            return ["--"] * n_algos
        return [
            fmt(summaries[lbl][field], decimals) if lbl == fcm_label else "N/A"
            for lbl in labels
        ]

    L.append(row("Xie--Beni (left, mean)", fcm_cell("xb_left_mean"), None))
    L.append(row("Xie--Beni (right, mean)", fcm_cell("xb_right_mean"), None))
    L.append(row("FPC (left, mean)", fcm_cell("fpc_left_mean"), None))
    L.append(row("FPC (right, mean)", fcm_cell("fpc_right_mean"), None))
    L.append(r"\hline")
    # Frames
    L.append(row(
        "Frames analyzed",
        [str(summaries[lbl]["n_frames"]) for lbl in labels],
        None))
    L.append(r"\hline")
    L.append(r"\end{tabular}")
    L.append(r"\end{table*}")

    with open(output, "w") as f:
        f.write("\n".join(L) + "\n")
    print(f"Wrote LaTeX table to {output}")


# --------------------------------------------------------------------
# Pairwise table (optional, useful as Table 2 in the paper)
# --------------------------------------------------------------------

def write_pairwise_table(
    summaries: OrderedDict,
    pairwise: dict,
    output: str,
) -> None:
    """Write a supplementary table with pairwise Wilcoxon tests
    (FCM vs each other algorithm), Bonferroni-corrected."""
    labels = list(summaries.keys())
    if len(labels) < 2:
        return
    ref = labels[0]
    others = labels[1:]

    L = []
    L.append(r"% Auto-generated by generate_latex_table.py")
    L.append(r"\begin{table}[!t]")
    L.append(r"\renewcommand{\arraystretch}{1.2}")
    L.append(rf"\caption{{Pairwise Wilcoxon signed-rank tests of {ref} against "
             r"each baseline (Bonferroni-corrected). Smaller $p$ indicates a "
             r"significant per-frame difference between the two algorithms.}")
    L.append(r"\label{tab:pairwise}")
    L.append(r"\centering")
    L.append(r"\begin{tabular}{lccc}")
    L.append(r"\hline")
    L.append(rf"\textbf{{Comparison vs {ref}}} & "
             r"\textbf{Time $p$} & \textbf{$|$Dev$|$ $p$} & "
             r"\textbf{Angle $p$} \\")
    L.append(r"\hline")
    for lbl in others:
        p_t = pairwise["time_ms"].get(lbl, {}).get("p_bonf", float("nan"))
        p_d = pairwise["center_deviation_px"].get(lbl, {}).get("p_bonf", float("nan"))
        p_a = pairwise["angle_deg"].get(lbl, {}).get("p_bonf", float("nan"))
        L.append(f"{lbl} & {fmt_p(p_t)} & {fmt_p(p_d)} & {fmt_p(p_a)} \\\\")
    L.append(r"\hline")
    L.append(r"\end{tabular}")
    L.append(r"\end{table}")

    with open(output, "w") as f:
        f.write("\n".join(L) + "\n")
    print(f"Wrote pairwise LaTeX table to {output}")


# --------------------------------------------------------------------
# Main
# --------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument(
        "--csv", action="append", type=parse_csv_arg, required=True,
        help="LABEL=PATH (repeatable). First --csv becomes the reference "
             "for pairwise post-hoc tests; should typically be FCM.")
    ap.add_argument("--output", default="table_comparison.tex",
                    help="Output LaTeX file for the main table")
    ap.add_argument("--output-pairwise", default=None,
                    help="Optional second output file for the pairwise table")
    args = ap.parse_args()

    if not args.csv:
        ap.error("At least one --csv required")

    # Preservar el orden dado por el usuario
    dfs = OrderedDict()
    for label, path in args.csv:
        dfs[label] = load_csv(path)

    summaries = OrderedDict()
    for label, df in dfs.items():
        summaries[label] = compute_summary(df, label)

    # Test omnibus por métrica
    friedman = {
        "time_ms": friedman_test(dfs, "time_ms"),
        "center_deviation_px": friedman_test(dfs, "center_deviation_px"),
        "angle_deg": friedman_test(dfs, "angle_deg"),
    }
    # Si solo hay 2 algoritmos, Friedman no es aplicable; usar Wilcoxon
    # pareado directo y mostrarlo como "Wilcoxon p".
    if len(dfs) == 2:
        labels = list(dfs.keys())
        for col in ("time_ms", "center_deviation_px", "angle_deg"):
            pw = pairwise_wilcoxon_vs_first(dfs, col)
            if labels[1] in pw:
                friedman[col]["p"] = pw[labels[1]]["p"]
                friedman[col]["n"] = pw[labels[1]]["n"]

    pairwise = {
        "time_ms": pairwise_wilcoxon_vs_first(dfs, "time_ms"),
        "center_deviation_px": pairwise_wilcoxon_vs_first(dfs, "center_deviation_px"),
        "angle_deg": pairwise_wilcoxon_vs_first(dfs, "angle_deg"),
    }

    # Resumen en consola
    print("=" * 70)
    print(f"  Algorithms: {list(dfs.keys())}")
    for lbl, s in summaries.items():
        print(f"  [{lbl}]  n={s['n_frames']}  "
              f"det={s['detection_rate_pct']:.1f}%  "
              f"time={s['time_mean_ms']:.2f}±{s['time_std_ms']:.2f}ms  "
              f"|dev|={s['center_dev_mean_abs']:.2f}px")
    for metric, res in friedman.items():
        print(f"  Omnibus({metric}): p={res['p']:.4f}  n={res['n']}")
    print("=" * 70)

    write_latex_table(summaries, friedman, args.output)
    if args.output_pairwise is not None and len(dfs) >= 2:
        write_pairwise_table(summaries, pairwise, args.output_pairwise)


if __name__ == "__main__":
    main()
