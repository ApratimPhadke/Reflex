#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Plot an `rxctl bench` run: ball position against the setpoint, AI pilot vs fallback controller alone.

    python3 tools/bench_plot.py bench.csv [bench.md] -o bench.png [--title "..."]

The CSV comes from `rxctl bench ... --csv bench.csv`; the optional .md (the same run's report) is only
used to copy its summary table into the figure. Every number in the figure is read from these files.
"""
import argparse
import csv
import collections

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt   # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("md", nargs="?")
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--title", default="")
    a = ap.parse_args()

    runs = collections.defaultdict(lambda: ([], [], []))           # pilot -> (t, sp, x), steps laid end to end
    offset = collections.defaultdict(float)
    last = {}
    for r in csv.DictReader(open(a.csv)):
        p, step, t = r["pilot"], int(r["step"]), float(r["t_s"])
        if last.get(p, (step, 0.0))[0] != step:
            offset[p] += last[p][1]
        last[p] = (step, t)
        T, SP, X = runs[p]
        T.append(offset[p] + t); SP.append(float(r["sp_mm"])); X.append(float(r["x_mm"]))

    rows = []
    if a.md:
        for line in open(a.md):
            if line.startswith("| ") and ("settling" in line or "overshoot" in line or "RMS" in line or "interventions" in line) \
               and not line.startswith("| step"):
                cells = [c.strip() for c in line.strip().strip("|").split("|")]
                if len(cells) == 3:
                    rows.append(cells)

    fig, ax = plt.subplots(figsize=(11, 4.8))
    if "ai" in runs:
        T, SP, _ = runs["ai"]
        ax.plot(T, SP, color="#8b949e", lw=1.2, ls="--", label="setpoint")
    for p, col, lab in (("fallback", "#3fb950", "fallback controller alone"), ("ai", "#1f6feb", "AI pilot (guardian on)")):
        if p in runs:
            T, _, X = runs[p]
            ax.plot(T, X, color=col, lw=1.6, label=lab)
    ax.set_xlabel("time [s] (steps laid end to end)")
    ax.set_ylabel("ball position [mm]")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper right")
    ax.set_title(a.title or "Reflex: AI pilot vs fallback controller, same setpoint sequence")
    if rows:
        tbl = ax.table(cellText=[r[1:] for r in rows], rowLabels=[r[0] for r in rows],
                       colLabels=["AI pilot", "fallback alone"], loc="bottom", bbox=[0.18, -0.62, 0.64, 0.42])
        tbl.auto_set_font_size(False)
        tbl.set_fontsize(9)
        fig.subplots_adjust(bottom=0.38)
    fig.savefig(a.out, dpi=130, bbox_inches="tight")
    print("wrote", a.out)


if __name__ == "__main__":
    main()
