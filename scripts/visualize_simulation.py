#!/usr/bin/env python3
"""Animate a simulation.json mission log produced by the CLI's Main.cpp.

Usage:
    python3 scripts/visualize_simulation.py [path/to/simulation.json] [--save out.mp4]
"""
import argparse
import json
import math
import sys

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

STATE_COLORS = {
    "STOPPED": "gray",
    "TURNING": "orange",
    "ACCELERATING": "gold",
    "MOVING": "tab:blue",
    "DECELERATING": "tab:red",
}


def load_steps(path):
    with open(path) as f:
        data = json.load(f)
    return data["simulation"]["steps"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path", nargs="?", default="simulation.json")
    ap.add_argument("--save", help="write animation to this file instead of showing it (e.g. out.mp4 or out.gif)")
    ap.add_argument("--interval", type=int, default=60, help="ms between animation frames")
    ap.add_argument("--stride", type=int, default=1, help="keep every Nth step (speeds up --save on slow machines)")
    args = ap.parse_args()

    steps = load_steps(args.path)[::args.stride]
    if not steps:
        sys.exit("no steps in " + args.path)

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_aspect("equal")

    all_ids = sorted({t["id"] for s in steps for t in s.get("targets", [])})
    id_colors = {tid: c for tid, c in zip(all_ids, plt.cm.tab10.colors)}

    xs = [s["position"]["x"] for s in steps] + [t["x"] for s in steps for t in s.get("targets", [])]
    ys = [s["position"]["y"] for s in steps] + [t["y"] for s in steps for t in s.get("targets", [])]
    pad = 20
    ax.set_xlim(min(xs) - pad, max(xs) + pad)
    ax.set_ylim(min(ys) - pad, max(ys) + pad)

    drone_path, = ax.plot([], [], "-", color="tab:blue", lw=1, alpha=0.4, label="drone path")

    drone_dot, = ax.plot([], [], "o", color="tab:blue", ms=10, label="drone")
    target_dots = {tid: ax.plot([], [], "o", color=id_colors[tid], ms=7)[0] for tid in all_ids}
    locked_ring, = ax.plot([], [], "o", mfc="none", mec="black", ms=14, mew=2, label="locked target")
    fire_dot, = ax.plot([], [], "x", color="black", ms=10, mew=2, label="fire point (dropPoint)")
    predicted_dot, = ax.plot([], [], "^", color="green", ms=8, label="predicted target @ impact")
    impact_dot, = ax.plot([], [], "*", color="purple", ms=12, label="aim point (real-time impact)")

    heading_arrow = ax.annotate("", xy=(0, 0), xytext=(0, 0), arrowprops=dict(arrowstyle="->", color="tab:blue", lw=2))

    title = ax.set_title("")
    handles = [drone_dot, locked_ring, fire_dot, predicted_dot, impact_dot] + list(target_dots.values())
    labels = ["drone", "locked target", "fire point (dropPoint)", "predicted target @ impact", "aim point (real-time impact)"] + [
        f"target {tid}" for tid in all_ids
    ]
    ax.legend(handles, labels, loc="upper left", fontsize=7)

    dxs, dys = [], []

    def update(i):
        s = steps[i]
        pos = s["position"]
        drop, aim, pred = s["dropPoint"], s["aimPoint"], s["predictedTarget"]

        dxs.append(pos["x"]); dys.append(pos["y"])
        drone_path.set_data(dxs, dys)

        drone_dot.set_data([pos["x"]], [pos["y"]])
        drone_dot.set_color(STATE_COLORS.get(s["state"], "tab:blue"))

        for tid, dot in target_dots.items():
            dot.set_data([], [])
        for t in s.get("targets", []):
            target_dots[t["id"]].set_data([t["x"]], [t["y"]])
            if t["id"] == s["targetIndex"]:
                locked_ring.set_data([t["x"]], [t["y"]])

        fire_dot.set_data([drop["x"]], [drop["y"]])
        predicted_dot.set_data([pred["x"]], [pred["y"]])
        impact_dot.set_data([aim["x"]], [aim["y"]])

        heading_arrow.xy = (pos["x"] + 10 * math.cos(s["direction"]),
                             pos["y"] + 10 * math.sin(s["direction"]))
        heading_arrow.xyann = (pos["x"], pos["y"])

        title.set_text(f"t={s['timeSecSinceStart']:.2f}s  state={s['state']}  target={s['targetIndex']}  speed={s['currentSpeed']:.1f}")

        return (drone_path, drone_dot, locked_ring, fire_dot, predicted_dot, impact_dot, heading_arrow, title,
                *target_dots.values())

    anim = FuncAnimation(fig, update, frames=len(steps), interval=args.interval, blit=False, repeat=False)

    if args.save:
        anim.save(args.save)
        print("saved", args.save)
    else:
        plt.show()


if __name__ == "__main__":
    main()
