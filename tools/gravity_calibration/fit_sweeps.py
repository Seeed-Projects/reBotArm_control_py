"""Fit sweep data against the URDF gravity model.

For each pdsweep_*.json produced by pd_sweep_id.py, fits
    tau_measured ~= k * g_urdf(q + offset) + fric * direction + c
over a grid of zero offsets, and prints k / offset / bias / friction / rms.

k ~= 1, offset ~= 0, |c| < friction  =>  the URDF mass model is correct.

Optionally exclude contact-suspect samples with --qmin (drop samples where
the active joint is below the threshold, e.g. still near the table).

Usage:
  python fit_sweeps.py data/pdsweep_joint3_*.json [--qmin 0.4] [--urdf path]
"""

import argparse
import glob
import json
from pathlib import Path

import numpy as np
import pinocchio as pin

ap = argparse.ArgumentParser()
ap.add_argument("files", nargs="+", help="pdsweep_*.json (globs ok)")
ap.add_argument("--qmin", type=float, default=None,
                help="drop samples with active-joint q below this (contact region)")
ap.add_argument("--urdf", default=str(Path(__file__).resolve().parents[2]
                                       / "urdf/RS/urdf/ReBot_Arm_RS.urdf"))
ap.add_argument("--off-max", type=float, default=0.3, help="offset grid half-width [rad]")
args = ap.parse_args()

model = pin.buildModelFromUrdf(args.urdf)
data = model.createData()

def g_of(q6):
    q = np.zeros(model.nq)
    q[:6] = q6
    return pin.computeGeneralizedGravity(model, data, q)[:6]

files = [f for pat in args.files for f in sorted(glob.glob(pat))]
print(f"{'file':44s} {'n':>5s} {'off':>6s} {'k':>6s} {'c':>7s} {'fric':>6s} {'rms':>6s}")
for f in files:
    d = json.load(open(f))
    J = int(d["joint"][-1]) - 1
    s = [x for x in d["samples"] if "q6" in x
         and (args.qmin is None or x["q"] > args.qmin)]
    if len(s) < 50:
        print(f"{Path(f).name:44s} skipped (need q6-logged samples)")
        continue
    taus = np.array([x["tau_est"] for x in s])
    dirs = np.array([x["dir"] for x in s], dtype=float)
    Q6 = np.array([x["q6"] for x in s])
    best = None
    for off in np.arange(-args.off_max, args.off_max + 0.01, 0.02):
        Qo = Q6.copy()
        Qo[:, J] += off
        pred = np.array([g_of(q)[J] for q in Qo[::2]])
        M = np.column_stack([pred, dirs[::2], np.ones_like(pred)])
        coef, *_ = np.linalg.lstsq(M, taus[::2], rcond=None)
        rms = float(np.sqrt(np.mean((M @ coef - taus[::2]) ** 2)))
        if best is None or rms < best[0]:
            best = (rms, off, *coef)
    rms, off, k, fric, c = best
    print(f"{Path(f).name:44s} {len(s):5d} {off:+6.2f} {k:6.3f} {c:+7.3f} {fric:+6.3f} {rms:6.3f}")
