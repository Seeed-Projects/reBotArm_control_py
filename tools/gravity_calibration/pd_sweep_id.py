"""PD-sweep gravity identification for one joint (safe: PD always in command).

Servos the target joint slowly across a range with pure PD (no feedforward).
Quasi-static holding torque tau(q) = kp*(tgt - q) - kd*vel_est is logged along
the sweep (up AND down to cancel friction by averaging), then fitted to
tau(q) = A*sin(q + phi) + C.

Comparing (A, phi) against the model's gravity profile gives:
  - the motor-zero -> URDF-zero OFFSET for this joint (phi mismatch)
  - the effective mass*g*lever error (A mismatch)

Safety: PD in command at all times (kp bounded), velocity abort via
finite-difference of mechPos (mechVel 0x701A scale unverified — logged only),
torque implicit bound kp*err_max, freeze-not-drop on abort.

Usage: .demo/bin/python pd_sweep_id.py --joint joint3 --range 2.2 --speed 0.12
"""

import argparse
import json
import signal
import sys
import time
from pathlib import Path

import numpy as np
import pinocchio as pin
from motorbridge import Controller, Mode

VENDOR_URDF = str(Path(__file__).resolve().parents[2] / "urdf/RS/urdf/ReBot_Arm_RS.urdf")
MECH_POS, MECH_VEL = 0x7019, 0x701A
MODELS = ["rs-06", "rs-06", "rs-06", "rs-00", "rs-00", "rs-00"]
JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

KP = {"joint2": 30.0, "joint3": 20.0, "joint4": 10.0, "joint5": 8.0}
KD = {"joint2": 1.5, "joint3": 1.2, "joint4": 0.6, "joint5": 0.5}
HOLD = {2: (25.0, 1.5), 3: (20.0, 1.2), 4: (8.0, 0.5), 5: (6.0, 0.4)}   # motor id -> kp,kd
VEL_ABORT = 1.2       # rad/s, finite-difference
RATE = 40.0

ap = argparse.ArgumentParser()
ap.add_argument("--joint", default="joint3", choices=JOINTS[1:5])
ap.add_argument("--range", type=float, default=2.2, help="sweep extent from start, rad (positive = away from stop)")
ap.add_argument("--speed", type=float, default=0.12, help="rad/s")
ap.add_argument("--pre", action="append", default=[],
                help="pre-position a joint before the sweep, e.g. --pre joint3=0.8 (repeatable)")
ap.add_argument("--tag", default="", help="suffix for the output filename")
args = ap.parse_args()
J = JOINTS.index(args.joint)
PRE = {}
for spec in args.pre:
    name, val = spec.split("=")
    PRE[JOINTS.index(name)] = float(val)
    assert JOINTS.index(name) + 1 in HOLD and name != args.joint

_model = pin.buildModelFromUrdf(VENDOR_URDF)
_data = _model.createData()
HOLD_FF_CLAMP = {2: 18.0, 3: 10.0, 4: 2.5, 5: 1.5}

def g_of(q6):
    q = np.zeros(_model.nq)
    q[:6] = q6
    return pin.computeGeneralizedGravity(_model, _data, q)[:6]

ctrl = Controller("can0")
motors = [ctrl.add_robstride_motor(mid, 0xFD, mdl) for mid, mdl in zip(range(1, 7), MODELS)]
active = motors[J]
hold_ids = [i for i in HOLD if i != J + 1]

def rp(m, idx):
    return m.robstride_get_param_f32(idx)

def read_q6():
    return np.array([rp(m, MECH_POS) for m in motors])

state = {"log": [], "hold_q": None}

def hold_ff(q6):
    g = g_of(q6)
    return {mid: float(np.clip(g[mid-1], -HOLD_FF_CLAMP[mid], HOLD_FF_CLAMP[mid])) for mid in hold_ids}

def command_holds(q6):
    ff = hold_ff(q6)
    for mid in hold_ids:
        motors[mid-1].send_mit(float(state["hold_q"][mid-1]), 0.0, *HOLD[mid], ff[mid])

def freeze(why, q_now):
    print(f"\nABORT: {why} — freeze + lower", flush=True)
    kp, kd = KP[args.joint], KD[args.joint]
    for _ in range(int(5 * RATE)):
        q6 = read_q6()
        active.send_mit(float(q_now), 0.0, kp, kd, 0.0)
        command_holds(q6)
        time.sleep(1/RATE)
    lower(q_now)
    dump()
    sys.exit(2)

def lower(q_from):
    kp, kd = KP[args.joint], KD[args.joint]
    steps = max(1, int(abs(q_from - 0.03) / args.speed * RATE))
    for tgt in np.linspace(q_from, 0.03, steps):
        q6 = read_q6()
        active.send_mit(float(tgt), 0.0, kp, kd, 0.0)
        command_holds(q6)
        time.sleep(1/RATE)
    if PRE:  # return pre-positioned joints to their original pose before fade
        start_hold = {i: state["hold_q"][i] for i in PRE}
        n = int(max(abs(start_hold[i] - state["q0"][i]) for i in PRE) / 0.12 * RATE) + 1
        for k in np.linspace(0.0, 1.0, n):
            q6 = read_q6()
            ff = hold_ff(q6)
            for mid in hold_ids:
                i = mid - 1
                tgt_i = start_hold[i] + k * (state["q0"][i] - start_hold[i]) if i in PRE else state["hold_q"][i]
                motors[mid-1].send_mit(float(tgt_i), 0.0, *HOLD[mid], ff[mid])
            active.send_mit(0.03, 0.0, kp, kd, 0.0)
            time.sleep(1/RATE)
        for i in PRE:
            state["hold_q"][i] = state["q0"][i]
    for k in np.linspace(1, 0, int(2 * RATE)):
        q6 = read_q6()
        ff = hold_ff(q6)
        active.send_mit(float(q6[J]), 0.0, k * kp, kd, 0.0)
        for mid in hold_ids:
            motors[mid-1].send_mit(float(q6[mid-1]), 0.0, k * HOLD[mid][0], HOLD[mid][1], k * ff[mid])
        time.sleep(1/RATE)
    for m in motors:
        try: m.disable()
        except Exception: pass
    print("all motors disabled", flush=True)

def dump():
    tag = f"_{args.tag}" if args.tag else ""
    out = Path(__file__).parent / f"pdsweep_{args.joint}{tag}_{int(time.time())}.json"
    json.dump({"joint": args.joint, "kp": KP[args.joint], "kd": KD[args.joint],
               "samples": state["log"]}, open(out, "w"), indent=1)
    print("WROTE", out, flush=True)
    return out

signal.signal(signal.SIGINT, lambda *_: freeze("SIGINT", rp(active, MECH_POS)))

q0 = np.array([rp(m, MECH_POS) for m in motors])
state["hold_q"] = q0.copy()
state["q0"] = q0.copy()
print(f"=== PD SWEEP {args.joint} (motor {J+1}) range +{args.range} rad @ {args.speed} rad/s ===")
print("start pose:", np.round(q0, 3), flush=True)

for mid in hold_ids + [J + 1]:
    motors[mid-1].ensure_mode(Mode.MIT, 1000)
for mid in hold_ids + [J + 1]:
    motors[mid-1].enable()
print("enabled:", hold_ids + [J + 1], flush=True)

if PRE:
    print(f"pre-positioning: { {JOINTS[i]: v for i, v in PRE.items()} }", flush=True)
    t_pre0 = time.time()
    n_steps = int(max(abs(v - q0[i]) for i, v in PRE.items()) / 0.12 * RATE) + 1
    for k in np.linspace(0.0, 1.0, n_steps):
        q6 = read_q6()
        ff = hold_ff(q6)
        ramp = min(1.0, (time.time() - t_pre0) / 2.0)  # soft-start off the table
        for mid in hold_ids:
            i = mid - 1
            tgt_i = q0[i] + k * (PRE[i] - q0[i]) if i in PRE else q0[i]
            motors[mid-1].send_mit(float(tgt_i), 0.0, *HOLD[mid], ramp * ff[mid])
        motors[J].send_mit(float(q0[J]), 0.0, KP[args.joint], KD[args.joint], 0.0)
        time.sleep(1/RATE)
    for i, v in PRE.items():
        state["hold_q"][i] = v
    q6 = read_q6()
    print(f"pre-position done, pose {np.round(q6,3)}", flush=True)

kp, kd = KP[args.joint], KD[args.joint]
dt = 1 / RATE
q_prev, t_prev = float(q0[J]), time.time()

# up then down
path = list(np.arange(q0[J], q0[J] + args.range, args.speed * dt)) \
     + list(np.arange(q0[J] + args.range, q0[J] + 0.02, -args.speed * dt))
try:
    for i, tgt in enumerate(path):
        q6 = read_q6()
        q = float(q6[J])
        t = time.time()
        vel_fd = (q - q_prev) / max(t - t_prev, 1e-4)
        if abs(vel_fd) > VEL_ABORT and i > 10:
            freeze(f"vel_fd {vel_fd:+.2f} rad/s", q)
        vel_param = rp(active, MECH_VEL)
        tau_est = kp * (tgt - q) - kd * vel_fd
        active.send_mit(float(tgt), 0.0, kp, kd, 0.0)
        command_holds(q6)
        state["log"].append({"t": round(t, 4), "tgt": round(float(tgt), 5),
                             "q": round(float(q), 5), "vel_fd": round(float(vel_fd), 4),
                             "vel_param": round(float(vel_param), 4),
                             "tau_est": round(float(tau_est), 4),
                             "q6": [round(float(v), 5) for v in q6],
                             "dir": 1 if i < len(path) / 2 else -1})
        q_prev, t_prev = q, t
        time.sleep(max(0.0, dt - 0.004))
finally:
    lower(rp(active, MECH_POS))

out = dump()

# quick fit: tau(q) = A sin(q + phi) + C, averaging up/down passes
s = state["log"]
qs = np.array([x["q"] for x in s]); taus = np.array([x["tau_est"] for x in s])
M = np.column_stack([np.sin(qs), np.cos(qs), np.ones_like(qs)])
a, b, c = np.linalg.lstsq(M, taus, rcond=None)[0]
A = float(np.hypot(a, b)); phi = float(np.arctan2(b, a))
print(f"\nFIT tau(q_motor) = {A:.3f}·sin(q {'+' if phi>=0 else '-'} {abs(phi):.3f}) {'+' if c>=0 else '-'} {abs(c):.3f}  [N·m]")
print(f"  A (m·g·l efectivo) = {A:.3f} N·m | phi (offset-related) = {phi:+.3f} rad | C (fricción/bias) = {c:+.3f}")
# scale check of vel_param
v_fd = np.array([x["vel_fd"] for x in s]); v_pa = np.array([x["vel_param"] for x in s])
mask = np.abs(v_fd) > 0.03
if mask.sum() > 10:
    ratio = np.median(v_pa[mask] / v_fd[mask])
    print(f"  mechVel(0x701A)/dq_dt median ratio = {ratio:+.2f}  (1.0 => rad/s)")
