"""Automatic single-joint gravity float test (option B — no hands needed).

Sequence (target joint, default joint3/elbow):
  1. Enable hold-PD on the neighbor joints (j2, j4, j5) at their current pose
     with live g(q) feedforward. j1/j6 stay disabled (no gravity load).
  2. Auto-lift: servo the target joint from its current q to LIFT_TARGET at
     LIFT_SPEED with PD (KP_LIFT) + live g(q) ff.
  3. Float: fade KP to 0 over FADE_S (kd stays), keep tau = g(q) + slow
     integral (clamped). Log for FLOAT_S. The integral converges to the
     model's gravity error at that pose.
  4. Return: re-engage PD, servo back down, ramp off, disable all.

Aborts -> FREEZE (PD hold at current pose + g ff, NOT torque-off), report,
controlled lower after 10 s, disable. SIGINT does the same.

Sign convention: motor == vendor URDF (verified 2026-07-17 hand-lift probe:
motors 2/3 rest at their lower stop q=0 and lifting reads positive, matching
vendor URDF j2/j3 in [0, pi]).

Usage: .demo/bin/python auto_float_test.py --joint joint3 --masses pr3
       [--lift 0.9] [--float-s 12]
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
PR3_MASSES = {2: 1.552, 3: 1.252, 4: 0.46, 5: 0.20120457182895, 6: 0.1}  # link idx -> kg

MECH_POS, MECH_VEL = 0x7019, 0x701A
MODELS = ["rs-06", "rs-06", "rs-06", "rs-00", "rs-00", "rs-00"]
JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

TAU_CLAMP = {1: 6.0, 2: 20.0, 3: 10.0, 4: 3.0, 5: 2.0, 6: 1.0}   # motor id -> N·m
HOLD_KP = {2: 25.0, 4: 8.0, 5: 6.0}
HOLD_KD = {2: 1.5, 4: 0.5, 5: 0.4}
KP_LIFT = {"joint2": 30.0, "joint3": 18.0, "joint4": 8.0, "joint5": 6.0}
KD_ACT = {"joint2": 1.5, "joint3": 1.2, "joint4": 0.5, "joint5": 0.4}

LIFT_SPEED = 0.15    # rad/s
FADE_S = 3.0
RATE = 40.0
KI = 0.8
INT_CLAMP = 2.5
VEL_ABORT = 1.0      # rad/s
POS_WINDOW = 0.5     # rad during float

ap = argparse.ArgumentParser()
ap.add_argument("--joint", default="joint3", choices=JOINTS[1:5])
ap.add_argument("--masses", default="pr3", choices=["vendor", "pr3"])
ap.add_argument("--lift", type=float, default=0.9)
ap.add_argument("--float-s", type=float, default=12.0)
ap.add_argument("--k", type=float, default=1.0, help="gravity correction scale for active joint")
ap.add_argument("--c", type=float, default=0.0, help="gravity correction bias N·m")
ap.add_argument("--offset", type=float, default=0.0, help="zero offset rad added to active joint q before g(q)")
args = ap.parse_args()
J = JOINTS.index(args.joint)          # q index (0-based); motor id = J+1

model = pin.buildModelFromUrdf(VENDOR_URDF)
if args.masses == "pr3":
    for idx, m_new in PR3_MASSES.items():
        ine = model.inertias[idx]
        model.inertias[idx] = pin.Inertia(m_new, ine.lever, ine.inertia * (m_new / ine.mass))
    print("[model] PR#3 masses applied onto vendor kinematics")
data = model.createData()

def g_of(q6):
    q = np.zeros(model.nq)
    q[:6] = q6
    q[J] += args.offset
    g = pin.computeGeneralizedGravity(model, data, q)[:6]
    g[J] = args.k * g[J] + args.c
    return g

ctrl = Controller("can0")
motors = [ctrl.add_robstride_motor(mid, 0xFD, mdl) for mid, mdl in zip(range(1, 7), MODELS)]
active = motors[J]
hold_ids = [i for i in (2, 4, 5) if i != J + 1]     # motor ids in hold mode

_fail = 0
def read_q6():
    global _fail
    out = np.zeros(6)
    for i, m in enumerate(motors):
        try:
            out[i] = m.robstride_get_param_f32(MECH_POS)
            _fail = 0
        except Exception:
            _fail += 1
            if _fail >= 3:
                freeze_and_exit(None, "3 consecutive read failures")
            return None
    return out

state = {"q_hold": None, "log": [], "frozen": False}

def clampv(mid, tau):
    c = TAU_CLAMP[mid]
    return float(np.clip(tau, -c, c))

def command_holds(q6, g):
    for mid in hold_ids:
        i = mid - 1
        motors[i].send_mit(float(state["q_hold"][i]), 0.0, HOLD_KP[mid], HOLD_KD[mid], clampv(mid, float(g[i])))

def freeze_and_exit(q6, why):
    state["frozen"] = True
    print(f"\nABORT: {why} — FREEZING (PD hold, no torque-off)", flush=True)
    try:
        if q6 is None:
            q6 = np.array([m.robstride_get_param_f32(MECH_POS) for m in motors])
        g = g_of(q6)
        kp = KP_LIFT[args.joint]
        for _ in range(int(10 * RATE)):
            active.send_mit(float(q6[J]), 0.0, kp, KD_ACT[args.joint], clampv(J + 1, float(g[J])))
            command_holds(q6, g)
            time.sleep(1.0 / RATE)
        lower_and_disable(float(q6[J]))
    finally:
        dump_log()
        sys.exit(2)

def lower_and_disable(q_from):
    print("lowering to rest...", flush=True)
    kp = KP_LIFT[args.joint]
    kd = KD_ACT[args.joint]
    steps = max(1, int(abs(q_from - 0.05) / LIFT_SPEED * RATE))
    for tgt in np.linspace(q_from, 0.05, steps):
        q6 = read_q6()
        if q6 is None:
            continue
        g = g_of(q6)
        active.send_mit(float(tgt), 0.0, kp, kd, clampv(J + 1, float(g[J])))
        command_holds(q6, g)
        time.sleep(1.0 / RATE)
    for k in np.linspace(1.0, 0.0, int(2 * RATE)):   # torque fade at rest
        q6 = read_q6()
        if q6 is None:
            continue
        g = g_of(q6)
        active.send_mit(float(q6[J]), 0.0, k * kp, kd, clampv(J + 1, k * float(g[J])))
        for mid in hold_ids:
            i = mid - 1
            motors[i].send_mit(float(q6[i]), 0.0, k * HOLD_KP[mid], HOLD_KD[mid], clampv(mid, k * float(g[i])))
        time.sleep(1.0 / RATE)
    for m in motors:
        try:
            m.disable()
        except Exception:
            pass
    print("all motors disabled", flush=True)

def dump_log():
    out = Path(__file__).parent / f"autofloat_{args.joint}_{args.masses}_{int(time.time())}.json"
    json.dump({"joint": args.joint, "masses": args.masses, "samples": state["log"]}, open(out, "w"), indent=1)
    print("WROTE", out, flush=True)

signal.signal(signal.SIGINT, lambda *_: freeze_and_exit(None, "SIGINT"))

q6 = read_q6()
assert q6 is not None
state["q_hold"] = q6.copy()
g = g_of(q6)
print(f"=== AUTO FLOAT {args.joint} (motor {J+1}) masses={args.masses} lift={args.lift} rad ===")
print(f"start pose {np.round(q6,3)}  g(q) {np.round(g,3)}", flush=True)

# arm everything: modes first, then enable (per-motor)
for mid in hold_ids + [J + 1]:
    motors[mid - 1].ensure_mode(Mode.MIT, 1000)
for mid in hold_ids + [J + 1]:
    motors[mid - 1].enable()
print("motors enabled:", hold_ids + [J + 1], flush=True)

dt = 1.0 / RATE
kp = KP_LIFT[args.joint]
kd = KD_ACT[args.joint]
_fd = {"q": float(q6[J]), "t": time.time()}

def fd_vel(q6):
    """mechVel(0x701A) scale is bogus on this firmware — use finite difference."""
    t = time.time()
    v = (float(q6[J]) - _fd["q"]) / max(t - _fd["t"], 1e-4)
    _fd["q"], _fd["t"] = float(q6[J]), t
    return v

# ---- lift
tgt = float(q6[J])
t_lift0 = time.time()
while tgt < args.lift:
    tgt = min(args.lift, tgt + LIFT_SPEED * dt)
    q6r = read_q6()
    if q6r is None:
        continue
    q6 = q6r
    g = g_of(q6)
    vel = fd_vel(q6)
    if abs(vel) > VEL_ABORT:
        freeze_and_exit(q6, f"lift velocity {vel:+.2f}")
    # soft-start: the arm rests on the table at enable — ramp ff over 2 s
    ramp = min(1.0, (time.time() - t_lift0) / 2.0)
    active.send_mit(tgt, 0.0, kp, kd, clampv(J + 1, ramp * float(g[J])))
    command_holds(q6, g)
    state["log"].append({"phase": "lift", "t": time.time(), "q": float(q6[J]), "tgt": tgt, "vel": float(vel)})
    time.sleep(dt)
print(f"LIFT done: q={q6[J]:+.3f} (target {args.lift})", flush=True)

# ---- fade to float
t0 = time.time()
integral = 0.0
while time.time() - t0 < FADE_S:
    k = 1.0 - (time.time() - t0) / FADE_S
    q6r = read_q6()
    if q6r is None:
        continue
    q6 = q6r
    g = g_of(q6)
    vel = fd_vel(q6)
    if abs(vel) > VEL_ABORT:
        freeze_and_exit(q6, f"fade velocity {vel:+.2f}")
    active.send_mit(float(args.lift), 0.0, k * kp, kd, clampv(J + 1, float(g[J])))
    command_holds(q6, g)
    time.sleep(dt)
q_float = float(q6[J])
print(f"FLOAT start at q={q_float:+.3f}", flush=True)

# ---- float + integral trim
t0 = time.time()
while time.time() - t0 < args.float_s:
    q6r = read_q6()
    if q6r is None:
        continue
    q6 = q6r
    g = g_of(q6)
    vel = fd_vel(q6)
    if abs(vel) > VEL_ABORT:
        freeze_and_exit(q6, f"float velocity {vel:+.2f}")
    if abs(q6[J] - q_float) > POS_WINDOW:
        freeze_and_exit(q6, f"float window exceeded ({q6[J]-q_float:+.3f})")
    integral = float(np.clip(integral - KI * vel * dt, -INT_CLAMP, INT_CLAMP))
    tau = clampv(J + 1, float(g[J]) + integral)
    active.send_mit(float(q6[J]), 0.0, 0.0, kd, tau)
    command_holds(q6, g)
    state["log"].append({"phase": "float", "t": time.time(), "q": float(q6[J]),
                         "vel": float(vel), "tau_model": float(g[J]),
                         "integral": integral, "tau_sent": tau})
    time.sleep(dt)

fl = [s for s in state["log"] if s["phase"] == "float"]
drift = fl[-1]["q"] - fl[0]["q"]
tail = fl[-int(3 * RATE):]
res = float(np.mean([s["integral"] for s in tail]))
tm = float(np.mean([s["tau_model"] for s in tail]))
print(f"\nRESULT {args.joint} masses={args.masses}: pose q={fl[-1]['q']:+.3f}")
print(f"  drift {drift:+.4f} rad | tau_model {tm:+.3f} | residual {res:+.3f} N·m "
      f"({100*abs(res)/max(abs(tm),1e-6):.1f}% del modelo)", flush=True)

# ---- return
lower_and_disable(fl[-1]["q"])
dump_log()
