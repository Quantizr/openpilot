"""Does ageing a prediction actually beat repeating it? Measured against REAL modelV2 from demo rlogs.

At 5 Hz modeld must emit something on the 3 frames between real runs. Two candidates:
  HOLD : repeat the last message verbatim
  AGE  : lowfreq.age -- resample the trajectory at t+dt and move the origin to where the car drove
The rlog gives the ground truth: the model's OWN prediction made dt later. So for every 20 Hz pair
(k, k+n) we compare both candidates against frame k+n's real plan.
"""
import sys, numpy as np
sys.path.insert(0, "/home/jimmy/Videos/openpilot")
sys.path.insert(0, "/home/jimmy/Videos/openpilot")
from openpilot.tools.lib.logreader import LogReader
from openpilot.selfdrive.modeld.constants import ModelConstants, Plan
# import lowfreq BY PATH: both checkouts provide an `openpilot` package, and the old one wins on sys.path
import importlib.util as _iu
_HERE = __import__("os").path.dirname(__import__("os").path.abspath(__file__))
_sp = _iu.spec_from_file_location("lowfreq", _HERE + "/lowfreq.py")
lowfreq = _iu.module_from_spec(_sp); _sp.loader.exec_module(lowfreq)

DT = 0.05
lr = LogReader(sys.argv[1] if len(sys.argv) > 1 else "/home/jimmy/distill_cache/demo_rlogs/2.rlog.zst")
plans, vs, yaws = [], [], []
for m in lr:
    if m.which() == "modelV2":
        p = m.modelV2.position
        o = m.modelV2.orientation
        vel = m.modelV2.velocity
        if not len(p.x): continue
        pl = np.zeros((ModelConstants.IDX_N, ModelConstants.PLAN_WIDTH), np.float32)
        pl[:, Plan.POSITION] = np.stack([p.x, p.y, p.z], -1)
        pl[:, Plan.VELOCITY] = np.stack([vel.x, vel.y, vel.z], -1)
        pl[:, Plan.T_FROM_CURRENT_EULER] = np.stack([o.x, o.y, o.z], -1)
        plans.append(pl)
        vs.append(float(vel.x[0]) if len(vel.x) else 0.0)
        yaws.append(float(o.z[4]) if len(o.z) > 4 else 0.0)   # heading 0.5 s ahead: a turn proxy
import numpy as _np
print(f"{len(plans)} plans | v_ego mean {_np.mean(vs):.1f} m/s max {_np.max(vs):.1f} | "
      f"|yaw@0.5s| mean {_np.mean(_np.abs(yaws)):.4f} rad max {_np.max(_np.abs(yaws)):.4f}")
if _np.mean(vs) < 1.0: print("  ** SEGMENT IS ~PARKED -- any method looks perfect here **")
if len(plans) < 50: sys.exit("not enough")

res = {}
for n in (1, 2, 3):                       # 50, 100, 150 ms of staleness
    hold_e, age_e = [], []
    for k in range(0, len(plans) - n - 1, 5):
        out = {"plan": plans[k][None]}
        truth = plans[k + n][:, Plan.POSITION]
        hold = plans[k][:, Plan.POSITION]
        aged = lowfreq.age(out, n * DT)["plan"][0][:, Plan.POSITION]
        # lateral error over the control-relevant near field (first 2.5 s ~ first 12 idxs)
        hold_e.append(np.abs(hold[:12, 1] - truth[:12, 1]).mean())
        age_e.append(np.abs(aged[:12, 1] - truth[:12, 1]).mean())
    res[n] = (np.mean(hold_e), np.mean(age_e), np.median(hold_e), np.median(age_e))
    print(f"  dt={n*DT*1000:3.0f} ms  n={len(hold_e):4d} | HOLD lat err mean {res[n][0]:.4f} m  med {res[n][2]:.4f}"
          f"  | AGE mean {res[n][1]:.4f} m  med {res[n][3]:.4f}  -> {100*(1-res[n][1]/max(res[n][0],1e-9)):+.0f}%")
