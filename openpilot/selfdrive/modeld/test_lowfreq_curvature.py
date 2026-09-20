"""The control-relevant test: desired CURVATURE, not path position.

`get_curvature_from_plan` is what modeld hands the lateral controller. Position error of a few mm can
still be a meaningful curvature error, because curvature is a derivative -- so compare the quantity the
car actually steers on.
"""
import sys, numpy as np, importlib.util as iu
sys.path.insert(0, "/home/jimmy/Videos/openpilot")
from openpilot.tools.lib.logreader import LogReader
from openpilot.selfdrive.modeld.constants import ModelConstants, Plan
from openpilot.selfdrive.controls.lib.drive_helpers import get_curvature_from_plan
_HERE = __import__("os").path.dirname(__import__("os").path.abspath(__file__))
sp = iu.spec_from_file_location("lowfreq", _HERE + "/lowfreq.py")
lowfreq = iu.module_from_spec(sp); sp.loader.exec_module(lowfreq)

DT, LAT_T = 0.05, 0.3
lr = LogReader(sys.argv[1])
plans, vs = [], []
for m in lr:
    if m.which() == "modelV2" and len(m.modelV2.position.x):
        p, o, vel = m.modelV2.position, m.modelV2.orientation, m.modelV2.velocity
        pl = np.zeros((ModelConstants.IDX_N, ModelConstants.PLAN_WIDTH), np.float32)
        pl[:, Plan.POSITION] = np.stack([p.x, p.y, p.z], -1)
        pl[:, Plan.VELOCITY] = np.stack([vel.x, vel.y, vel.z], -1)
        pl[:, Plan.T_FROM_CURRENT_EULER] = np.stack([o.x, o.y, o.z], -1)
        pl[:, Plan.ORIENTATION_RATE] = np.stack([m.modelV2.orientationRate.x, m.modelV2.orientationRate.y,
                                                 m.modelV2.orientationRate.z], -1)
        plans.append(pl); vs.append(max(float(vel.x[0]), 1.0))
def curv(pl, v, t):
    return get_curvature_from_plan(pl[:, Plan.T_FROM_CURRENT_EULER][:, 2], pl[:, Plan.ORIENTATION_RATE][:, 2],
                                   ModelConstants.T_IDXS, v, t)
print(f"{len(plans)} plans, v mean {np.mean(vs):.1f} m/s")
for n in (1, 2, 3):
    h, a = [], []
    for k in range(0, len(plans) - n - 1, 5):
        v = vs[k + n]
        truth = curv(plans[k + n], v, LAT_T)
        h.append(abs(curv(plans[k], v, LAT_T) - truth))
        a.append(abs(curv(lowfreq.age({"plan": plans[k][None]}, n * DT)["plan"][0], v, LAT_T) - truth))
    h, a = np.array(h), np.array(a)
    print(f"  dt={n*DT*1e3:3.0f} ms | HOLD curv err mean {h.mean():.6f} p95 {np.percentile(h,95):.6f} "
          f"| AGE mean {a.mean():.6f} p95 {np.percentile(a,95):.6f} 1/m  -> {100*(1-a.mean()/max(h.mean(),1e-12)):+.0f}%")
