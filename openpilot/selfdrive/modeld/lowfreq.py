"""Publish at 20 Hz from a model that only runs at 5 Hz, by ageing the last prediction forward.

The distilled cinque model costs up to ~150 ms of Adreno time, so it runs once every
``RUN_EVERY`` camera frames (5 Hz). Everything downstream of modeld -- controlsd, the UI, the logs --
still expects a 20 Hz ``modelV2``. Repeating the identical message four times would make the plan appear
to freeze for 200 ms and then jump, which the controller sees as a step in desired curvature.

Instead each republish AGES the prediction: the model already predicts a whole trajectory, so a
prediction made ``dt`` seconds ago still describes the future -- just from an origin the car has since
driven away from. Ageing = resample that trajectory at ``t + dt`` and re-express it in the car's CURRENT
frame. Between real runs the published path is therefore continuous, and at the moment a fresh
prediction lands the two agree to within the model's own error.

## What "re-express in the current frame" means, and why both parts are needed
The car has moved along its own predicted path during ``dt``. So for the plan we
  1. resample every time-indexed quantity at ``T_IDXS + dt``   (advance time), then
  2. subtract the position the car reached at ``dt`` and rotate by minus the yaw it turned through
     (move the origin).
Doing only (1) leaves the path starting ``v*dt`` metres behind the car -- a constant lateral/longitudinal
bias that grows with speed. Doing only (2) leaves a stale trajectory shape. In a turn the rotation is
what matters: at 0.2 rad/s and dt=0.2 s the heading is already 2.3 deg off, which at 50 m of lookahead
is ~2 m of lateral error.

## Lane lines and road edges are indexed by DISTANCE, not time
``lane_lines``/``road_edges`` are (y, z) sampled at ``X_IDXS`` metres ahead, so ageing them is a shift
along X by the distance travelled, plus the same lateral/heading correction. They are resampled on the
same grid so the message shape is unchanged.

## What is deliberately NOT aged
``meta``, ``desire_pred``, ``desire_state``, ``lane_lines_prob``, ``lead_prob`` are probabilities, and
``pose``/``road_transform``/``wide_from_device_euler`` are instantaneous estimates. Ageing a probability
has no defensible meaning, and the pose of a 200 ms old frame is not improved by extrapolation, so these
are passed through unchanged. ``lead`` IS time-indexed and is aged on ``LEAD_T_IDXS``.
⚠ This is the honest limit of the approach: between real runs those fields are up to 200 ms stale. The
alternative (freezing everything) is strictly worse, but "aged" is not "fresh".
"""
from __future__ import annotations

import numpy as np

from openpilot.selfdrive.modeld.constants import ModelConstants, Plan

T_IDXS = np.array(ModelConstants.T_IDXS, dtype=np.float64)
X_IDXS = np.array(ModelConstants.X_IDXS, dtype=np.float64)
LEAD_T = np.array(ModelConstants.LEAD_T_IDXS, dtype=np.float64)

# time-indexed, aged on their own grid; everything else is passed through (see the module docstring)
_TIME_FIELDS = {"plan": T_IDXS, "lead": LEAD_T}
_DIST_FIELDS = ("lane_lines", "road_edges")


def _interp_axis(y: np.ndarray, grid: np.ndarray, new_grid: np.ndarray, axis: int) -> np.ndarray:
  """np.interp along one axis, holding the endpoints (the model says nothing past 10 s / 192 m)."""
  yv = np.moveaxis(y, axis, 0)
  out = np.empty((len(new_grid),) + yv.shape[1:], dtype=y.dtype)
  flat, oflat = yv.reshape(len(grid), -1), out.reshape(len(new_grid), -1)
  for j in range(flat.shape[1]):
    oflat[:, j] = np.interp(new_grid, grid, flat[:, j])
  return np.moveaxis(out, 0, axis)


def _ego_at(plan: np.ndarray, dt: float) -> tuple[np.ndarray, float]:
  """Where the car is, and how far it has turned, ``dt`` into its own predicted plan."""
  pos = np.array([np.interp(dt, T_IDXS, plan[:, Plan.POSITION][:, i]) for i in range(3)])
  yaw = float(np.interp(dt, T_IDXS, plan[:, Plan.T_FROM_CURRENT_EULER][:, 2]))
  return pos, yaw


def _rot(yaw: float) -> np.ndarray:
  c, s = np.cos(-yaw), np.sin(-yaw)      # rotate INTO the new heading
  return np.array([[c, -s], [s, c]])


def age(out: dict[str, np.ndarray], dt: float) -> dict[str, np.ndarray]:
  """Return `out` as it would look `dt` seconds later, from the car's new position and heading."""
  if dt <= 0:
    return out
  aged = dict(out)
  plan = out["plan"][0].astype(np.float64)
  pos, yaw = _ego_at(plan, dt)
  R = _rot(yaw)

  # --- plan: advance time, then move the origin -------------------------------------------------
  p = _interp_axis(plan, T_IDXS, T_IDXS + dt, axis=0)
  p[:, Plan.POSITION] -= pos
  for sl in (Plan.POSITION, Plan.VELOCITY, Plan.ACCELERATION, Plan.ORIENTATION_RATE):
    v = p[:, sl]
    v[:, :2] = v[:, :2] @ R.T                        # x,y rotate; z is unaffected by a yaw turn
    p[:, sl] = v
  p[:, Plan.T_FROM_CURRENT_EULER][:, 2] -= yaw       # heading is now relative to the NEW heading
  aged["plan"] = p[None].astype(out["plan"].dtype)

  # --- lane lines / road edges: indexed by DISTANCE, so shift along X by how far we drove --------
  travelled = float(np.hypot(pos[0], pos[1]))
  for k in _DIST_FIELDS:
    if k not in out:
      continue
    a = out[k][0].astype(np.float64)                 # (N, IDX_N, 2) = (y, z) at X_IDXS
    b = _interp_axis(a, X_IDXS, X_IDXS + travelled, axis=1)
    xy = np.stack([np.broadcast_to(X_IDXS, b.shape[:2]) - travelled, b[..., 0]], -1)
    b[..., 0] = (xy @ R.T)[..., 1]                   # lateral offset after the heading change
    aged[k] = b[None].astype(out[k].dtype)

  # --- lead: time-indexed on its own grid -------------------------------------------------------
  if "lead" in out and out["lead"].ndim >= 3:
    lead = out["lead"][0].astype(np.float64)
    ax = lead.ndim - 2                               # (..., LEAD_T, WIDTH)
    if lead.shape[ax] == len(LEAD_T):
      aged["lead"] = _interp_axis(lead, LEAD_T, LEAD_T + dt, axis=ax)[None].astype(out["lead"].dtype)
  return aged
