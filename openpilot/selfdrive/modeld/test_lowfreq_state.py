#!/usr/bin/env python3
"""Assert cinque's queues keep 20 Hz semantics when the model runs at 5 Hz.

Every failure here is SILENT at runtime -- the model just receives inputs it was never distilled on --
so each cadence property gets an explicit assertion.
"""
import sys
import numpy as np

sys.path.insert(0, "/home/jimmy/Videos/openpilot")
from openpilot.selfdrive.modeld.lowfreq_state import LowFreqState, DESIRE_WINDOW, FEAT_STRIDE

SHAPES = {"state_img_q": ((2, 5, 6, 8, 8), "uint8"),
          "state_desire_q": ((132, 1, 8), "float32"),
          "state_feat_q": ((128, 1, 16384), "float32")}
RUN_EVERY = 4


def fail(msg):
  print(f"  FAIL {msg}"); sys.exit(1)


st = LowFreqState(SHAPES, RUN_EVERY)
feats = []
for k in range(200):
  frame = np.full((2, 6, 8, 8), k % 251, dtype=np.uint8)          # frame id encoded in the pixels
  desire = np.zeros(8, np.float32)
  # a pulse landing BETWEEN model runs (k % RUN_EVERY != 0), and recent enough to still be in the
  # 132-frame (6.6 s) queue at the end of the loop -- an earlier pulse legitimately ages out
  if k == 189:
    desire[3] = 1.0
  st.tick_frame(frame, desire)
  if k % RUN_EVERY == 0:
    q = st.feed()
    cur = st.newest_frame()
    # 1. the motion cue must be exactly 2 frames, 1 camera-period apart
    prev_id, cur_id = int(q["state_img_q"][0, -1, 0, 0, 0]), int(cur[0, 0, 0, 0])
    if k and (cur_id - prev_id) % 251 != 1:
      fail(f"motion cue spans {(cur_id - prev_id) % 251} frames at k={k}, want 1 (50 ms)")
    if k and prev_id == cur_id:
      fail(f"motion cue is two IDENTICAL frames at k={k}")
    f = np.full(16384, k, np.float32)
    st.absorb(f); feats.append(k)
print(f"  OK   motion cue is always 2 consecutive 20 Hz frames ({RUN_EVERY}x downrate)")

# 2. the stride-4 read must land on distinct, consecutive 5 Hz features
read = st.feed()["state_feat_q"][0::FEAT_STRIDE, 0, 0]
want = np.array(feats[-len(read):], dtype=np.float32)
if not np.array_equal(read, want):
  fail(f"stride-{FEAT_STRIDE} read gives {read[-6:]}, want the last {len(read)} run features {want[-6:]}")
print(f"  OK   state_feat_q[0::{FEAT_STRIDE}] = the last {len(read)} features, one per model run (5 Hz)")

# 3. a desire pulse between runs must survive into its own 200 ms window
d = st.feed()["state_desire_q"].reshape(-1, DESIRE_WINDOW, 8).max(axis=1)   # the model's own max-pool
if d[:, 3].sum() == 0:
  fail("desire pulse fired between model runs was LOST")
hits = np.flatnonzero(d[:, 3])
if len(hits) != 1:
  fail(f"pulse smeared across {len(hits)} windows, want exactly 1")
print(f"  OK   a desire pulse between runs survives, in exactly 1 of the {d.shape[0]} windows")

# 4. windows must be 4 frames, not 4 model runs
st2 = LowFreqState(SHAPES, RUN_EVERY)
for k in range(8):
  dd = np.zeros(8, np.float32); dd[1] = 1.0 if k in (0, 5) else 0.0
  st2.tick_frame(np.zeros((2, 6, 8, 8), np.uint8), dd)
w = st2.feed()["state_desire_q"].reshape(-1, DESIRE_WINDOW, 8).max(axis=1)[:, 1]
if np.flatnonzero(w).size != 2:
  fail(f"two pulses 5 frames apart landed in {np.flatnonzero(w).size} windows, want 2 (200 ms each)")
print("  OK   desire windows span 4 CAMERA frames (200 ms), not 4 model runs (800 ms)")

# 5. the model RE-APPENDS `desire` to whatever queue we pass:
#      next_state_desire_q = concat(state_desire_q[1:], desire)
# so feed() must be rolled such that this reconstructs our queue EXACTLY. Otherwise the current pulse
# lands at both slot -2 and -1 and can smear across two 200 ms windows.
st3 = LowFreqState(SHAPES, RUN_EVERY)
for k in range(40):
  dd = np.zeros(8, np.float32)
  if k in (7, 22, 39): dd[2] = 1.0
  st3.tick_frame(np.zeros((2, 6, 8, 8), np.uint8), dd)
fed = st3.feed()["state_desire_q"]
rebuilt = np.concatenate([fed[1:], st3.current_desire().reshape(1, 1, -1)], axis=0)
if not np.array_equal(rebuilt, st3.desire):
  fail("model's own append does NOT reconstruct our desire queue -- feed() roll is wrong")
n_pulse = int((st3.desire[:, 0, 2] > 0.5).sum())
if n_pulse != 3:
  fail(f"{n_pulse} pulses in the queue, want 3 (one per rising edge, none duplicated)")
print("  OK   concat(feed[1:], desire) reconstructs the queue exactly -- no double-counted pulse")
print("\nALL LOW-FREQ QUEUE CHECKS PASS")
