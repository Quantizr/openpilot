"""Keep cinque's state queues on their NATIVE 20 Hz cadence while the model only runs at 5 Hz.

Every one of cinque's queues encodes a sampling rate, and all three break if they simply advance once per
model run. They are read as (verified against the graph's own Slice/Gather args):

  state_img_q    [2,5,6,128,256]  Gather(cam) then the last 2 frames  -> the MOTION CUE, 50 ms apart
  state_desire_q [132,1,8]        reshape [33,4,8] then ReduceMax(1)  -> 33 windows of FOUR 20 Hz frames
  state_feat_q   [128,1,16384]    Slice(step=4)                       -> a 5 Hz sample of a 20 Hz ring

At 5 Hz, advancing once per run gives: frames 200 ms apart (4x the trained optical flow), desire windows
spanning 800 ms each (and any pulse between runs dropped entirely), and a feature history sampled at
1.25 Hz. None of those would raise an error -- the model would just quietly receive inputs it was never
distilled on. So modeld keeps the queues itself, ticking them every camera frame, and only the expensive
tower+policy run at 5 Hz.

  tick_frame(warped, desire)   every 20 Hz camera frame  (cheap: a warp and an 8-float push)
  feed(inputs)                 on a RUN frame: hand the model the correctly-spaced queues
  absorb(hidden_state)         after a run: push the new feature FOUR times, see below

★ WHY THE FEATURE GOES IN FOUR TIMES. The policy reads `state_feat_q[0:128:4]`, so consecutive READ slots
are 4 apart. Writing each 5 Hz feature into 4 consecutive slots makes that stride land on exactly one
slot per distinct feature -- reproducing the intended 32-entry, 5 Hz history with no graph change. It is
the same trick the ring was already doing, just driven at the rate we actually produce features.

⚠ The model's own `next_state_*` outputs are DISCARDED in this mode. That is deliberate: they implement
the 20 Hz-run semantics, which is precisely what is wrong here. Do not "fix" this by feeding them back.
"""
from __future__ import annotations

import numpy as np

DESIRE_WINDOW = 4          # state_desire_q is [33,4,8] before the max-pool
FEAT_STRIDE = 4            # state_feat_q is read with step=4


class LowFreqState:
  """Owns cinque's queues so they keep 20 Hz semantics while the model runs at 1/`run_every`."""

  def __init__(self, shapes: dict[str, tuple], run_every: int = 4):
    self.run_every = run_every
    self.img = np.zeros(shapes["state_img_q"][0], dtype=np.dtype(shapes["state_img_q"][1]))
    self.desire = np.zeros(shapes["state_desire_q"][0], dtype=np.dtype(shapes["state_desire_q"][1]))
    self.feat = np.zeros(shapes["state_feat_q"][0], dtype=np.dtype(shapes["state_feat_q"][1]))
    self.prev_desire = np.zeros(self.desire.shape[-1], dtype=np.float32)
    self.newest: np.ndarray | None = None   # current frame  -> becomes `new_img`
    self.prev: np.ndarray | None = None     # frame 50 ms ago -> lives in state_img_q[:, -1]
    self.have_prev_frame = False

  # ---- 20 Hz -------------------------------------------------------------------------------------
  def tick_frame(self, warped: np.ndarray, desire_pulse: np.ndarray) -> None:
    """One camera frame. `warped` is (2,6,H,W), the same tensor the model calls `new_img`.

    ★ The model forms its 2-frame motion cue as (state_img_q[:, -1], new_img) -- it appends new_img to
    the ring and takes the last two. So the QUEUE must hold the PREVIOUS frame and `new_img` the current
    one. Writing the current frame into slot -1 as well would hand the tower two identical frames and a
    motion cue of exactly zero, which trains-time never saw and which nothing downstream would flag.
    Only slot -1 is ever read, so the rest of the ring is left alone."""
    self.prev = self.newest
    self.newest = warped.copy()
    if self.prev is not None:
      self.img[:, -1] = self.prev
      self.have_prev_frame = True
    # desire is a PULSE on the rising edge. Detecting it here, every frame, is what stops a pulse that
    # begins and ends between two model runs from being missed -- at 5 Hz that is a 200 ms blind window.
    pulse = np.where(desire_pulse - self.prev_desire > 0.99, desire_pulse, 0)
    self.prev_desire[:] = desire_pulse
    self.desire[:-1] = self.desire[1:]
    self.desire[-1, 0] = pulse

  # ---- 5 Hz --------------------------------------------------------------------------------------
  def feed(self) -> dict[str, np.ndarray]:
    """The queues as the model should see them on a run frame.

    `new_img` is the newest frame and the queue's last slot is the one BEFORE it, so the model's
    "last 2 frames" are 50 ms apart exactly as in the 20 Hz case.

    ★ THE DESIRE QUEUE IS SHIFTED BY ONE ON PURPOSE. The graph builds
        next_state_desire_q = concat(state_desire_q[1:], desire)
    and max-pools THAT. So whatever we pass is re-appended with `desire`. Handing it our queue as-is
    would place the current pulse at BOTH slot -2 and slot -1 -- double-counting it, and (when the
    window boundary falls between them) smearing one pulse across two 200 ms windows. Rolling by one
    means the model's own append reconstructs our queue exactly: roll(q,1)[1:] + q[-1] == q."""
    return {"state_img_q": self.img.copy(), "state_desire_q": np.roll(self.desire, 1, axis=0),
            "state_feat_q": self.feat.copy()}

  def current_desire(self) -> np.ndarray:
    """The pulse for THIS frame -- the model's `desire` input, i.e. the newest queue entry.

    ★ SINGLE SOURCE OF TRUTH. ModelState.run used to do its own rising-edge detection against a
    `prev_desire` that only advanced on RUN frames; with the model at 5 Hz that detector both misses
    pulses fired in between AND can re-fire one this class already consumed. The edge is detected once,
    here, every camera frame."""
    return self.desire[-1, 0]

  def newest_frame(self) -> np.ndarray:
    """What to pass as `new_img`: the CURRENT frame (the queue holds the previous one)."""
    return self.newest

  def absorb(self, hidden_state: np.ndarray) -> None:
    """Push the feature this run produced, FOUR times, so the stride-4 read sees 5 Hz spacing."""
    n = FEAT_STRIDE
    self.feat[:-n] = self.feat[n:]
    self.feat[-n:] = hidden_state.reshape(1, -1).astype(self.feat.dtype)
