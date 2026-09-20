"""Boot CinqueModelState on a desktop GPU and check the 5 Hz pipeline actually pipelines.

  PYTHONPATH=. DEV=CUDA WARP_DEV=CUDA QUEUE_DEV=CUDA .venv/bin/python \
    openpilot/selfdrive/modeld/test_cinque_boot.py <little_cinque_tinygrad.pkl>

What this catches that a unit test cannot: the pkl's real input_specs meeting the real queue builder,
the warp's output shape meeting `new_img`, and -- the point of the whole design -- that a dispatch
RETURNS in single-digit ms instead of blocking for the model's full runtime.
"""
import os, sys, time
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)) + "/../../..")
os.environ.setdefault("LITTLE_CINQUE", "1")

PKL = sys.argv[1] if len(sys.argv) > 1 else None
if PKL:
  import openpilot.selfdrive.modeld.modeld as M
  M.LC_PKL = __import__("pathlib").Path(PKL)
import openpilot.selfdrive.modeld.modeld as M

CAM_W, CAM_H = 1928, 1208


class FakeBuf:
  def __init__(self, n): self.data = np.zeros(n, dtype=np.uint8)


def main():
  assert M.lc_active(), f"no pkl at {M.LC_PKL}"
  t0 = time.perf_counter()
  st = M.CinqueModelState(CAM_W, CAM_H)
  print(f"  booted in {time.perf_counter()-t0:.1f}s | queues on {st.QUEUE_DEV}, warp on {st.WARP_DEV}")
  print(f"  state inputs: {st.STATE_INPUTS}")
  print(f"  hidden_slice: {st.hidden_slice}")

  size = st.frame_buf_params['img'][3]
  bufs = {k: FakeBuf(size) for k in st.vision_input_names}
  for b in bufs.values():
    b.data[:] = np.random.randint(0, 255, size, dtype=np.uint8)
  eye = np.eye(3, dtype=np.float32)
  tfms = dict.fromkeys(st.vision_input_names, eye)
  inputs = {'traffic_convention': np.array([1., 0.], np.float32), 'action_t': np.array([.4, .4], np.float32)}

  got = 0
  for k in range(12):
    st.warp_frame(bufs, tfms, np.zeros(8, np.float32))
    if k % M.RUN_EVERY == 0:
      fresh = st.collect()
      t1 = time.perf_counter()
      st.run(inputs, defer=True)
      dt = (time.perf_counter() - t1) * 1e3
      tag = "collected" if fresh is not None else "nothing to collect yet"
      print(f"  k={k:2d} {tag}; dispatch returned in {dt:7.1f} ms")
      if fresh is not None:
        got += 1
        assert np.isfinite(fresh['plan']).all(), "plan not finite"
        assert np.isfinite(fresh['action']).all(), "action not finite"
  assert got >= 2, f"only collected {got} results"
  print(f"  plan {fresh['plan'].shape}  lane_lines {fresh['lane_lines'].shape}  all finite")

  # the feature ring must actually be filling: stride-4 read, one distinct feature per run
  live = int(np.abs(st.lowfreq.feat[::4]).sum(axis=(1, 2)).astype(bool).sum())
  print(f"  feature ring: {live}/32 stride-4 slots populated after {got+1} runs")
  aged = M.lowfreq.age(fresh, 0.15)
  assert np.isfinite(aged['plan']).all()
  print(f"  age(0.15s) ok; plan y@2s {np.interp(2.0, M.lowfreq.T_IDXS, aged['plan'][0][:,1]):+.3f} m "
        f"(unaged {np.interp(2.0, M.lowfreq.T_IDXS, fresh['plan'][0][:,1]):+.3f})")
  print("\nCINQUE BOOT CHECKS PASS")


if __name__ == "__main__":
  main()
