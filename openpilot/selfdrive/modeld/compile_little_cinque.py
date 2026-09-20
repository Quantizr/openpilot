"""Compile the composed little-cinque model (v21c vision + cinque policy) for this branch's tinygrad.

  DEV=QCOM IMAGE=1 FLOAT16=1 NOLOCALS=1 JIT_BATCH_SIZE=0 OPENPILOT_HACKS=1 \
    python3 openpilot/selfdrive/modeld/compile_little_cinque.py \
      --onnx models/little_cinque_fp32_slim.onnx --model-size 512x256 \
      --camera-resolutions 1928x1208 1344x760 --output models/little_cinque_tinygrad.pkl

★ WHY THIS FILE AND NOT compile_modeld.py. compile_modeld builds the OLD model's queues inside the JIT
(`img_q`/`big_img_q`/`feat_q`/`desire_q` + `shift_and_sample`). cinque v3 carries its queues as MODEL I/O
(`state_*_q` in, `next_state_*` out) and modeld keeps them on their native 20 Hz cadence in LowFreqState,
so none of that machinery applies. Everything that does apply -- the warp, `compile_jit`'s
random-replay check, the OOB pickle -- is imported rather than copied.

★ WHY THIS BRANCH. tinygrad `587cac01e` runs this graph at **141 ms** on the Adreno; openpilot master's
pinned `9d0446a4b` runs the identical graph at **209 ms**, same harness, same flags. The 1.6x is the
tinygrad version alone (our two fork patches, reverted, changed nothing). The newer tree's
`compile_onnx.py` never existed alongside the fast codegen -- `1c53bd6b9`, the commit that added it,
already measures 208.58 ms -- so the fast number is only reachable from here.

★ ONLY `outputs` IS RETURNED, and that is a speedup, not a simplification. The three `next_state_*`
outputs re-implement 20 Hz-run semantics that are wrong at 5 Hz, so modeld discards them; returning them
anyway would make the JIT materialise a 128x16384 fp32 ring write (8.4 MB) every frame for nothing.
`hidden_state` is a slice of `outputs`, so the feature modeld feeds back is still right there.
"""
import argparse
import math
import os
from functools import partial

import numpy as np

from tinygrad.tensor import Tensor
from tinygrad.device import Device
from tinygrad.engine.jit import TinyJit
from tinygrad.nn.onnx import OnnxRunner

def _install_helpers_stub():
  """Let this run from a bare checkout of just the compile path.

  `modeld.helpers` imports `common.hardware`, which imports `cereal`, which needs a built capnp -- a
  whole toolchain we do not need to emit a pkl. `compile_modeld` only wants `dump_oob`/`load_oob` from
  it, and those are stdlib-only, so when the real module will not import we register an equivalent under
  its name rather than duplicating `compile_modeld`'s warp math here. If the real one imports, it wins.
  """
  import importlib, io, pickle, shutil, struct, sys, tempfile
  try:
    importlib.import_module("openpilot.selfdrive.modeld.helpers")
    return False
  except Exception:
    pass

  def dump_oob(obj, f):
    with tempfile.TemporaryFile(dir=".") as tmp:
      def buffer_callback(pb):
        m = pb.raw()
        tmp.write(struct.pack("<q", m.nbytes)); tmp.write(m)
        pb.release()                      # keep peak RAM at ~1 buffer
      stream = io.BytesIO()
      pickle.Pickler(stream, protocol=5, buffer_callback=buffer_callback).dump(obj)
      f.write(struct.pack("<q", len(op := stream.getvalue()))); f.write(op)
      tmp.seek(0); shutil.copyfileobj(tmp, f)

  def load_oob(f):
    opcodes = f.read(struct.unpack("<q", f.read(8))[0])
    def buffers():
      while (h := f.read(8)):
        pb = pickle.PickleBuffer(bytearray(struct.unpack("<q", h)[0]))
        f.readinto(pb)
        yield pb
    return pickle.load(io.BytesIO(opcodes), buffers=buffers())

  import types
  mod = types.ModuleType("openpilot.selfdrive.modeld.helpers")
  mod.dump_oob, mod.load_oob = dump_oob, load_oob
  sys.modules["openpilot.selfdrive.modeld.helpers"] = mod
  print("note: using the stdlib-only helpers stub (cereal unavailable)", flush=True)
  return True


_install_helpers_stub()

from openpilot.selfdrive.modeld.compile_modeld import (NV12Frame, WARP_DEV, WARP_INPUTS, compile_jit,
                                                       make_random_images, make_warp, _parse_size)
from openpilot.selfdrive.modeld.get_model_metadata import make_metadata_dict
from openpilot.selfdrive.modeld.helpers import dump_oob
from openpilot.system.camerad.cameras.nv12_info import get_nv12_info

# what run_model takes besides `warped`, in order. The three state queues live on the model device; the
# small scalars ride in one packed NPY buffer so a frame costs one host->device copy, as upstream does.
STATE_INPUTS = ['state_img_q', 'state_desire_q', 'state_feat_q']
SCALAR_INPUTS = ['desire', 'traffic_convention', 'action_t']
MODEL_INPUTS = STATE_INPUTS + ['packed_npy_inputs']


def make_cinque_warp_queues(input_shapes, device):
  """Just the two transforms. This branch's `make_warp` reads only tfm/big_tfm (WARP_INPUTS)."""
  npy = {'tfm': np.zeros((3, 3), dtype=np.float32), 'big_tfm': np.zeros((3, 3), dtype=np.float32)}
  return {k: Tensor(v, device='NPY').realize() for k, v in npy.items()}, npy


def make_cinque_queues(input_shapes, device):
  input_queues, npy = make_cinque_warp_queues(input_shapes, device)
  for name in STATE_INPUTS:
    shape, dtype = input_shapes[name]
    input_queues[name] = Tensor(np.zeros(shape, dtype=dtype), device=device).contiguous().realize()
  shapes = {k: tuple(input_shapes[k][0]) for k in SCALAR_INPUTS}
  sizes = [math.prod(s) for s in shapes.values()]
  packed = np.zeros(sum(sizes), dtype=np.float32)
  npy.update({k: v.reshape(s) for (k, s), v in
              zip(shapes.items(), np.split(packed, np.cumsum(sizes[:-1])), strict=True)})
  input_queues['packed_npy_inputs'] = Tensor(packed, device='NPY').realize()
  return input_queues, npy


def make_run_model(model_runner, scalar_shapes):
  offs, cuts = 0, {}
  for name, shape in scalar_shapes.items():
    cuts[name] = (offs, offs + math.prod(shape), shape)
    offs += math.prod(shape)

  def run_model(warped, state_img_q, state_desire_q, state_feat_q, packed_npy_inputs):
    warped = warped.to(Device.DEFAULT)
    packed = packed_npy_inputs.to(Device.DEFAULT)
    Tensor.realize(warped, packed)
    scalars = {n: packed[a:b].reshape(s) for n, (a, b, s) in cuts.items()}
    out = model_runner({'new_img': warped, 'state_img_q': state_img_q,
                        'state_desire_q': state_desire_q, 'state_feat_q': state_feat_q, **scalars})
    return [out['outputs'].realize()]

  return run_model


def main():
  p = argparse.ArgumentParser(description=__doc__)
  p.add_argument('--onnx', required=True)
  p.add_argument('--model-size', type=_parse_size, required=True)
  p.add_argument('--camera-resolutions', type=_parse_size, nargs='+', required=True)
  p.add_argument('--output', required=True)
  args = p.parse_args()

  model_path = args.onnx
  model_w, model_h = args.model_size
  model_runner = OnnxRunner(model_path)

  out = {'metadata': make_metadata_dict(model_path)}
  ishapes = out['metadata']['input_shapes']
  # make_metadata_dict gives shapes only; pair each with the dtype the runner declares, because
  # state_img_q is uint8 and allocating it float32 would be silently 4x the buffer and the wrong bytes.
  spec = {name: (tuple(s.shape), np.dtype(s.dtype.fmt)) for name, s in model_runner.graph_inputs.items()}
  out['metadata']['input_specs'] = spec
  scalar_shapes = {k: tuple(spec[k][0]) for k in SCALAR_INPUTS}

  make_queues = partial(make_cinque_queues, spec)
  warped_shape = (2, 6, model_h // 2, model_w // 2)
  make_random_model_inputs = partial(make_random_images, keys=['warped'], shape=warped_shape, device=WARP_DEV)
  run_model_jit = TinyJit(make_run_model(model_runner, scalar_shapes), prune=True)
  out['run_policy'] = compile_jit(run_model_jit, make_random_model_inputs, MODEL_INPUTS, make_queues)

  for cam_w, cam_h in args.camera_resolutions:
    nv12 = NV12Frame(cam_w, cam_h, *get_nv12_info(cam_w, cam_h))
    make_random_warp_inputs = partial(make_random_images, keys=['frame', 'big_frame'], shape=nv12.size, device=WARP_DEV)
    warp = TinyJit(make_warp(nv12, model_w, model_h, 1), prune=True)
    out[(cam_w, cam_h)] = compile_jit(warp, make_random_warp_inputs, WARP_INPUTS,
                                      partial(make_cinque_warp_queues, spec))

  with open(args.output, "wb") as f:
    dump_oob(out, f)
  print(f"Saved JITs to {args.output} ({os.path.getsize(args.output) / 1e6:.2f} MB)")


if __name__ == "__main__":
  main()
