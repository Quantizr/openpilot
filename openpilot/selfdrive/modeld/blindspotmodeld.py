#!/usr/bin/env python3
"""Blind Spot Assist: detects cars in the blind spots from the cabin camera. Not blind spot monitoring: it can miss cars.

A MobileNetV2 + GRU runs at 5 Hz on the last three warped cabin frames; publishes per-side probabilities and detections.
"""
import time
import pickle
import numpy as np
from tinygrad.tensor import Tensor

from openpilot.cereal import messaging
from openpilot.cereal.messaging import PubMaster
from openpilot.cereal.visionipc import VisionStreamType
from msgq.visionipc import VisionIpcClient, VisionBuf
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.modeld.helpers import MODELS_DIR, load_oob
from openpilot.system.camerad.cameras.nv12_info import get_nv12_info

MODEL_PKL_PATH = MODELS_DIR / 'bsa_model_tinygrad.pkl'
MODEL_W, MODEL_H = 448, 128
N_FRAMES = 3
FRAME_STEP = 4  # cabin camera runs at 20 fps, the model at 5 Hz
DT_STEP = FRAME_STEP * 0.05
# leaky bucket per side: detected while level >= THRESHOLD, and for MIN_ON s after
BUCKET_FLOOR, BUCKET_GAIN, BUCKET_DECAY, BUCKET_THRESHOLD, MIN_ON = 0.2, 2.0, 0.5, 0.4, 1.0

# training geometry: cabin image scaled to a centred 964x604 canvas (f = 283.5), model band = canvas cols 34-930, rows 144-400
CANVAS_W, CANVAS_H = 964, 604
BAND_X0, BAND_Y0, BAND_SCALE = 34, 144, 2  # canvas pixels per model pixel


def canvas_geometry(cam_w: int, cam_h: int) -> tuple[int, int, int, int]:
  """size and offset of the camera image inside the training canvas"""
  s = CANVAS_W / 1928 if (cam_w, cam_h) == (1928, 1208) else (567.0 / 425.25) * CANVAS_W / 1928
  tw, th = min(2 * round(cam_w * s / 2), CANVAS_W), min(2 * round(cam_h * s / 2), CANVAS_H)
  return tw, th, 2 * ((CANVAS_W - tw) // 4), 2 * ((CANVAS_H - th) // 4)


def model_to_camera(cam_w: int, cam_h: int) -> np.ndarray:
  """3x3 transform from model-band pixel (x, y) to camera pixel (the warp's M_inv), pixel-centre aligned"""
  tw, th, x0, y0 = canvas_geometry(cam_w, cam_h)
  sx, sy = cam_w / tw, cam_h / th
  ax, bx = BAND_SCALE * sx, (BAND_X0 + 0.5 * BAND_SCALE - x0) * sx - 0.5
  ay, by = BAND_SCALE * sy, (BAND_Y0 + 0.5 * BAND_SCALE - y0) * sy - 0.5
  return np.array([[ax, 0, bx], [0, ay, by], [0, 0, 1]], dtype=np.float32)


class Detector:
  """per-side on/off signal from the leaky bucket (BUCKET_*, MIN_ON), updated once per model step"""
  def __init__(self):
    self.level = 0.0
    self.hold = 0.0  # s left to hold the detection

  def update(self, p: float, dt: float = DT_STEP) -> bool:
    self.level = min(1.0, max(0.0, self.level - BUCKET_DECAY * dt) + BUCKET_GAIN * max(0.0, p - BUCKET_FLOOR))
    if self.level >= BUCKET_THRESHOLD:
      self.hold = MIN_ON
      return True
    self.hold -= dt
    return self.hold > 1e-6


class ModelState:
  """precompiled warp + model; frame history and GRU state are kept on the host (GRU state is f32 out, f16 in on QCOM)"""
  def __init__(self, cam_w: int, cam_h: int):
    jits = load_oob(MODEL_PKL_PATH)
    self.DEV = jits['input_specs']['frames'][2]
    self.model_run = jits['run']
    self.outputs = {name: Tensor(np.zeros(shape, dtype=dtype), device=device).realize()
                    for name, (shape, dtype, device) in jits['output_specs'].items()}
    h_shape, h_dtype, _ = jits['input_specs']['h_gru']
    self.h_np = np.zeros(h_shape, dtype=h_dtype)
    self.h_host = Tensor(self.h_np, device='NPY')._buffer()
    self.h_gru = Tensor(self.h_np, device=self.DEV).realize()
    with open(MODELS_DIR / f'bsa_warp_{cam_w}x{cam_h}_tinygrad.pkl', "rb") as f:
      self.image_warp = pickle.load(f)['run']
    self.frame_size = get_nv12_info(cam_w, cam_h)[3]
    self.transform = Tensor(model_to_camera(cam_w, cam_h), device='NPY')
    self._blob_cache: dict[int, Tensor] = {}
    # last N_FRAMES warped frames, 6 yuv420 planes each, oldest first
    self.frames_np = np.zeros((1, 6 * N_FRAMES, MODEL_H // 2, MODEL_W // 2), dtype=np.uint8)
    self.frames_host = Tensor(self.frames_np, device='NPY')._buffer()
    self.frames = Tensor(self.frames_np, device=self.DEV).realize()
    self.first = True

  def reset(self):
    self.first = True
    self.h_np[:] = 0

  def run(self, buf: VisionBuf) -> np.ndarray:
    ptr = np.frombuffer(buf.data, dtype=np.uint8).ctypes.data
    # camerad uses a ring of buffers: cache one tensor per buffer
    if ptr not in self._blob_cache:
      self._blob_cache[ptr] = Tensor.from_blob(ptr, (self.frame_size,), dtype='uint8', device=self.DEV)
    warped = self.image_warp(input_frame=self._blob_cache[ptr], M_inv=self.transform).numpy().reshape(6, MODEL_H // 2, MODEL_W // 2)
    if self.first:  # at the start, repeat the first frame (as in training)
      self.frames_np[0] = np.tile(warped, (N_FRAMES, 1, 1))
      self.first = False
    else:
      self.frames_np[0, :-6] = self.frames_np[0, 6:]
      self.frames_np[0, -6:] = warped
    self.frames._buffer().copy_from(self.frames_host)
    self.h_gru._buffer().copy_from(self.h_host)
    self.model_run(output_buffers=self.outputs, frames=self.frames, h_gru=self.h_gru)
    self.h_np[:] = self.outputs['h_gru_out'].numpy()
    # [vehicle left, vehicle right, adjacent left, adjacent right, approaching left, approaching right]
    return self.outputs['probs'].numpy().astype(np.float32).reshape(-1)


def main():
  cloudlog.warning("blindspotmodeld: connecting to cabin stream")
  vipc_client = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_CABIN, True)
  while not vipc_client.connect(False):
    time.sleep(0.1)
  cloudlog.warning(f"blindspotmodeld: connected, {vipc_client.width}x{vipc_client.height}")

  model = ModelState(vipc_client.width, vipc_client.height)
  pm = PubMaster(["blindSpotAssist"])
  last_frame_id = None
  detectors = [Detector(), Detector()]

  while True:
    buf = vipc_client.recv()
    if buf is None:
      continue
    frame_id = vipc_client.frame_id
    if frame_id % FRAME_STEP != 0:
      continue
    # stream gap (camera restart, dropped frames): frame stack and GRU state are stale
    if last_frame_id is not None and frame_id - last_frame_id != FRAME_STEP:
      model.reset()
      detectors = [Detector(), Detector()]
    last_frame_id = frame_id

    t1 = time.perf_counter()
    probs = model.run(buf)
    msg = messaging.new_message('blindSpotAssist', valid=True)
    bs = msg.blindSpotAssist
    bs.frameId = frame_id
    bs.leftProb, bs.rightProb = float(probs[0]), float(probs[1])
    bs.leftAdjacentProb, bs.rightAdjacentProb, bs.leftApproachingProb, bs.rightApproachingProb = (float(p) for p in probs[2:6])
    bs.leftDetected, bs.rightDetected = detectors[0].update(bs.leftProb), detectors[1].update(bs.rightProb)
    bs.leftLevel, bs.rightLevel = detectors[0].level, detectors[1].level
    bs.modelExecutionTime = time.perf_counter() - t1
    pm.send("blindSpotAssist", msg)


if __name__ == "__main__":
  try:
    main()
  except KeyboardInterrupt:
    cloudlog.warning("got SIGINT")
