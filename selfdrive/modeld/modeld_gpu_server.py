#!/usr/bin/env python3
import os
import socket
import struct
import time
import pickle
import numpy as np
from pathlib import Path
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.modeld.parse_model_outputs import Parser
tg_dev = os.getenv("DEV", "CUDA") # importing modeld changes this
from openpilot.selfdrive.modeld.modeld import InputQueues
os.environ["DEV"] = tg_dev
from tinygrad.tensor import Tensor
from tinygrad.dtype import dtypes

# --- Server Configuration ---
HOST = "0.0.0.0"      # Listen on all interfaces
PORT = 5501
BUF_SIZE = 1 << 20    # 1 MiB per recv() call

# --- Model Paths ---
# Use larger, more powerful models on the server
VISION_PKL_PATH = Path(__file__).parent / 'models/driving_vision_tinygrad.pkl'
POLICY_PKL_PATH = Path(__file__).parent / 'models/driving_policy_tinygrad.pkl'
VISION_METADATA_PATH = Path(__file__).parent / 'models/driving_vision_metadata.pkl'
POLICY_METADATA_PATH = Path(__file__).parent / 'models/driving_policy_metadata.pkl'

# --- Global Model Data (loaded once) ---
with open(VISION_METADATA_PATH, 'rb') as f:
  vision_metadata = pickle.load(f)
with open(POLICY_METADATA_PATH, 'rb') as f:
  policy_metadata = pickle.load(f)
with open(VISION_PKL_PATH, "rb") as f:
  vision_run = pickle.load(f)
with open(POLICY_PKL_PATH, "rb") as f:
  policy_run = pickle.load(f)


class ServerModelState:
  def __init__(self):
    self.parser = Parser()
    policy_input_shapes = policy_metadata['input_shapes']

    self.numpy_inputs = {k: np.zeros(policy_input_shapes[k], dtype=np.float32) for k in policy_input_shapes}
    self.full_input_queues = InputQueues(ModelConstants.MODEL_CONTEXT_FREQ, ModelConstants.MODEL_RUN_FREQ, ModelConstants.N_FRAMES)
    for k in ['desire_pulse', 'features_buffer']:
      self.full_input_queues.update_dtypes_and_shapes({k: self.numpy_inputs[k].dtype}, {k: self.numpy_inputs[k].shape})

    # The number of frames needed to have a full history for the policy model.
    self.frames_to_prime = self.full_input_queues.shapes['features_buffer'][1]
    self.reset()

  def reset(self):
    print("Server state is being reset.")
    self.full_input_queues.reset()
    self.frames_enqueued_count = 0
    self.is_ready = False

  def run_model(self, inputs: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
    # 1. Run Vision Model
    vision_inputs_tensor = {k: Tensor(v, dtype=dtypes.uint8).realize() for k, v in inputs.items() if 'img' in k}
    vision_output = vision_run(**vision_inputs_tensor).contiguous().realize().uop.base.buffer.numpy()
    vision_outputs_dict = self.parser.parse_vision_outputs({k: vision_output[np.newaxis, v] for k, v in vision_metadata['output_slices'].items()})

    # 2. Update State with InputQueues
    self.full_input_queues.enqueue({
      'features_buffer': vision_outputs_dict['hidden_state'],
      'desire_pulse': inputs['desire_pulse']
    })
    self.frames_enqueued_count += 1
    if not self.is_ready and self.frames_enqueued_count >= self.frames_to_prime:
      print(f"Server buffer primed after {self.frames_enqueued_count} frames. Server is ready.")
      self.is_ready = True

    for k in ['desire_pulse', 'features_buffer']:
      self.numpy_inputs[k][:] = self.full_input_queues.get(k)[k]
    self.numpy_inputs['traffic_convention'][:] = inputs['traffic_convention']

    # 3. Run Policy Model only if the history buffer is full
    if self.is_ready:
      policy_inputs_tensor = {k: Tensor(v, device='NPY').realize() for k, v in self.numpy_inputs.items()}
      policy_output = policy_run(**policy_inputs_tensor).contiguous().realize().uop.base.buffer.numpy()
    else:
      policy_output = np.zeros(policy_metadata['output_shapes']['outputs'], dtype=np.float32)

    return {'vision_output': vision_output, 'policy_output': policy_output}


def parse_request_packet(data: bytes) -> dict[str, np.ndarray]:
  offset = 0
  seq, flags, num_arrays = struct.unpack_from("!IBB", data, offset)
  offset += 6
  arrays = {'seq': seq, 'reset_flag': bool(flags & 1)}

  descriptors = []
  descriptor_offset = offset
  for _ in range(num_arrays):
    name_len = data[descriptor_offset]
    descriptor_offset += 1
    name = data[descriptor_offset:descriptor_offset+name_len].decode('utf-8')
    descriptor_offset += name_len
    dtype_len = data[descriptor_offset]
    descriptor_offset += 1
    dtype = data[descriptor_offset:descriptor_offset+dtype_len].decode('utf-8')
    descriptor_offset += dtype_len
    num_dims = data[descriptor_offset]
    descriptor_offset += 1
    shape = struct.unpack_from("!" + "I" * num_dims, data, descriptor_offset)
    descriptor_offset += 4 * num_dims
    descriptors.append({'name': name, 'dtype': np.dtype(dtype), 'shape': shape})

  data_offset = descriptor_offset
  for desc in descriptors:
    num_bytes = int(np.prod(desc['shape'])) * desc['dtype'].itemsize
    arr = np.frombuffer(data, dtype=desc['dtype'], count=int(np.prod(desc['shape'])), offset=data_offset).reshape(desc['shape'])
    arrays[desc['name']] = arr
    data_offset += num_bytes
  return arrays

def build_response_packet(seq: int, is_ready: bool, arrays: dict[str, np.ndarray]) -> bytes:
  flags = 1 if is_ready else 0
  header = struct.pack("!IBB", seq, flags, len(arrays))
  descriptors = bytearray()
  data_parts = []

  for name, arr in arrays.items():
    name_bytes, dtype_bytes = name.encode('utf-8'), str(arr.dtype).encode('utf-8')
    shape_bytes = struct.pack("!" + "I" * len(arr.shape), *arr.shape)
    descriptors += struct.pack("!B", len(name_bytes)) + name_bytes
    descriptors += struct.pack("!B", len(dtype_bytes)) + dtype_bytes
    descriptors += struct.pack("!B", len(arr.shape)) + shape_bytes
    data_parts.append(arr.tobytes())
  return header + descriptors + b''.join(data_parts)


def handle_connection(conn, addr):
  print(f"Connection from {addr}")
  conn.settimeout(5)
  conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

  server_state = ServerModelState()
  last_stat_time, request_count, total_compute_us, total_bytes_in, total_bytes_out = time.perf_counter(), 0, 0, 0, 0
  first_request = True

  while True:
    try:
      # 1. Read header (4 bytes for total length) and then the full packet
      header = conn.recv(4, socket.MSG_WAITALL)
      if len(header) < 4:
        if len(header) == 0:
          print("Client disconnected cleanly.")
        else:
          print("Short header received.")
        break
      total_len = struct.unpack("!I", header)[0]

      data = bytearray(total_len)
      view = memoryview(data)
      to_recv = total_len
      while to_recv > 0:
        n = conn.recv_into(view[total_len - to_recv:], min(BUF_SIZE, to_recv))
        if n == 0:
          raise ConnectionError("peer closed early")
        to_recv -= n

      # 2. On first request, log payload sizes
      if first_request:
        dummy_outputs = {
          'vision_output': np.zeros(vision_metadata['output_shapes']['outputs'], dtype=np.float32),
          'policy_output': np.zeros(policy_metadata['output_shapes']['outputs'], dtype=np.float32),
          'compute_us': np.array([0], dtype=np.float64)
        }
        estimated_output_size = len(build_response_packet(0, True, dummy_outputs)) + 4
        print(f"First request received. Input size: {total_len + 4} bytes. Estimated output size: {estimated_output_size} bytes.")
        first_request = False

      # 3. Parse packet and check for reset
      inputs = parse_request_packet(data)
      seq = inputs.get('seq', -1)
      if inputs.get('reset_flag', False):
        server_state.reset()

      # 4. Run the model
      ts_compute_start = time.perf_counter_ns()
      outputs = server_state.run_model(inputs)
      ts_compute_end = time.perf_counter_ns()
      compute_us = (ts_compute_end - ts_compute_start) / 1000

      # 5. Build and send the response
      outputs['compute_us'] = np.array([compute_us], dtype=np.float64)
      response_bytes = build_response_packet(seq, server_state.is_ready, outputs)
      conn.sendall(struct.pack("!I", len(response_bytes)) + response_bytes)

      # 6. Update stats
      request_count += 1
      total_compute_us += compute_us
      total_bytes_in += len(data) + 4
      total_bytes_out += len(response_bytes) + 4
      elapsed = time.perf_counter() - last_stat_time
      if elapsed >= 2.0:
        fps = request_count / elapsed
        avg_compute_ms = (total_compute_us / request_count) / 1000 if request_count > 0 else 0
        in_mib_s, out_mib_s = (total_bytes_in / elapsed) / (1 << 20), (total_bytes_out / elapsed) / (1 << 20)
        print(f"stats: {fps:5.1f} fps | avg_compute: {avg_compute_ms:5.1f} ms | in: {in_mib_s:4.2f} MiB/s | out: {out_mib_s:4.2f} MiB/s")
        last_stat_time, request_count, total_compute_us, total_bytes_in, total_bytes_out = time.perf_counter(), 0, 0, 0, 0

    except (TimeoutError, OSError, ValueError, ConnectionError, struct.error) as e:
      print(f"Connection error with {addr}: {e}")
      break

def main():
  print(f"remote-vision-policy-server ready on {HOST}:{PORT}")
  with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as srv:
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, PORT))
    srv.listen()
    while True:
      conn, addr = srv.accept()
      handle_connection(conn, addr)

if __name__ == "__main__":
  try:
    main()
  except KeyboardInterrupt:
    print("Server shutting down.")

