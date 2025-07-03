#include <cassert>

#include "system/loggerd/loggerd.h"
#include "system/loggerd/encoder/jpeg_encoder.h"
#include "system/loggerd/encoder/audio_encoder.h"

#ifdef QCOM2
#include "system/loggerd/encoder/v4l_encoder.h"
#define Encoder V4LEncoder
#else
#include "system/loggerd/encoder/ffmpeg_encoder.h"
#define Encoder FfmpegEncoder
#endif

ExitHandler do_exit;

struct EncoderdState {
  int max_waiting = 0;

  // Sync logic for startup
  std::atomic<int> encoders_ready = 0;
  std::atomic<uint32_t> start_frame_id = 0;
  bool camera_ready[VISION_STREAM_WIDE_ROAD + 1] = {};
  bool camera_synced[VISION_STREAM_WIDE_ROAD + 1] = {};

  bool audio_ready = false;
  bool audio_synced = false;
  uint64_t audio_start_frame_id = 0;
};

// Handle initial encoder syncing by waiting for all encoders to reach the same frame id
bool sync_encoders(EncoderdState *s, VisionStreamType cam_type, uint32_t frame_id) {
  if (s->camera_synced[cam_type]) return true;

  if (s->max_waiting > 1 && s->encoders_ready != s->max_waiting) {
    // add a small margin to the start frame id in case one of the encoders already dropped the next frame
    update_max_atomic(s->start_frame_id, frame_id + 2);
    if (std::exchange(s->camera_ready[cam_type], true) == false) {
      ++s->encoders_ready;
      LOGD("camera %d encoder ready", cam_type);
    }
    return false;
  } else {
    if (s->max_waiting == 1) update_max_atomic(s->start_frame_id, frame_id);
    bool synced = frame_id >= s->start_frame_id;
    s->camera_synced[cam_type] = synced;
    if (!synced) LOGD("camera %d waiting for frame %d, cur %d", cam_type, (int)s->start_frame_id, frame_id);
    return synced;
  }
}

bool sync_audio_encoder(EncoderdState *s, uint64_t frame_id) {
  if (s->audio_synced) return true;
  if (s->max_waiting > 1 && s->encoders_ready != s->max_waiting) {
    s->audio_start_frame_id = frame_id;
    if (std::exchange(s->audio_ready, true) == false) {
      ++s->encoders_ready;
      LOGD("audio encoder ready");
    }
    return false;
  } else {
    s->audio_start_frame_id = frame_id;
    s->audio_synced = true;
    return true;
  }
}

void encoder_thread(EncoderdState *s, const LogCameraInfo &cam_info) {
  util::set_thread_name(cam_info.thread_name);

  std::vector<std::unique_ptr<Encoder>> encoders;
  VisionIpcClient vipc_client = VisionIpcClient("camerad", cam_info.stream_type, false);

  std::unique_ptr<JpegEncoder> jpeg_encoder;

  int cur_seg = 0;
  while (!do_exit) {
    if (!vipc_client.connect(false)) {
      util::sleep_for(5);
      continue;
    }

    // init encoders
    if (encoders.empty()) {
      const VisionBuf &buf_info = vipc_client.buffers[0];
      LOGW("encoder %s init %zux%zu", cam_info.thread_name, buf_info.width, buf_info.height);
      assert(buf_info.width > 0 && buf_info.height > 0);

      for (const auto &encoder_info : cam_info.encoder_infos) {
        auto &e = encoders.emplace_back(new Encoder(encoder_info, buf_info.width, buf_info.height));
        e->encoder_open();
      }

      // Only one thumbnail can be generated per camera stream
      if (auto thumbnail_name = cam_info.encoder_infos[0].thumbnail_name) {
        jpeg_encoder = std::make_unique<JpegEncoder>(thumbnail_name, buf_info.width / 4, buf_info.height / 4);
      }
    }

    bool lagging = false;
    while (!do_exit) {
      VisionIpcBufExtra extra;
      VisionBuf* buf = vipc_client.recv(&extra);
      if (buf == nullptr) continue;

      // detect loop around and drop the frames
      if (buf->get_frame_id() != extra.frame_id) {
        if (!lagging) {
          LOGE("encoder %s lag  buffer id: %" PRIu64 " extra id: %d", cam_info.thread_name, buf->get_frame_id(), extra.frame_id);
          lagging = true;
        }
        continue;
      }
      lagging = false;

      if (!sync_encoders(s, cam_info.stream_type, extra.frame_id)) {
        continue;
      }
      if (do_exit) break;

      // do rotation if required
      const int frames_per_seg = SEGMENT_LENGTH * MAIN_FPS;
      if (cur_seg >= 0 && extra.frame_id >= ((cur_seg + 1) * frames_per_seg) + s->start_frame_id) {
        for (auto &e : encoders) {
          e->encoder_close();
          e->encoder_open();
        }
        ++cur_seg;
      }

      // encode a frame
      for (int i = 0; i < encoders.size(); ++i) {
        int out_id = encoders[i]->encode_frame(buf, &extra);

        if (out_id == -1) {
          LOGE("Failed to encode frame. frame_id: %d", extra.frame_id);
        }
      }

      if (jpeg_encoder && (extra.frame_id % 1200 == 100)) {
        jpeg_encoder->pushThumbnail(buf, extra);
      }
    }
  }
}

void audio_encoder_thread(EncoderdState *s) { // TODO: this should've just been in encoder_thread, by checking encoder_info.has_audio and rotating with the corresponding encoder_thread. This needs a audioEncodeData per encode stream though.
  util::set_thread_name("audio_encoder");

  const int sample_rate = 16000;
  const int bitrate = 32000;

  AudioEncoder audio_encoder(AV_CODEC_ID_AAC, sample_rate, bitrate);
  std::unique_ptr<Context> ctx(Context::create());
  SubSocket *sock = SubSocket::create(ctx.get(), "rawAudioData");
  assert(sock != NULL);
  int cur_seg = 0;
  bool encoder_opened = false;
  while (!do_exit) {
    if (!encoder_opened) {
      audio_encoder.encoder_open();
      encoder_opened = true;
      LOGD("Audio encoder opened for segment %d", cur_seg);
    }

    Message *msg = sock->receive(true);
    if (msg != nullptr) {
      capnp::FlatArrayMessageReader cmsg(kj::ArrayPtr<capnp::word>((capnp::word *)msg->getData(), msg->getSize() / sizeof(capnp::word)));
      auto event = cmsg.getRoot<cereal::Event>();
      auto audio_data = event.getRawAudioData().getData();
      uint64_t start_frame_idx = event.getRawAudioData().getStartFrameIdx();

      if (!sync_audio_encoder(s, start_frame_idx)) continue;

      const int samples_per_seg = SEGMENT_LENGTH * sample_rate;
      // LOGE("segment num %f", (float) start_frame_idx/samples_per_seg);
      if (cur_seg >= 0 && start_frame_idx >= ((cur_seg + 1) * samples_per_seg) + s->audio_start_frame_id) {
        LOGE("Audio encoder rotating from segment %d to %d", cur_seg, cur_seg + 1);
        audio_encoder.encoder_close();
        audio_encoder.encoder_open();
        ++cur_seg;
      }

      audio_encoder.encode_audio_data((uint8_t*)audio_data.begin(), audio_data.size(), event.getLogMonoTime()); // convert to microseconds
      delete msg;
    }
    util::sleep_for(10);
  }

  audio_encoder.encoder_close();
  delete sock;
}

template <size_t N>
void encoderd_thread(const LogCameraInfo (&cameras)[N]) {
  EncoderdState s;

  std::set<VisionStreamType> streams;
  while (!do_exit) {
    streams = VisionIpcClient::getAvailableStreams("camerad", false);
    if (!streams.empty()) {
      break;
    }
    util::sleep_for(100);
  }

  if (!streams.empty()) {
    std::vector<std::thread> encoder_threads;
    for (auto stream : streams) {
      auto it = std::find_if(std::begin(cameras), std::end(cameras),
                             [stream](auto &cam) { return cam.stream_type == stream; });
      assert(it != std::end(cameras));
      ++s.max_waiting;
      encoder_threads.push_back(std::thread(encoder_thread, &s, *it));
    }

    bool has_audio = false;
    for (const auto &cam : cameras) {
      for (const auto &encoder_info : cam.encoder_infos) {
        if (encoder_info.include_audio) {
          has_audio = true;
          break;
        }
      }
      if (has_audio) break;
    }
    std::thread audio_thread;
    if (has_audio) {
      LOGE("Starting audio encoder thread");
      ++s.max_waiting;
      audio_thread = std::thread(audio_encoder_thread, &s);
    }

    for (auto &t : encoder_threads) t.join();
    if (audio_thread.joinable()) audio_thread.join();
  }
}

int main(int argc, char* argv[]) {
  if (!Hardware::PC()) {
    int ret;
    ret = util::set_realtime_priority(52);
    assert(ret == 0);
    ret = util::set_core_affinity({3});
    assert(ret == 0);
  }
  if (argc > 1) {
    std::string arg1(argv[1]);
    if (arg1 == "--stream") {
      encoderd_thread(stream_cameras_logged);
    } else {
      LOGE("Argument '%s' is not supported", arg1.c_str());
    }
  } else {
    encoderd_thread(cameras_logged);
  }
  return 0;
}
