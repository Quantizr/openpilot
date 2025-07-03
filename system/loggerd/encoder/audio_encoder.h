#pragma once

#include <deque>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
}

#include "cereal/messaging/messaging.h"
#include "system/loggerd/loggerd.h"

class AudioEncoder {
public:
  AudioEncoder(AVCodecID codec_id = AV_CODEC_ID_AAC, int sample_rate = 16000, int bitrate = 32000);
  virtual ~AudioEncoder();

  void encoder_open();
  void encoder_close();
  void encode_audio_data(uint8_t *data, int len, uint64_t timestamp);

  // Segment management for coordination with video encoders
  void set_segment_num(int segment_num) { current_segment_num = segment_num; }
  int get_segment_num() const { return current_segment_num; }

private:
  void publisher_publish(int segment_num, uint32_t idx, uint64_t timestamp, kj::ArrayPtr<uint8_t> dat);
  void flush_encoder();

  AVCodecID codec_id;
  int sample_rate;
  int bitrate;
  // int frame_size; // Will be set after codec is opened

  // Audio codec context and frame
  AVCodecContext *audio_codec_ctx = nullptr;
  AVFrame *audio_frame = nullptr;

  // Audio encoding state
  std::deque<float> audio_buffer;
  uint64_t next_audio_pts = 0;
  uint64_t first_audio_timestamp = 0;
  int current_segment_num = 0;

  // Publishing
  std::unique_ptr<PubMaster> pm;
  std::vector<uint8_t> msg_cache;

  // Frame counter for encoding
  std::atomic<uint32_t> encode_idx_counter{0};
  bool is_open = false;
  bool extradata_sent = false;
};
