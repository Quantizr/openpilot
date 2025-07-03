#include "system/loggerd/encoder/audio_encoder.h"

#include <algorithm>
#include <cassert>
#include <cstring>

#include "cereal/messaging/messaging.h"
#include "common/swaglog.h"
#include "common/util.h"

AudioEncoder::AudioEncoder(AVCodecID codec_id, int sample_rate, int bitrate)
  : codec_id(codec_id), sample_rate(sample_rate), bitrate(bitrate) {
  pm.reset(new PubMaster({"audioEncodeData"}));
}

AudioEncoder::~AudioEncoder() {
  encoder_close();
}

void AudioEncoder::encoder_open() {
  if (is_open) {
    encoder_close();
  }

  // Find AAC encoder - copied from video_writer.cc
  const AVCodec *audio_avcodec = avcodec_find_encoder(this->codec_id);
  assert(audio_avcodec);

  // Allocate codec context - copied from video_writer.cc
  this->audio_codec_ctx = avcodec_alloc_context3(audio_avcodec);
  assert(this->audio_codec_ctx);

  // Set audio codec parameters - copied from video_writer.cc
  this->audio_codec_ctx->sample_fmt = AV_SAMPLE_FMT_FLTP;
  this->audio_codec_ctx->sample_rate = this->sample_rate; // from system/micd.py
  #if LIBAVUTIL_VERSION_INT >= AV_VERSION_INT(57, 28, 100) // FFmpeg 5.1+
  av_channel_layout_default(&this->audio_codec_ctx->ch_layout, 1);
  #else
  this->audio_codec_ctx->channel_layout = AV_CH_LAYOUT_MONO;
  #endif
  this->audio_codec_ctx->bit_rate = this->bitrate;
  this->audio_codec_ctx->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;

  // Open codec - copied from video_writer.cc
  int err = avcodec_open2(this->audio_codec_ctx, audio_avcodec, NULL);
  assert(err >= 0);

  // Suppress "QAvg" info messages - copied from video_writer.cc
  av_log_set_level(AV_LOG_WARNING);

  // Allocate audio frame - copied from video_writer.cc
  this->audio_frame = av_frame_alloc();
  assert(this->audio_frame);

  this->audio_frame->format = this->audio_codec_ctx->sample_fmt;
  #if LIBAVUTIL_VERSION_INT >= AV_VERSION_INT(57, 28, 100) // FFmpeg 5.1+
  av_channel_layout_copy(&this->audio_frame->ch_layout, &this->audio_codec_ctx->ch_layout);
  #else
  this->audio_frame->channel_layout = this->audio_codec_ctx->channel_layout;
  #endif
  this->audio_frame->sample_rate = this->audio_codec_ctx->sample_rate;
  this->audio_frame->nb_samples = this->audio_codec_ctx->frame_size;

  int ret = av_frame_get_buffer(this->audio_frame, 0);
  if (ret < 0) {
    LOGE("AUDIO: Failed to allocate frame buffer: %d", ret);
    av_frame_free(&this->audio_frame);
    this->audio_frame = nullptr;
    return;
  }

  // Reset encoding state
  audio_buffer.clear();
  next_audio_pts = 0;
  first_audio_timestamp = 0;
  encode_idx_counter = 0;
  current_segment_num++;

  is_open = true;
  LOGD("Audio encoder opened for segment %d", current_segment_num);
}

void AudioEncoder::encoder_close() {
  if (!is_open) return;

  // Flush any remaining audio data
  flush_encoder();

  // Clean up codec context and frame - copied from video_writer.cc
  if (audio_codec_ctx) {
    avcodec_free_context(&audio_codec_ctx);
    audio_codec_ctx = nullptr;
  }

  if (audio_frame) {
    av_frame_free(&audio_frame);
    audio_frame = nullptr;
  }

  is_open = false;
  LOGD("Audio encoder closed");
}

void AudioEncoder::encode_audio_data(uint8_t *data, int len, uint64_t timestamp) {
  if (!is_open || !audio_codec_ctx || !audio_frame) {
    LOGW("Audio encoder not ready, dropping audio data");
    return;
  }

  // Sync the timestampEof of first video packet with the logMonoTime of first audio packet
  if (first_audio_timestamp == 0) {
    first_audio_timestamp = timestamp; // nanoseconds
  }

  // Convert s16le samples to fltp and add to buffer
  const int16_t *raw_samples = reinterpret_cast<const int16_t*>(data);
  int sample_count = len / sizeof(int16_t);
  constexpr float normalizer = 1.0f / 32768.0f;
  const size_t original_size = audio_buffer.size();
  audio_buffer.resize(original_size + sample_count);
  std::transform(raw_samples, raw_samples + sample_count, audio_buffer.begin() + original_size,
                [](int16_t sample) { return sample * normalizer; });

  while (audio_buffer.size() >= audio_codec_ctx->frame_size) {
    audio_frame->pts = next_audio_pts;

    float *f_samples = reinterpret_cast<float*>(audio_frame->data[0]);
    std::copy(audio_buffer.begin(),  audio_buffer.begin() + audio_codec_ctx->frame_size, f_samples);
    audio_buffer.erase(audio_buffer.begin(), audio_buffer.begin() + audio_codec_ctx->frame_size);

    int send_result = avcodec_send_frame(audio_codec_ctx, audio_frame);
    if (send_result >= 0) {
      AVPacket *pkt = av_packet_alloc();
      while (avcodec_receive_packet(audio_codec_ctx, pkt) == 0) {
        uint64_t time_diff_ns = (audio_frame->pts * 1000000000ULL) / audio_codec_ctx->sample_rate;
        uint64_t synchronized_time = first_audio_timestamp + time_diff_ns;

        publisher_publish(current_segment_num, encode_idx_counter++, synchronized_time, kj::arrayPtr(pkt->data, pkt->size));

        av_packet_unref(pkt);
      }
      av_packet_free(&pkt);
    } else {
      LOGW("AUDIO: Failed to send audio frame to encoder: %d", send_result);
    }

    next_audio_pts += audio_codec_ctx->frame_size;
  }
}

void AudioEncoder::flush_encoder() {
  if (!is_open || !audio_codec_ctx) return;
  audio_frame->pts = next_audio_pts;

  // Flush encoder with NULL frame - copied from video_writer.cc
  int send_result = avcodec_send_frame(audio_codec_ctx, NULL);
  if (send_result >= 0) {
    AVPacket *pkt = av_packet_alloc();
    while (avcodec_receive_packet(audio_codec_ctx, pkt) == 0) {
      // Calculate synchronized timestamp for final packets
      uint64_t time_diff_us = (next_audio_pts * 1000000ULL) / audio_codec_ctx->sample_rate;
      uint64_t synchronized_time = first_audio_timestamp + time_diff_us;

      publisher_publish(current_segment_num, encode_idx_counter++, synchronized_time, kj::arrayPtr(pkt->data, pkt->size));

      av_packet_unref(pkt);
    }
    av_packet_free(&pkt);
  }
}

void AudioEncoder::publisher_publish(int segment_num, uint32_t idx, uint64_t timestamp, kj::ArrayPtr<uint8_t> dat) {
  // Follow VideoEncoder pattern for publishing
  MessageBuilder msg;
  auto event = msg.initEvent(true);

  // Set up audio encode data message
  auto encodeData = event.initAudioEncodeData();
  // TODO: send extradata in a header
  auto edata = encodeData.initIdx();

  encodeData.setCodecId(codec_id);
  encodeData.setBitrate(bitrate);
  encodeData.setSampleRate(sample_rate);
  // encodeData.setStartTimestamp(timestamp);
  // encodeData.setSegmentNum(segment_num);
  // encodeData.setSegmentId(idx);

  encodeData.adoptData(msg.getOrphanage().referenceExternalData(dat));

  // edata.setFrameId(extra.frame_id);
  // edata.setTimestampSof(extra.timestamp_sof);
  edata.setTimestampEof(timestamp);
  // edata.setType(encoder_info.encode_type);
  // edata.setEncodeId(cnt++);
  edata.setSegmentNum(segment_num);
  edata.setSegmentId(idx);
  // edata.setFlags(flags);
  edata.setLen(dat.size());


  // Send message - following VideoEncoder pattern
  uint32_t bytes_size = capnp::computeSerializedSizeInWords(msg) * sizeof(capnp::word);
  if (msg_cache.size() < bytes_size) {
    msg_cache.resize(bytes_size);
  }

  kj::ArrayOutputStream output_stream(kj::ArrayPtr<uint8_t>(msg_cache.data(), bytes_size));
  capnp::writeMessage(output_stream, msg);
  pm->send("audioEncodeData", msg_cache.data(), bytes_size);
}
