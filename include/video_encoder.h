/*
 * @Description:
 * @Version: V1.0
 * @Author: hongyuan.liu@corenetic.ai
 * @Date: 2025-03-13 05:21:50
 * @LastEditors: hongyuan.liu@corenetic.ai
 * @LastEditTime: 2025-04-21 16:59:21
 * Copyright (C) 2024-2050 Corenetic Technology Inc All rights reserved.
 */
#pragma once

#include "NvBufSurface.h"
#include "NvVideoEncoder.h"
#include "common_def.h"
#include "data_queue.hpp"
#include "rtsp_server_wrapper.h"

typedef struct {
  NvVideoEncoder *m_enc = nullptr;

  uint32_t m_encoder_pixfmt = V4L2_PIX_FMT_H264;
  uint32_t m_raw_pixfmt = V4L2_PIX_FMT_YUV420M;
  uint32_t m_src_pixfmt = V4L2_PIX_FMT_YUV420M;
  uint32_t m_width = 1920;
  uint32_t m_height = 1080;

  uint32_t m_bitrate = 4 * 1024 * 1024;
  bool m_enable_ratecontrol = true;
  uint32_t m_profile = V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE;
  uint32_t m_level = -1;
  enum v4l2_mpeg_video_bitrate_mode m_ratecontrol =
      V4L2_MPEG_VIDEO_BITRATE_MODE_CBR;
  uint32_t m_iframe_interval = 30;
  uint32_t m_idr_interval = 256;
  uint32_t m_fps_n = 30;
  uint32_t m_fps_d = 1;
  uint32_t m_encode_width = 1920;
  uint32_t m_encode_height = 1080;
  uint32_t m_num_reference_frames = 1;
  uint32_t m_num_b_frames = 0;
  enum v4l2_enc_hw_preset_type m_hw_preset_type = V4L2_ENC_HW_PRESET_ULTRAFAST;
  enum v4l2_memory m_output_memory_type = V4L2_MEMORY_DMABUF;
  enum v4l2_memory m_capture_memory_type = V4L2_MEMORY_MMAP;
  int32_t m_output_plane_fd[32];
  int32_t m_capture_plane_fd[32];
  bool m_enable_extended_colorformat = false;
  uint32_t m_input_frames_queued_count = 0;
  uint32_t m_num_output_buffers = 6;

  bool m_blocking_mode = true;
  bool m_enableLossless = false;
  bool m_insert_sps_pps_at_idr = true;
  bool m_b_use_enc_cmd = false;

  bool m_b_output_buf_alloc = false;
  bool m_b_capture_buf_alloc = false;
  bool m_b_copy_timestamp = false;

  uint32_t m_ouput_buf_idx = 0;
} EncoderContext_t;

typedef struct {
  uint32_t m_raw_pixfmt = V4L2_PIX_FMT_YUV420M;
  uint32_t m_width = 1920;
  uint32_t m_height = 1080;

  uint32_t m_encoder_pixfmt = 0;  // 0-H264, 1-H265
  uint32_t m_encode_width = 1920;
  uint32_t m_encode_height = 1080;
  uint32_t m_bitrate = 10 * 1024 * 1024;
  uint32_t m_ratecontrol = 1;  // 0-VBR, 1-CBR
  uint32_t m_fps = 30;

  int32_t m_rtsp_chn_id = -1;
} EncCreateParam_t;

class VideoEncoder {
 public:
  VideoEncoder(uint32_t chn_id, const EncCreateParam_t &enc_param);
  ~VideoEncoder();

  bool start();
  void stop();
  bool putImage(Image_t *image_ptr);
  bool processFrame(struct v4l2_buffer *v4l2_buf, NvBuffer *buffer,
                    NvBuffer *shared_buffer);
 private:
  void setDefaults();
  bool createEncoder();
  void destroyEncoder();
  void asyncResetEncoder();
  bool startStream();
  void stopStream();
  bool setupOutputDmaBuf(uint32_t num_buffers);
  void destroyOutputDmaBuf();
  bool allocImageBuf();
  void freeImageBuf();
  bool setupCaptureDmaBuf(uint32_t num_buffers);
  void destroyCaptureDmaBuf();
  bool allocStreamBuf();
  void freeStreamBuf();
  bool enqueueStreamBuf();
  void encodeThread();
  bool fillVideoFrame(NvBuffer &buffer, uint8_t *src_data);
  bool enqueueImageBuf(uint32_t idx, Image_t *image_ptr);
  bool pushImage(Image_t *image_ptr);
  bool doEncode(Image_t *image_ptr);
  bool insertTimestamp(uint8_t *data, uint64_t timestamp);
  void fps();

 private:
  std::atomic_bool m_b_init_{false};
  EncoderContext_t m_ctx_;

  uint32_t m_chn_id_ = -1;
  std::atomic_bool m_b_reseting_{false};

  int32_t m_rtsp_chn_id_ = -1;

  std::unique_ptr<DataQueue<Image_t *>> m_image_queue_;

  std::atomic_bool m_b_encode_{false};
  std::shared_ptr<std::thread> m_encode_thread_;

  std::atomic_uint32_t m_frame_count_{0};

  uint64_t m_last_time_ = 0;
  uint32_t m_count_ = 0;
};
