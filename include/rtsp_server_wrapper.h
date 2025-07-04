/*
 * @Description:
 * @Version: V1.0
 * @Author: hongyuan.liu@corenetic.ai
 * @Date: 2025-03-15 04:41:39
 * @LastEditors: hongyuan.liu@corenetic.ai
 * @LastEditTime: 2025-03-21 10:10:16
 * Copyright (C) 2024-2050 Corenetic Technology Inc All rights reserved.
 */
#pragma once

#include <atomic>
#include <iostream>

#include "librtspserver.h"

struct FrameHeader_t {
  int32_t m_chn_id;
  int32_t m_stream_id;
  int32_t m_venc_type;
  int32_t m_frame_type;
  uint64_t m_timestamp;
};

class RtspServerWrapper {
 public:
  RtspServerWrapper();

  ~RtspServerWrapper();

  static RtspServerWrapper* getInstance() {
    static RtspServerWrapper obj;
    return &obj;
  }

  bool init(int32_t server_port, int32_t n_chn);

  void deinit();

  bool sendFrame(const FrameHeader_t& frame_hdr, uint8_t* p_data, uint32_t len);

 private:
  int32_t m_server_port_ = 554;
  int32_t m_chn_num_ = 3;
  std::atomic_bool m_b_init_{false};
};
