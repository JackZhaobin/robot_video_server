/*
 * @Description:
 * @Version: V1.0
 * @Author: hongyuan.liu@corenetic.ai
 * @Date: 2025-03-15 04:42:20
 * @LastEditors: hongyuan.liu@corenetic.ai
 * @LastEditTime: 2025-04-22 17:30:08
 * Copyright (C) 2024-2050 Corenetic Technology Inc All rights reserved.
 */
#include "rtsp_server_wrapper.h"
#include "yllog.h"
#include "utils.hpp"

RtspServerWrapper::RtspServerWrapper() {}

RtspServerWrapper::~RtspServerWrapper() {
  if (m_b_init_) {
    StopRtspServer();
    m_b_init_ = false;
  }
}

bool RtspServerWrapper::init(int32_t server_port, int32_t n_chn) {
  int32_t ret = 0;

  if (server_port <= 0 || n_chn <= 0) {
    return false;
  }

  ret = StartRTSPServer(server_port, n_chn, 0, NULL);
  if (ret < 0) {
    YLLOG_ERR("Start rtspserver failed!");
    return false;
  }

  YLLOG_INFO("Start rtspserver success.");

  m_server_port_ = server_port;
  m_chn_num_ = n_chn;
  m_b_init_ = true;

  return true;
}

void RtspServerWrapper::deinit() {
  if (m_b_init_) {
    StopRtspServer();
    m_b_init_ = false;
  }
}

bool RtspServerWrapper::sendFrame(const FrameHeader_t& frame_hdr,
                                  uint8_t* p_data, uint32_t len) {
  int32_t ret = 0;

  // uint32_t travel_time = getCurrentTimeUs() - frame_hdr.m_timestamp;
  // YLLOG_DBG("Image frame travel befor rtspserver: %u us", travel_time);

  ret = SendVideoStreamToRtspServer(
      frame_hdr.m_chn_id, frame_hdr.m_stream_id, frame_hdr.m_venc_type,
      frame_hdr.m_frame_type, frame_hdr.m_timestamp, len, p_data);

  return ret == 0;
}
