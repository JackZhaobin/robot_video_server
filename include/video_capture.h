/*
 * @Description:
 * @Version: V1.0
 * @Author: hongyuan.liu@corenetic.ai
 * @Date: 2025-03-17 10:56:09
 * @LastEditors: hongyuan.liu@corenetic.ai
 * @LastEditTime: 2025-04-21 10:53:59
 * Copyright (C) 2024-2050 Corenetic Technology Inc All rights reserved.
 */
#pragma once

#include <atomic>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>

#include "libobsensor/ObSensor.hpp"

enum RawFmt_t : int32_t {
  RAW_FORMAT_RGB = 0,
  RAW_FORMAT_NV12 = 1,
  RAW_FORMAT_BUTT
};

class VideoCapture {
 public:
  using FrameCallback = std::function<void(
      uint8_t* data, uint32_t len, uint32_t width, uint32_t height, int32_t fmt,
      uint64_t timestamp, int32_t id)>;

  VideoCapture(int32_t id, const std::string& serial_num, uint32_t width = 1280,
               uint32_t height = 720, uint32_t fps = 30,
               int32_t fmt = RAW_FORMAT_RGB);
  ~VideoCapture();

  bool start(FrameCallback frameCallback);
  void stop();

 private:
  bool initContex();
  std::shared_ptr<ob::Device> selectDeviceBySerialNumber(
      const std::shared_ptr<ob::DeviceList>& list,
      const std::string& servial_number);

 private:
  std::atomic_bool m_b_init_{false};

  std::shared_ptr<ob::Context> m_p_ctx_;
  std::shared_ptr<ob::Pipeline> m_p_pipeline_;
  std::shared_ptr<ob::Config> m_p_config_ = nullptr;
  std::shared_ptr<ob::DeviceList> m_device_list_ = nullptr;
  std::shared_ptr<ob::Device> m_p_device_ = nullptr;

  int32_t m_id_ = -1;
  std::string m_serial_number_ = "";
  uint32_t m_width_ = 0;
  uint32_t m_height_ = 0;
  uint32_t m_fps_ = 0;
  int32_t m_fmt_ = 0;
  FrameCallback m_frame_callback_ = nullptr;
};