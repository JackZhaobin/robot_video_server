/*
 * @Description:
 * @Version: V1.0
 * @Author: hongyuan.liu@corenetic.ai
 * @Date: 2025-03-17 10:56:41
 * @LastEditors: hongyuan.liu@corenetic.ai
 * @LastEditTime: 2025-04-21 11:37:51
 * Copyright (C) 2024-2050 Corenetic Technology Inc All rights reserved.
 */
#include "video_capture.h"
#include "yllog.h"

VideoCapture::VideoCapture(int32_t id, const std::string& serial_num,
                           uint32_t width, uint32_t height, uint32_t fps,
                           int32_t fmt)
    : m_id_(id),
      m_serial_number_(serial_num),
      m_width_(width),
      m_height_(height),
      m_fps_(fps),
      m_fmt_(fmt) {}

VideoCapture::~VideoCapture() { stop(); }

bool VideoCapture::initContex() {
  m_p_ctx_ = std::make_shared<ob::Context>();
  m_device_list_ = m_p_ctx_->queryDeviceList();
  m_p_device_ = selectDeviceBySerialNumber(m_device_list_, m_serial_number_);
  if (!m_p_device_) {
    YLLOG_ERR(
        "Can't find device serial number %s .", m_serial_number_.c_str());
    return false;
  }

  m_p_pipeline_ = std::make_shared<ob::Pipeline>(m_p_device_);
  if (!m_p_pipeline_) {
    YLLOG_ERR("Create pipeline failed!");
    return false;
  }

  m_p_config_ = std::make_shared<ob::Config>();

  static bool bPrint = true;

  if (bPrint) {
    for (uint32_t i = 0; i < m_device_list_->getCount(); i++) {
      auto device = m_device_list_->getDevice(i);
      auto device_info = device->getDeviceInfo();

      YLLOG_INFO(
          "name: %s, serial number: %s", device_info->name(),
          device_info->serialNumber());
    }

    bPrint = false;
  }

  return true;
}

std::shared_ptr<ob::Device> VideoCapture::selectDeviceBySerialNumber(
    const std::shared_ptr<ob::DeviceList>& list,
    const std::string& servial_number) {
  for (uint32_t i = 0; i < list->getCount(); i++) {
    auto device = list->getDevice(i);
    auto device_info = device->getDeviceInfo();
    if (strcmp(device_info->serialNumber(), servial_number.c_str()) == 0) {
      return device;
    }
  }

  return nullptr;
}

bool VideoCapture::start(FrameCallback frameCallback) {
  if (!initContex()) {
    return false;
  }

  // sync_config.syncMode = OB_MULTI_DEVICE_SYNC_MODE_STANDALONE;
  // sync_config.depthDelayUs = 0;
  // sync_config.colorDelayUs = 0;
  // sync_config.trigger2ImageDelayUs = 0;
  // sync_config.triggerOutDelayUs = 0;
  // sync_config.triggerOutEnable = 0;
  // sync_config.framesPerTrigger = 1;
  // device->setMultiDeviceSyncConfig(sync_config);

  // auto sync_config = p_device_->getMultiDeviceSyncConfig();
  // std::cout << "syncMode: " << sync_config.syncMode << std::endl;
  // std::cout << "depthDelayUs: " << sync_config.depthDelayUs << std::endl;
  // std::cout << "colorDelayUs: " << sync_config.colorDelayUs << std::endl;
  // std::cout << "trigger2ImageDelayUs: " << sync_config.trigger2ImageDelayUs
  // << std::endl; std::cout << "triggerOutEnable: " <<
  // sync_config.triggerOutEnable << std::endl; std::cout << "framesPerTrigger:
  // " << sync_config.framesPerTrigger << std::endl;

  // 开启color流
  auto colorProfiles = m_p_pipeline_->getStreamProfileList(OB_SENSOR_COLOR);

  YLLOG_DBG(
      "serial_num: %s, m_width_: %u, m_height_: %u, m_fps_: %u, m_fmt_: %d",
      m_serial_number_.c_str(), m_width_, m_height_, m_fps_, m_fmt_);

  auto colorProfile = colorProfiles->getVideoStreamProfile(
      m_width_, m_height_,
      m_fmt_ == RAW_FORMAT_NV12 ? OB_FORMAT_NV12 : OB_FORMAT_RGB, m_fps_);
  m_p_config_->enableStream(colorProfile);

  auto ob_callback = [this](std::shared_ptr<ob::FrameSet> frameset) {
    auto colorFrame = frameset->colorFrame();
    if (colorFrame) {
      if (m_frame_callback_) {
        m_frame_callback_(
            static_cast<uint8_t*>(colorFrame->data()), colorFrame->dataSize(),
            colorFrame->getWidth(), colorFrame->getHeight(),
            (int32_t)(colorFrame->format()), colorFrame->timeStamp(), m_id_);
      }
    }
  };

  m_frame_callback_ = frameCallback;

  m_p_pipeline_->start(m_p_config_, ob_callback);

  m_b_init_ = true;

  return true;
}

void VideoCapture::stop() {
  if (m_b_init_) {
    m_p_pipeline_->stop();
    m_b_init_ = false;
  }
}