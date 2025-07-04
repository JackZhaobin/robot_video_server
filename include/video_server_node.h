/*
 * @Description:
 * @Version: V1.0
 * @Author: hongyuan.liu@corenetic.ai
 * @Date: 2025-03-12 06:42:58
 * @LastEditors: hongyuan.liu@corenetic.ai
 * @LastEditTime: 2025-04-21 14:23:35
 * Copyright (C) 2024-2050 Corenetic Technology Inc All rights reserved.
 */
#pragma once

#include <cv_bridge/cv_bridge.h>
#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <memory>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <unordered_map>

#include "depth_encoder.h"
#include "video_capture.h"
#include "video_encoder.h"
#include "yhdds.h"
#include "yllog.h"

// enum { IMAGE_TYPE_COLOR = 0, IMAGE_TYPE_DEPTH, IMAGE_TYPE_END } ImageType_e;

struct ImageConfig_t {
  uint32_t m_width;
  uint32_t m_height;
  uint32_t m_fps;
  int32_t m_fmt;
  uint32_t m_bitrate;
};

struct CameraConfig_t {
  int32_t m_id;
  int32_t m_rtsp_chn;
  bool m_b_enable;
  bool m_b_enable_depth;
  std::string m_name_;
  std::string m_serial_num;
  std::string m_topic_color_image_raw;
  std::string m_topic_depth_image_raw;

  ImageConfig_t m_color_image;
  ImageConfig_t m_depth_image;
  ImageConfig_t m_enc_image;
};

class RobotVideoServer : public rclcpp::Node {
 public:
  RobotVideoServer();
  ~RobotVideoServer();

  bool init();
  void deinit();

 private:
  bool loadCameraConfig(const std::string& config_file_path);
  bool loadLoggerConfig(const std::string& config_file_path);
  bool initLogger();
  bool setupVideoSource();
  void destroyVideoSource();
  bool setupVideoEncoder();
  void destroyVideoEncoder();
  bool setupRtspServer();
  void destroyRtstpServer();

 private:
  std::string m_camera_config_path_ = "";
  std::string m_dds_config_path_ = "";
  std::string m_logger_config_path_ = "";
  
  bool m_b_use_sdk_ = true;
  uint32_t m_chn_offset_ = 0;

  std::map<int32_t, CameraConfig_t> m_camera_config_;
  LogParam_t m_log_param_;
  int32_t m_logger_ = 0;

  std::map<int32_t, std::shared_ptr<VideoCapture>> m_captures_;
  std::map<int32_t, std::shared_ptr<VideoEncoder>> m_encoders_;
  std::map<int32_t, std::shared_ptr<DepthImageEncoder>> m_depth_encoders_;
  std::map<int32_t, rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr>
      m_color_image_subs;
  std::map<int32_t, rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr>
      m_depth_image_subs;

  std::shared_ptr<CDDSWrapper> m_dds_wrapper = nullptr;
  std::map<int32_t, CDataWriter*> m_depth_writers;
  std::map<int32_t, uint32_t> m_depth_frame_ids;
};
