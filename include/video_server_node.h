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

// 新增深度压缩配置结构体
struct ImageCompressConfig_t {
  std::string m_name;
  uint32_t m_width;
  uint32_t m_height;
  uint32_t m_fps;
  int32_t  m_fmt;
  std::string m_topic;
  bool     m_b_enable;
};

// 修改视频流配置结构体，添加topic字段
struct ImageStreamConfig_t {
  std::string m_name;
  uint32_t m_width;
  uint32_t m_height;
  uint32_t m_fps;
  int32_t  m_fmt;
  uint32_t m_bitrate;
  uint32_t m_rtsp_chn;
  std::string m_topic;  // 新增topic字段
  bool     m_b_enable;
};

// 移除原有的ImageDepthConfig_t，修改CameraConfig_t
struct CameraConfig_t {
  int32_t m_id;
  std::string m_name_;
  std::string m_serial_num;
  bool m_b_enable_compress;  // 修改字段名
  bool m_b_enable_stream;
 
  // 移除原有的单独topic定义
  // std::string m_topic_color_image_raw;
  // std::string m_topic_depth_image_raw;
  // std::string m_topic_left_ir_image_raw;
  // std::string m_topic_right_ir_image_raw;

  // 移除原有的depth_image配置
  // ImageDepthConfig_t m_depth_image;
  
  std::vector<ImageStreamConfig_t> m_video_streams;
  std::vector<ImageCompressConfig_t> m_compress_streams;  // 新增压缩流配置
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

  bool setupSDKMultiStream();
  bool setupROS2MultiStream();
  bool setupVideoSource();

  void destroyVideoSource();

  bool setupVideoEncoder();
  void destroyVideoStreamEncoders();
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

 
  //std::map<int32_t, rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> m_depth_image_subs;
  std::map<int32_t, std::map<std::string, rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr>> m_compress_subs_;



  /*compress 编码1路*/
  // std::map<int32_t, std::shared_ptr<DepthImageEncoder>> m_depth_encoders_;
  // std::map<int32_t, CDataWriter*> m_depth_writers;
  // std::map<int32_t, uint32_t> m_depth_frame_ids;

   /*compress 编码多路*/
  std::map<int32_t, std::map<std::string, std::shared_ptr<DepthImageEncoder>>> m_compress_encoders_;
  std::map<int32_t, std::map<std::string, CDataWriter*>> m_compress_writers_;
  std::map<int32_t, std::map<std::string, uint32_t>> m_compress_frame_ids_;


  std::shared_ptr<CDDSWrapper> m_dds_wrapper = nullptr;
 
  


 
  std::map<int32_t, std::map<std::string, std::shared_ptr<VideoEncoder>>> m_stream_encoders_;
  std::map<int32_t, std::map<std::string, rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr>> m_image_subs_;
};
