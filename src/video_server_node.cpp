/*
 * @Description:
 * @Version: V1.0
 * @Author: hongyuan.liu@corenetic.ai
 * @Date: 2025-03-12 06:43:18
 * @LastEditors: hongyuan.liu@corenetic.ai
 * @LastEditTime: 2025-04-22 17:30:33
 * Copyright (C) 2024-2050 Corenetic Technology Inc All rights reserved.
 */
#include "video_server_node.h"

#include "nlohmann/json.hpp"
#include "rtsp_server_wrapper.h"
#include "utils.hpp"
#include "video_capture.h"
#include "yllog.h"

RobotVideoServer::RobotVideoServer() : Node("robot_video_server") {
  this->declare_parameter(
      "config_file",
      "/home/corenetic/robot_workspace/install/robot_video_server/share/"
      "robot_video_server/config/camera_config.json");

  this->declare_parameter(
      "dds_config_file",
      "/home/corenetic/robot_workspace/install/robot_video_server/share/"
      "robot_video_server/config/dds_config.json");

  this->declare_parameter(
    "logger_config_file",
    "/home/corenetic/robot_workspace/install/robot_video_server/share/"
    "robot_video_server/config/logger_config.json");

  m_camera_config_path_ = this->get_parameter("config_file").as_string();
  m_dds_config_path_ = this->get_parameter("dds_config_file").as_string();
  m_logger_config_path_ = this->get_parameter("logger_config_file").as_string();

  m_dds_wrapper = std::make_shared<CDDSWrapper>(m_dds_config_path_);

  if(!init()) {
    throw std::runtime_error("RobotVideoServer init failed");
  }
}

RobotVideoServer::~RobotVideoServer() { deinit(); }

bool RobotVideoServer::setupVideoSource() {
  if (m_b_use_sdk_) {
    for (auto &camera : m_camera_config_) {
      if (!camera.second.m_b_enable) {
        continue;
      }

      auto p_video_capture = std::make_shared<VideoCapture>(
          camera.second.m_id, camera.second.m_serial_num,
          camera.second.m_color_image.m_width,
          camera.second.m_color_image.m_height,
          camera.second.m_color_image.m_fps, camera.second.m_color_image.m_fmt);

      auto FrameCallback = [this](uint8_t *data, uint32_t len, uint32_t width,
                                  uint32_t height, int32_t fmt,
                                  uint64_t timestamp, int32_t id) {
        (void)len;
        (void)fmt;

        // 打印函数参数
        // std::cout << "Length: " << len << std::endl;
        // std::cout << "Width: " << width << std::endl;
        // std::cout << "Height: " << height << std::endl;
        // std::cout << "Format: " << fmt << std::endl;
        // std::cout << "Timestamp: " << timestamp << std::endl;
        // std::cout << "id: " << id << std::endl;

#if 0
                Image_t *image_ptr = (Image_t *)malloc(sizeof(Image_t) + len);
                image_ptr->m_width = width;
                image_ptr->m_height = height;
                image_ptr->m_pixfmt = fmt;
                image_ptr->m_length = len;
                image_ptr->m_timestamp = timestamp;
                memcpy(image_ptr->m_data, data, len);
                
                m_encoders_[id]->putImage(image_ptr);

#else
        auto size = width * height * 3 / 2;

        Image_t *image_ptr = (Image_t *)malloc(sizeof(Image_t) + size);
        image_ptr->m_width = width;
        image_ptr->m_height = height;
        image_ptr->m_pixfmt = 0;
        image_ptr->m_length = size;
        image_ptr->m_timestamp = timestamp;

        convertRGB2YUV420M(data, width, height, image_ptr->m_data);

        m_encoders_[id]->putImage(image_ptr);

#endif
        // if(id == 2)
        // {
        //     displayRGBFrame(data, width, height);
        // }
      };

      if (p_video_capture->start(FrameCallback)) {
        m_captures_[camera.second.m_id] = p_video_capture;
      } else {
        RCLCPP_INFO(this->get_logger(), "Start capture failed!");
      }
    }
  } else {
    for (auto &camera : m_camera_config_) {
      if (!camera.second.m_b_enable) {
        continue;
      }

      auto ImageCallback = [this, id = camera.second.m_id](
                               const sensor_msgs::msg::Image::SharedPtr msg) {
        auto size = msg->width * msg->height * 3 / 2;

        Image_t *image_ptr = (Image_t *)malloc(sizeof(Image_t) + size);
        image_ptr->m_width = msg->width;
        image_ptr->m_height = msg->height;
        image_ptr->m_pixfmt = 0;
        image_ptr->m_length = size;
        
        uint64_t sec = msg->header.stamp.sec;
        uint64_t nsec = msg->header.stamp.nanosec;
        image_ptr->m_timestamp = sec * 1000000ULL + nsec / 1000ULL;
   
        // image_ptr->m_timestamp = getCurrentTimeUs();
        YLLOG_DBG("Camera %d: timestamp %lu, current time %lu, diff %u us.", id, image_ptr->m_timestamp, getCurrentTimeUs(), getCurrentTimeUs()-image_ptr->m_timestamp);

        convertRGB2YUV420M(msg->data.data(), msg->width, msg->height,
                           image_ptr->m_data);

        m_encoders_[id]->putImage(image_ptr);
      };

      auto p_image_subscription =
          this->create_subscription<sensor_msgs::msg::Image>(
              camera.second.m_topic_color_image_raw, 10, ImageCallback);
      if (p_image_subscription) {
        m_color_image_subs[camera.second.m_id] = p_image_subscription;
      } else {
        YLLOG_ERR("create_subscription failed!");
      }

      // 订阅深度流
      if (camera.second.m_b_enable_depth) {
        auto DepthCallback = [this, id = camera.second.m_id](
                                 const sensor_msgs::msg::Image::SharedPtr msg) {
          m_depth_frame_ids[id]++;

          uint32_t size = msg->data.size();

          Image_t *image_ptr = (Image_t *)malloc(sizeof(Image_t) + size);
          image_ptr->m_id = m_depth_frame_ids[id];
          image_ptr->m_width = msg->width;
          image_ptr->m_height = msg->height;
          image_ptr->m_pixfmt = 0;
          image_ptr->m_length = size;
          
          uint64_t sec = msg->header.stamp.sec;
          uint64_t nsec = msg->header.stamp.nanosec;
          image_ptr->m_timestamp = sec * 1000000ULL + nsec / 1000ULL;

          // image_ptr->m_timestamp = getCurrentTimeUs();

          YLLOG_DBG("Depth %d: timestamp %lu, current time %lu, diff %u us.", id, image_ptr->m_timestamp, getCurrentTimeUs(), getCurrentTimeUs()-image_ptr->m_timestamp);

          memcpy(image_ptr->m_data, msg->data.data(), msg->data.size());

          m_depth_encoders_[id]->putImage(image_ptr);
        };

        // std::cout << "----------- m_topic_depth_image_raw: " <<
        // camera.second.m_topic_depth_image_raw << std::endl;

        auto p_depth_subscription =
            this->create_subscription<sensor_msgs::msg::Image>(
                camera.second.m_topic_depth_image_raw, 10, DepthCallback);
        m_depth_image_subs[camera.second.m_id] = p_depth_subscription;
      }
    }
  }

  return true;
}

void RobotVideoServer::destroyVideoSource() {
  if (m_b_use_sdk_) {
    for (auto &capture : m_captures_) {
      capture.second->stop();
    }

    m_captures_.clear();
  }
}

bool RobotVideoServer::setupVideoEncoder() {
  for (auto &camera : m_camera_config_) {
    if (!camera.second.m_b_enable) {
      continue;
    }

    EncCreateParam_t create_param;

    create_param.m_raw_pixfmt =
        V4L2_PIX_FMT_YUV420M;  //: m_b_use_sdk_ ? V4L2_PIX_FMT_NV12 :
                               //: V4L2_PIX_FMT_YUV420M;
    create_param.m_width = camera.second.m_color_image.m_width;
    create_param.m_height = camera.second.m_color_image.m_height;
    create_param.m_encoder_pixfmt = 0;  // 0-H264, 1-H265
    create_param.m_encode_width =
        camera.second.m_enc_image.m_width > camera.second.m_color_image.m_width
            ? camera.second.m_color_image.m_width
            : camera.second.m_enc_image.m_width;
    create_param.m_encode_height = camera.second.m_enc_image.m_height >
                                           camera.second.m_color_image.m_height
                                       ? camera.second.m_color_image.m_height
                                       : camera.second.m_enc_image.m_height;
    create_param.m_fps =
        camera.second.m_enc_image.m_fps > camera.second.m_color_image.m_fps
            ? camera.second.m_color_image.m_fps
            : camera.second.m_enc_image.m_fps;
    create_param.m_bitrate = camera.second.m_enc_image.m_bitrate * 1024;
    create_param.m_ratecontrol = 1;  // 0-VBR, 1-CBR
    create_param.m_rtsp_chn_id = camera.second.m_rtsp_chn;

    auto p_encoder =
        std::make_shared<VideoEncoder>(camera.second.m_id, create_param);
    if (p_encoder->start()) {
      m_encoders_[camera.second.m_id] = p_encoder;
    } else {
      YLLOG_ERR("Start encoder %d failed!", camera.second.m_id);
    }

    // 编码深度流
    if (camera.second.m_b_enable_depth) {
      std::string compressed_topic =
          camera.second.m_topic_depth_image_raw + "/compressed";
      m_depth_writers[camera.second.m_id] =
          m_dds_wrapper->createDataWriter(compressed_topic);

      auto p_depth_encoder = std::make_shared<DepthImageEncoder>(
          camera.second.m_id, m_depth_writers[camera.second.m_id]);
      if (p_depth_encoder->start()) {
        m_depth_encoders_[camera.second.m_id] = p_depth_encoder;
      }
    }
  }

  return true;
}

void RobotVideoServer::destroyVideoEncoder() {
  for (auto &encoder : m_encoders_) {
    encoder.second->stop();
  }
  m_encoders_.clear();

  for (auto &depthEncoder : m_depth_encoders_) {
    depthEncoder.second->stop();
  }
  m_depth_encoders_.clear();
}

bool RobotVideoServer::setupRtspServer() {
  RtspServerWrapper::getInstance()->init(8554, 3);

  return true;
}

void RobotVideoServer::destroyRtstpServer() {
  RtspServerWrapper::getInstance()->deinit();
}

bool RobotVideoServer::init() {
  if(!initLogger()) {
    return false;
  }

  if (!loadCameraConfig(m_camera_config_path_)) {
    return false;
  }

  setupRtspServer();
  setupVideoEncoder();
  setupVideoSource();

  return true;
}

void RobotVideoServer::deinit() {
  destroyVideoSource();
  destroyVideoEncoder();
  destroyRtstpServer();
}

bool RobotVideoServer::loadCameraConfig(const std::string &config_file_path) {
  std::ifstream file(config_file_path);
  if (!file.is_open()) {
    YLLOG_ERR("Failed to open JSON file: %s", config_file_path.c_str());
    return false;
  }

  nlohmann::json jsonData;
  file >> jsonData;

  m_b_use_sdk_ = jsonData["use_sdk"].get<bool>();

  auto cameras = jsonData["cameras"];
  for (auto &camera : cameras) {
    CameraConfig_t camConfig;

    camConfig.m_id = camera["id"].get<int32_t>();
    camConfig.m_rtsp_chn = camera["rtsp_chn"].get<int32_t>();
    camConfig.m_b_enable = camera["enable"].get<bool>();
    camConfig.m_b_enable_depth = camera["enable_depth"].get<bool>();
    camConfig.m_name_ = camera["name"].get<std::string>();
    camConfig.m_serial_num = camera["serial_num"].get<std::string>();
    camConfig.m_topic_color_image_raw =
        camera["sub_topic_color_image_raw"].get<std::string>();
    camConfig.m_topic_depth_image_raw =
        camera["sub_topic_depth_image_raw"].get<std::string>();

    camConfig.m_color_image.m_width =
        camera["color_image"]["width"].get<uint32_t>();
    camConfig.m_color_image.m_height =
        camera["color_image"]["height"].get<uint32_t>();
    camConfig.m_color_image.m_fps =
        camera["color_image"]["fps"].get<uint32_t>();
    camConfig.m_color_image.m_fmt =
        camera["color_image"]["format"].get<int32_t>();

    camConfig.m_depth_image.m_width =
        camera["depth_image"]["width"].get<uint32_t>();
    camConfig.m_depth_image.m_height =
        camera["depth_image"]["height"].get<uint32_t>();
    camConfig.m_depth_image.m_fps =
        camera["depth_image"]["fps"].get<uint32_t>();
    camConfig.m_depth_image.m_fmt =
        camera["depth_image"]["format"].get<int32_t>();

    camConfig.m_enc_image.m_width =
        camera["enc_image"]["width"].get<uint32_t>();
    camConfig.m_enc_image.m_height =
        camera["enc_image"]["height"].get<uint32_t>();
    camConfig.m_enc_image.m_fps = camera["enc_image"]["fps"].get<uint32_t>();
    camConfig.m_enc_image.m_fmt = camera["enc_image"]["format"].get<int32_t>();
    camConfig.m_enc_image.m_bitrate =
        camera["enc_image"]["bitrate"].get<uint32_t>();

    m_camera_config_[camConfig.m_id] = camConfig;
  }

  YLLOG_DBG("use_sdk: %s",  m_b_use_sdk_ ? "true" : "false");

  for (const auto &pair : m_camera_config_) {
    const auto &camConfig = pair.second;

    YLLOG_DBG("Camera Serial: %s", camConfig.m_serial_num.c_str());
    YLLOG_DBG("ID: %d", camConfig.m_id);
    YLLOG_DBG("Rtsp Chn: %d", camConfig.m_rtsp_chn);
    YLLOG_DBG("Name: %s", camConfig.m_name_.c_str());
    YLLOG_DBG("Color Image: %dx%d @ %d FPS, Format: %d",
              camConfig.m_color_image.m_width, camConfig.m_color_image.m_height,
              camConfig.m_color_image.m_fps, camConfig.m_color_image.m_fmt);
    YLLOG_DBG("Depth Image: %dx%d @ %d FPS, Format: %d",
              camConfig.m_depth_image.m_width, camConfig.m_depth_image.m_height,
              camConfig.m_depth_image.m_fps, camConfig.m_depth_image.m_fmt);
    YLLOG_DBG("Encoded Image: %dx%d @ %d FPS, Format: %d bitrate: %d",
              camConfig.m_enc_image.m_width, camConfig.m_enc_image.m_height,
              camConfig.m_enc_image.m_fps, camConfig.m_enc_image.m_fmt,
              camConfig.m_enc_image.m_bitrate);
    YLLOG_DBG("Color Image Raw Topic: %s",
              camConfig.m_topic_color_image_raw.c_str());
    YLLOG_DBG("Depth Image Raw Topic: %s",
              camConfig.m_topic_depth_image_raw.c_str());
    YLLOG_DBG("chn enable: %d, depth enable: %d", camConfig.m_b_enable,
              camConfig.m_b_enable_depth);
    YLLOG_DBG("--------------------------------------------------");
  }

  m_chn_offset_ = m_camera_config_.size();

  return true;
}

bool RobotVideoServer::loadLoggerConfig(const std::string &config_file_path) {
  std::ifstream file(config_file_path);
  if (!file.is_open()) {
    std::cerr << "Failed to open JSON file: " << config_file_path << std::endl;
    return false;
  }

  nlohmann::json jsonData;
  file >> jsonData;

  nlohmann::json logConfig = jsonData["log"];

  nlohmann::json consoleConfig = logConfig["console"];
  bool console_enalbe = consoleConfig["enable"].get<bool>();
  if(console_enalbe) {
    m_logger_ |= LOGGER_CONS;
  }
  m_log_param_.m_stConParam.m_iLevel = consoleConfig["level"].get<int32_t>();
  m_log_param_.m_stConParam.m_iPattern = LOG_PATTERN_SIMPLE;
  strcpy(m_log_param_.m_stConParam.m_acLoggerName, "consolelogger");

  nlohmann::json fileConfig = logConfig["file"];
  bool file_enable = fileConfig["enable"].get<bool>();
  if(file_enable) {
    m_logger_ |= LOGGER_FILE;
  }
  m_log_param_.m_stFileParam.m_iRollType = fileConfig["rollType"].get<int32_t>();
  m_log_param_.m_stFileParam.m_unParam.m_stRollIndexParam.m_iLevel = fileConfig["level"].get<int32_t>();
  m_log_param_.m_stFileParam.m_unParam.m_stRollIndexParam.m_bImmediateFlush = fileConfig["immediaFlush"].get<bool>();
  m_log_param_.m_stFileParam.m_unParam.m_stRollIndexParam.m_iBufferSize = fileConfig["buffSize"].get<int32_t>();
  std::string file_path = fileConfig["filePath"].get<std::string>();
  if(!file_path.empty()) {
    strcpy(m_log_param_.m_stFileParam.m_unParam.m_stRollIndexParam.m_acFilePath, fileConfig["filePath"].get<std::string>().c_str());
  }
  std::string file_name = fileConfig["fileName"].get<std::string>();
  if(!file_name.empty()) {
    strcpy(m_log_param_.m_stFileParam.m_unParam.m_stRollIndexParam.m_acFileName, fileConfig["fileName"].get<std::string>().c_str());
  }
  m_log_param_.m_stFileParam.m_unParam.m_stRollIndexParam.m_iMaxBackupIndex = fileConfig["MaxBackupIndex"].get<int32_t>();
  m_log_param_.m_stFileParam.m_unParam.m_stRollIndexParam.m_iMaxFileSize = fileConfig["MaxFileSize"].get<int32_t>();
  m_log_param_.m_stFileParam.m_unParam.m_stRollIndexParam.m_iPattern = LOG_PATTERN_SIMPLE;
  strcpy(m_log_param_.m_stFileParam.m_unParam.m_stRollIndexParam.m_acLoggerName, "filelogger");

  #if 0
  // 打印日志配置
  std::cout << "Logger Config:" << std::endl;
  std::cout << "Console Enable: " << console_enalbe << std::endl;
  std::cout << "Console Level: " << m_log_param_.m_stConParam.m_iLevel << std::endl;
  std::cout << "File Enable: " << file_enable << std::endl;
  std::cout << "File Roll Type: " << m_log_param_.m_stFileParam.m_iRollType << std::endl;
  std::cout << "File Level: " << m_log_param_.m_stFileParam.m_unParam.m_stRollIndexParam.m_iLevel << std::endl;
  std::cout << "File Immediate Flush: " << m_log_param_.m_stFileParam.m_unParam.m_stRollIndexParam.m_bImmediateFlush << std::endl;
  std::cout << "File Buffer Size: " << m_log_param_.m_stFileParam.m_unParam.m_stRollIndexParam.m_iBufferSize << std::endl;
  std::cout << "File Path: " << m_log_param_.m_stFileParam.m_unParam.m_stRollIndexParam.m_acFilePath << std::endl;
  std::cout << "File Name: " << m_log_param_.m_stFileParam.m_unParam.m_stRollIndexParam.m_acFileName << std::endl;
  std::cout << "File Max Backup Index: " << m_log_param_.m_stFileParam.m_unParam.m_stRollIndexParam.m_iMaxBackupIndex << std::endl;
  std::cout << "File Max File Size: " << m_log_param_.m_stFileParam.m_unParam.m_stRollIndexParam.m_iMaxFileSize << std::endl;
  std::cout << "File Pattern: " << m_log_param_.m_stFileParam.m_unParam.m_stRollIndexParam.m_iPattern << std::endl;
  std::cout << "File Logger Name: " << m_log_param_.m_stFileParam.m_unParam.m_stRollIndexParam.m_acLoggerName << std::endl;
  #endif

  return true;
}

bool RobotVideoServer::initLogger(){
  YLLOG_GetDefaultParam(&m_log_param_);
  if (!loadLoggerConfig(m_logger_config_path_)) {
    return false;
  }
  YLLOG_Init(m_logger_, &m_log_param_);

  return true;
}