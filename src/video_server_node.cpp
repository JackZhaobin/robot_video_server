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


bool RobotVideoServer::setupSDKMultiStream()
{
   
  return true;
}

/*
 *
 */
bool RobotVideoServer::setupROS2MultiStream() {
  for (auto &camera : m_camera_config_) {
    /*
     * 视频编码处理
     */
    if (!camera.second.m_b_enable_stream) {
      continue;
    }

    std::map<std::string, rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> camera_subs;

    // 为每个视频流创建订阅
    for (auto &stream : camera.second.m_video_streams) {
      if (!stream.m_b_enable) {
        continue;
      }

      // 直接使用配置中的topic
      std::string topic_name = stream.m_topic;

      auto ImageCallback = [this, camera_id = camera.second.m_id, 
                           stream_name = stream.m_name, 
                           stream_format = stream.m_fmt](
                               const sensor_msgs::msg::Image::SharedPtr msg) {
        
        auto size = msg->width * msg->height * 3 / 2;  // YUV420M size
        
        Image_t *image_ptr = (Image_t *)malloc(sizeof(Image_t) + size);
        image_ptr->m_width = msg->width;
        image_ptr->m_height = msg->height;
        image_ptr->m_pixfmt = 0;
        image_ptr->m_length = size;
        
        uint64_t sec = msg->header.stamp.sec;
        uint64_t nsec = msg->header.stamp.nanosec;
        image_ptr->m_timestamp = sec * 1000000ULL + nsec / 1000ULL;

        // 根据流格式进行相应转换
        switch(stream_format) {
          case 0:  // RGB
            convertRGB2YUV420M(msg->data.data(), msg->width, msg->height,
                              image_ptr->m_data);
            break;
          case 1:  // Y8/IR
            convertY8ToYUV420M(msg->data.data(), msg->width, msg->height,
                              image_ptr->m_data);
            break;
          default:
            YLLOG_ERR("Unsupported format: %d", stream_format);
            free(image_ptr);
            return;
        }

        // 发送到对应的编码器
        auto camera_encoders = m_stream_encoders_.find(camera_id);
        if (camera_encoders != m_stream_encoders_.end()) {
          auto encoder = camera_encoders->second.find(stream_name);
          if (encoder != camera_encoders->second.end()) {
            encoder->second->putImage(image_ptr);
          }
        }
      };

      auto subscription = this->create_subscription<sensor_msgs::msg::Image>(topic_name, 10, ImageCallback);
      camera_subs[stream.m_name] = subscription;
        
      YLLOG_INFO("Created subscription for camera %d, stream %s, topic %s", 
                camera.second.m_id, stream.m_name.c_str(), topic_name.c_str());
    }

    m_image_subs_[camera.second.m_id] = camera_subs;


    /*
    * 修改：压缩流处理 - 使用独立的编码器
    */
    if (camera.second.m_b_enable_compress) {
      std::map<std::string, rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> compress_subs;

      for (auto &compress_stream : camera.second.m_compress_streams) {
        if (!compress_stream.m_b_enable) {
          continue;
        }

        auto CompressCallback = [this, camera_id = camera.second.m_id, 
                              compress_name = compress_stream.m_name](
                                  const sensor_msgs::msg::Image::SharedPtr msg) {
          
          uint32_t size = msg->data.size();
          Image_t *image_ptr = (Image_t *)malloc(sizeof(Image_t) + size);
          
          // 使用独立的帧ID计数器
          auto camera_frame_ids = m_compress_frame_ids_.find(camera_id);
          if (camera_frame_ids != m_compress_frame_ids_.end()) {
            auto frame_id_it = camera_frame_ids->second.find(compress_name);
            if (frame_id_it != camera_frame_ids->second.end()) {
              image_ptr->m_id = ++frame_id_it->second;
            } else {
              image_ptr->m_id = 1;
            }
          } else {
            image_ptr->m_id = 1;
          }
          
          image_ptr->m_width = msg->width;
          image_ptr->m_height = msg->height;
          image_ptr->m_pixfmt = 0;
          image_ptr->m_length = size;
          
          uint64_t sec = msg->header.stamp.sec;
          uint64_t nsec = msg->header.stamp.nanosec;
          image_ptr->m_timestamp = sec * 1000000ULL + nsec / 1000ULL;

          YLLOG_DBG("Compress %s: timestamp %lu, current time %lu, diff %u us.", 
                  compress_name.c_str(), image_ptr->m_timestamp, getCurrentTimeUs(), 
                  getCurrentTimeUs()-image_ptr->m_timestamp);

          memcpy(image_ptr->m_data, msg->data.data(), msg->data.size());

          // 找到对应的独立编码器
          auto camera_encoders = m_compress_encoders_.find(camera_id);
          if (camera_encoders != m_compress_encoders_.end()) {
            auto encoder_it = camera_encoders->second.find(compress_name);
            if (encoder_it != camera_encoders->second.end()) {
              encoder_it->second->putImage(image_ptr);
            } else {
              YLLOG_ERR("Compress encoder not found for camera %d, stream %s", 
                        camera_id, compress_name.c_str());
              free(image_ptr);
            }
          } else {
            YLLOG_ERR("Camera compress encoders not found for camera %d", camera_id);
            free(image_ptr);
          }
        };

        auto p_compress_subscription = this->create_subscription<sensor_msgs::msg::Image>(
            compress_stream.m_topic, 10, CompressCallback);
        
        // 保存压缩流订阅器
        compress_subs[compress_stream.m_name] = p_compress_subscription;
        
        YLLOG_INFO("Created compress subscription for camera %d, stream %s, topic %s", 
                  camera.second.m_id, compress_stream.m_name.c_str(), compress_stream.m_topic.c_str());
      }

      // 保存该相机的所有压缩流订阅器
      m_compress_subs_[camera.second.m_id] = compress_subs;
    }
  }
  
  return true;
}


bool RobotVideoServer::setupVideoSource() {
    if (m_b_use_sdk_) {
        // SDK模式需要扩展VideoCapture支持多流  不实现
        //return setupSDKMultiStream();

        return setupROS2MultiStream();
    } else {
        // ROS2模式：为每个流创建订阅
        return setupROS2MultiStream();
    }
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
    if (!camera.second.m_b_enable_stream) {
      continue;
    }

    // 为每个相机的每个视频流创建编码器
    std::map<std::string, std::shared_ptr<VideoEncoder>> camera_encoders;
    int stream_index = 0;
        
    for (auto &stream : camera.second.m_video_streams) {
      if (!stream.m_b_enable) {
        stream_index++;
        continue;
      }
      
      EncCreateParam_t create_param;
      create_param.m_raw_pixfmt = V4L2_PIX_FMT_YUV420M;
      create_param.m_width = stream.m_width;
      create_param.m_height = stream.m_height;
      create_param.m_encoder_pixfmt = 0;  // H264
      create_param.m_encode_width = stream.m_width;
      create_param.m_encode_height = stream.m_height;
      create_param.m_fps = stream.m_fps;
      create_param.m_bitrate = stream.m_bitrate * 1024;
      create_param.m_ratecontrol = 1;  // CBR
      create_param.m_rtsp_chn_id = stream.m_rtsp_chn;

      uint32_t encoder_id = camera.second.m_id * 100 + stream_index;
      auto p_encoder = std::make_shared<VideoEncoder>(encoder_id, create_param);
      
      if (p_encoder->start()) {
        camera_encoders[stream.m_name] = p_encoder;
        YLLOG_INFO("Created encoder for camera %d, stream %s, RTSP channel %d", 
                  camera.second.m_id, stream.m_name.c_str(), stream.m_rtsp_chn);
      } else {
        YLLOG_ERR("Failed to start encoder for camera %d, stream %s", 
                  camera.second.m_id, stream.m_name.c_str());
      }
      stream_index++;
    }
    m_stream_encoders_[camera.second.m_id] = camera_encoders;

    /*
     * 修改：为每个压缩流创建独立的编码器
     */
    if (camera.second.m_b_enable_compress) {
      std::map<std::string, std::shared_ptr<DepthImageEncoder>> camera_compress_encoders;
      std::map<std::string, CDataWriter*> camera_compress_writers;
      std::map<std::string, uint32_t> camera_compress_frame_ids;


      int compress_stream_index = 0;
      for (auto &compress_stream : camera.second.m_compress_streams) {
        if (!compress_stream.m_b_enable) {
          compress_stream_index++;
          continue;
        }

        // 为每个压缩流创建独立的topic
        std::string compressed_topic = compress_stream.m_topic + "/compressed";
        //std::string compressed_topic = compress_stream.m_topic + "/zdepth";
        
        // 为每个压缩流创建独立的数据写入器
        camera_compress_writers[compress_stream.m_name] = m_dds_wrapper->createDataWriter(compressed_topic);
       
       /*
        if (!camera_compress_writers[compress_stream.m_name]) {
          YLLOG_ERR("Failed to create data writer for camera %d, compress stream %s", 
                    camera.second.m_id, compress_stream.m_name.c_str());
          continue;
        }*/
        
        // 创建唯一的编码器ID：camera_id * 1000 + compress_stream_index
        int32_t compress_encoder_id = camera.second.m_id * 1000 + compress_stream_index;
  
        
        // 为每个压缩流创建独立的编码器
        auto p_compress_encoder = std::make_shared<DepthImageEncoder>(compress_encoder_id, camera_compress_writers[compress_stream.m_name] );
        
        if (p_compress_encoder->start()) {
          // 保存到对应的映射中
          camera_compress_encoders[compress_stream.m_name] = p_compress_encoder;
         // camera_compress_writers[compress_stream.m_name] = writer;
          camera_compress_frame_ids[compress_stream.m_name] = 0;  // 初始化帧ID
          
          YLLOG_INFO("Created compress encoder for camera %d, stream %s, topic %s", 
                    camera.second.m_id, compress_stream.m_name.c_str(), compressed_topic.c_str());
        } else {
          YLLOG_ERR("Failed to start compress encoder for camera %d, stream %s", 
                    camera.second.m_id, compress_stream.m_name.c_str());
          // 清理失败的writer
          // 注意：这里需要根据你的DDS库提供相应的清理函数
          // m_dds_wrapper->destroyDataWriter(writer);
        }
        compress_stream_index++;
      }
      
      // 保存该相机的所有压缩编码器和相关资源
      m_compress_encoders_[camera.second.m_id] = camera_compress_encoders;
      m_compress_writers_[camera.second.m_id] = camera_compress_writers;
      m_compress_frame_ids_[camera.second.m_id] = camera_compress_frame_ids;
    }
  }

  return true;
}

void RobotVideoServer::destroyVideoStreamEncoders() {
  for (auto &camera_encoders : m_stream_encoders_) {
    int32_t camera_id = camera_encoders.first;
    
    YLLOG_DBG("Destroying encoders for camera %d", camera_id);
    
    for (auto &stream_encoder : camera_encoders.second) {
      const std::string &stream_name = stream_encoder.first;
      auto &encoder = stream_encoder.second;
      
      if (encoder) {
        YLLOG_DBG("Stopping encoder for camera %d, stream %s", 
                  camera_id, stream_name.c_str());
        
        try {
          encoder->stop();
          YLLOG_DBG("Successfully stopped encoder for camera %d, stream %s", 
                    camera_id, stream_name.c_str());
        } catch (const std::exception& e) {
          YLLOG_ERR("Exception while stopping encoder for camera %d, stream %s: %s", 
                    camera_id, stream_name.c_str(), e.what());
        }
      } else {
        YLLOG_WARN("Null encoder found for camera %d, stream %s", 
                   camera_id, stream_name.c_str());
      }
    }
    
    // 清空该相机的所有流编码器
    camera_encoders.second.clear();
  }
  
  // 清空整个编码器映射
  m_stream_encoders_.clear();
  
  YLLOG_INFO("All video stream encoders destroyed");
}


#if 0
void RobotVideoServer::destroyVideoEncoder() {

  destroyVideoStreamEncoders(); 

  for (auto &depthEncoder : m_depth_encoders_) {
    depthEncoder.second->stop();
  }
  m_depth_encoders_.clear();
}
#endif

void RobotVideoServer::destroyVideoEncoder() {
  // 销毁视频流编码器
  destroyVideoStreamEncoders(); 

  // 销毁压缩流编码器
  for (auto &camera_compress_encoders : m_compress_encoders_) {
    int32_t camera_id = camera_compress_encoders.first;
    
    YLLOG_DBG("Destroying compress encoders for camera %d", camera_id);
    
    for (auto &compress_encoder : camera_compress_encoders.second) {
      const std::string &compress_name = compress_encoder.first;
      auto &encoder = compress_encoder.second;
      
      if (encoder) {
        YLLOG_DBG("Stopping compress encoder for camera %d, stream %s", 
                  camera_id, compress_name.c_str());
        
        try {
          encoder->stop();
          YLLOG_DBG("Successfully stopped compress encoder for camera %d, stream %s", 
                    camera_id, compress_name.c_str());
        } catch (const std::exception& e) {
          YLLOG_ERR("Exception while stopping compress encoder for camera %d, stream %s: %s", 
                    camera_id, compress_name.c_str(), e.what());
        }
      } else {
        YLLOG_WARN("Null compress encoder found for camera %d, stream %s", 
                   camera_id, compress_name.c_str());
      }
    }
    
    // 清空该相机的所有压缩编码器
    camera_compress_encoders.second.clear();
  }
  
  // 清空整个压缩编码器映射
  m_compress_encoders_.clear();
  
  // 清理压缩流数据写入器
  for (auto &camera_writers : m_compress_writers_) {
    int32_t camera_id = camera_writers.first;
    
    for (auto &writer_pair : camera_writers.second) {
      const std::string &compress_name = writer_pair.first;
      CDataWriter* writer = writer_pair.second;
      
      if (writer) {
        YLLOG_DBG("Destroying compress writer for camera %d, stream %s", 
                  camera_id, compress_name.c_str());
        // 根据你的DDS库提供相应的清理函数
        // m_dds_wrapper->destroyDataWriter(writer);
      }
    }
    
    camera_writers.second.clear();
  }
  
  m_compress_writers_.clear();
  m_compress_frame_ids_.clear();
  
  YLLOG_INFO("All compress encoders destroyed");
}

bool RobotVideoServer::setupRtspServer() {
  //RtspServerWrapper::getInstance()->init(8554, 3);

  uint32_t total_channels = 0;
  for (auto &camera : m_camera_config_) {
      for (auto &stream : camera.second.m_video_streams) {
          if (stream.m_b_enable) {
              total_channels = std::max(total_channels, stream.m_rtsp_chn + 1);
          }
      }
  }

  RtspServerWrapper::getInstance()->init(8554, total_channels);
  
  YLLOG_INFO("RTSP server initialized with %d channels", total_channels);
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

/*
 *
 */
bool RobotVideoServer::loadCameraConfig(const std::string &config_file_path) {
  YLLOG_INFO("=== Starting to load camera configuration ===");
  YLLOG_INFO("Config file path: %s", config_file_path.c_str());

  std::ifstream file(config_file_path);
  if (!file.is_open()) {
    YLLOG_ERR("Failed to open JSON file: %s", config_file_path.c_str());
    return false;
  }

  YLLOG_INFO("Successfully opened config file");

  nlohmann::json jsonData;
  try {
    file >> jsonData;
    YLLOG_INFO("Successfully parsed JSON data");
  } catch (const std::exception& e) {
    YLLOG_ERR("Failed to parse JSON: %s", e.what());
    return false;
  }

  // 解析use_sdk配置
  try {
    m_b_use_sdk_ = jsonData["use_sdk"].get<bool>();
    YLLOG_INFO("use_sdk: %s", m_b_use_sdk_ ? "true" : "false");
  } catch (const std::exception& e) {
    YLLOG_ERR("Failed to parse use_sdk: %s", e.what());
    return false;
  }

  auto cameras = jsonData["cameras"];
  YLLOG_INFO("Found %zu cameras in configuration", cameras.size());

  int camera_count = 0;
  for (auto &camera : cameras) {
    camera_count++;
    YLLOG_INFO("--- Processing camera %d ---", camera_count);

    CameraConfig_t camConfig;

    try {
      // 解析基本相机配置
      camConfig.m_id = camera["id"].get<int32_t>();
      camConfig.m_name_ = camera["name"].get<std::string>();
      camConfig.m_serial_num = camera["serial_num"].get<std::string>();
      camConfig.m_b_enable_compress = camera["enable_compress"].get<bool>();  // 修改字段名
      camConfig.m_b_enable_stream = camera["enable_stream"].get<bool>();
      
      YLLOG_INFO("Camera ID: %d", camConfig.m_id);
      YLLOG_INFO("Camera name: %s", camConfig.m_name_.c_str());
      YLLOG_INFO("Serial number: %s", camConfig.m_serial_num.c_str());
      YLLOG_INFO("Enable compress: %s", camConfig.m_b_enable_compress ? "true" : "false");
      YLLOG_INFO("Enable stream: %s", camConfig.m_b_enable_stream ? "true" : "false");
      
    } catch (const std::exception& e) {
      YLLOG_ERR("Failed to parse basic camera config for camera %d: %s", camera_count, e.what());
      continue;
    }

    // 解析视频流配置（修改部分）
    try {
      auto video_images = camera["video_image"];
      YLLOG_INFO("Found %zu video streams for camera %d", video_images.size(), camConfig.m_id);
      
      int stream_count = 0;
      for (auto &video_img : video_images) {
        stream_count++;
        auto img = video_img["image"];
        
        ImageStreamConfig_t streamConfig;
        streamConfig.m_name = img["name"].get<std::string>();
        streamConfig.m_width = img["width"].get<uint32_t>();
        streamConfig.m_height = img["height"].get<uint32_t>();
        streamConfig.m_fps = img["fps"].get<uint32_t>();
        streamConfig.m_fmt = img["format"].get<int32_t>();
        streamConfig.m_bitrate = img["bitrate"].get<uint32_t>();
        streamConfig.m_rtsp_chn = img["rtsp_chn"].get<int32_t>();
        streamConfig.m_topic = img["topic"].get<std::string>();  // 新增topic解析
        streamConfig.m_b_enable = img["enable"].get<bool>();
        
        YLLOG_INFO("  Stream %d:", stream_count);
        YLLOG_INFO("    Name: %s", streamConfig.m_name.c_str());
        YLLOG_INFO("    Topic: %s", streamConfig.m_topic.c_str());  // 新增日志
        YLLOG_INFO("    Resolution: %ux%u", streamConfig.m_width, streamConfig.m_height);
        YLLOG_INFO("    FPS: %u", streamConfig.m_fps);
        YLLOG_INFO("    Format: %d", streamConfig.m_fmt);
        YLLOG_INFO("    Bitrate: %u", streamConfig.m_bitrate);
        YLLOG_INFO("    RTSP channel: %d", streamConfig.m_rtsp_chn);
        YLLOG_INFO("    Enabled: %s", streamConfig.m_b_enable ? "true" : "false");
        
        camConfig.m_video_streams.push_back(streamConfig);
      }
      
      YLLOG_INFO("Successfully parsed %zu video streams for camera %d", camConfig.m_video_streams.size(), camConfig.m_id);
        
    } catch (const std::exception& e) {
      YLLOG_ERR("Failed to parse video streams for camera %d: %s", camConfig.m_id, e.what());
      continue;
    }

    // 新增：解析压缩流配置
    try {
      auto deep_comp = camera["deep_comp"];
      YLLOG_INFO("Found %zu compress streams for camera %d", deep_comp.size(), camConfig.m_id);
      
      int compress_count = 0;
      for (auto &comp : deep_comp) {
        compress_count++;
        auto compress_cfg = comp["compress"];
        
        ImageCompressConfig_t compressConfig;
        compressConfig.m_name = compress_cfg["name"].get<std::string>();
        compressConfig.m_width = compress_cfg["width"].get<uint32_t>();
        compressConfig.m_height = compress_cfg["height"].get<uint32_t>();
        compressConfig.m_fps = compress_cfg["fps"].get<uint32_t>();
        compressConfig.m_fmt = compress_cfg["format"].get<int32_t>();
        compressConfig.m_topic = compress_cfg["topic"].get<std::string>();
        compressConfig.m_b_enable = compress_cfg["enable"].get<bool>();
        
        YLLOG_INFO("  Compress %d:", compress_count);
        YLLOG_INFO("    Name: %s", compressConfig.m_name.c_str());
        YLLOG_INFO("    Topic: %s", compressConfig.m_topic.c_str());
        YLLOG_INFO("    Resolution: %ux%u", compressConfig.m_width, compressConfig.m_height);
        YLLOG_INFO("    FPS: %u", compressConfig.m_fps);
        YLLOG_INFO("    Format: %d", compressConfig.m_fmt);
        YLLOG_INFO("    Enabled: %s", compressConfig.m_b_enable ? "true" : "false");
        
        camConfig.m_compress_streams.push_back(compressConfig);
      }
      
      YLLOG_INFO("Successfully parsed %zu compress streams for camera %d", camConfig.m_compress_streams.size(), camConfig.m_id);
        
    } catch (const std::exception& e) {
      YLLOG_ERR("Failed to parse compress streams for camera %d: %s", camConfig.m_id, e.what());
      // 压缩流解析失败不影响主流程，继续处理
    }

    m_camera_config_[camConfig.m_id] = camConfig;
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