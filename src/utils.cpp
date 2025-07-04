/*
 * @Description:
 * @Version: V1.0
 * @Author: hongyuan.liu@corenetic.ai
 * @Date: 2025-03-21 01:26:33
 * @LastEditors: hongyuan.liu@corenetic.ai
 * @LastEditTime: 2025-04-22 17:15:44
 * Copyright (C) 2024-2050 Corenetic Technology Inc All rights reserved.
 */

#include "utils.hpp"

void dumpYUV420M(const std::string& filename, uint8_t* yuv, int32_t width,
                 int32_t height) {
  std::ofstream file(filename, std::ios::binary);
  if (file.is_open()) {
    file.write(reinterpret_cast<char*>(yuv), width * height * 3 / 2);
    file.close();
    std::cout << "YUV420M 数据已保存至：" << filename << std::endl;
  } else {
    std::cerr << "无法打开文件：" << filename << std::endl;
  }
}

bool dumpBufferToFile(const uint8_t* buffer, size_t size,
                      const std::string& filename) {
  if (!buffer || size == 0) {
    return false;
  }

  std::ofstream file(filename, std::ios::binary);
  if (!file) {
    return false;
  }

  file.write(reinterpret_cast<const char*>(buffer), size);
  return file.good();
}

void convertRGB2YUV420M(const uint8_t* rgb, int32_t width, int32_t height,
                        uint8_t* yuv) {
  // TimePerf t("RGB8_to_YUV420M");

  int32_t frameSize = width * height;
  int32_t uvIndex = frameSize;                   // U 分量的起始位置
  int32_t vIndex = frameSize + (frameSize / 4);  // V 分量的起始位置

  for (int32_t j = 0; j < height; j++) {
    for (int32_t i = 0; i < width; i++) {
      int32_t rgbIndex = (j * width + i) * 3;
      uint8_t R = rgb[rgbIndex];
      uint8_t G = rgb[rgbIndex + 1];
      uint8_t B = rgb[rgbIndex + 2];

      // 计算 Y 分量
      yuv[j * width + i] =
          static_cast<uint8_t>(0.299 * R + 0.587 * G + 0.114 * B);

      // 只对偶数行和偶数列采样 U 和 V
      if (j % 2 == 0 && i % 2 == 0) {
        int32_t avgIndex = (j / 2) * (width / 2) + (i / 2);
        uint8_t Y = yuv[j * width + i];

        // 计算 U 分量
        yuv[uvIndex + avgIndex] = static_cast<uint8_t>((B - Y) * 0.565 + 128);

        // 计算 V 分量
        yuv[vIndex + avgIndex] = static_cast<uint8_t>((R - Y) * 0.713 + 128);
      }
    }
  }
}

void convertDepth2YUV420M(const uint16_t* depth_data, int32_t width,
                          int32_t height, int32_t bit_depth,
                          uint8_t* yuv_buffer) {
  // 1. 确保位深度为16-bit
  if (bit_depth != 16) {
    std::cerr << "Error: Depth image must be 16-bit!" << std::endl;
    return;
  }

  // 2. 将 16-bit 深度数据转换为 8-bit（通过缩放）
  cv::Mat depth_image(
      height, width, CV_16U,
      const_cast<unsigned short*>(depth_data));  // 使用原始数据指针
  cv::Mat depth_8bit;
  depth_image.convertTo(depth_8bit, CV_8U,
                        255.0 / 65535.0);  // 将16-bit转换为8-bit

  // 3. 创建 YUV420M 图像的内存空间
  int32_t y_size = width * height;
  int32_t uv_size = (width / 2) * (height / 2);  // 下采样为 2x2

  unsigned char* y_data = yuv_buffer;
  unsigned char* u_data = yuv_buffer + y_size;
  unsigned char* v_data = u_data + uv_size;

  // 4. 填充 Y 分量 (深度图是灰度图，直接填充)
  memcpy(y_data, depth_8bit.data, y_size);

  // 5. 计算 U 和 V 分量 (此处可以设置为 0，因为是灰度图)
  memset(u_data, 0, uv_size);
  memset(v_data, 0, uv_size);

  // 不再写入文件，而是将数据保存在提供的缓冲区中
  std::cout << "YUV420M data stored in the provided buffer" << std::endl;
}

bool encodeRGBToPNG(const uint8_t* rgbData, int width, int height,
                    uint8_t* outputBuffer, uint32_t& outputSize) {
  if (!rgbData || width <= 0 || height <= 0 || !outputBuffer) {
    return false;
  }

  // 将 RGB 数据转换为 OpenCV Mat 格式
  cv::Mat image(height, width, CV_8UC3, (void*)rgbData);

  // 使用 OpenCV 编码为 PNG 格式
  std::vector<uint8_t> encodedData;
  std::vector<int> compressionParams = {cv::IMWRITE_PNG_COMPRESSION,
                                        2};  // 最大压缩率
  if (!cv::imencode(".png", image, encodedData, compressionParams)) {
    return false;
  }

  // 将编码后的 PNG 数据拷贝到用户提供的 buffer 中
  std::memcpy(outputBuffer, encodedData.data(), encodedData.size());
  outputSize = encodedData.size();

  return true;
}

bool encodeDepthToPNG(const uint8_t* depthData, int width, int height,
                      uint8_t* outputBuffer, uint32_t& outputSize) {
  // TimePerf t("encodeDepthToPNG");

  static uint32_t frame_count = 0;
  static uint32_t total_time = 0;
  static uint32_t min_diff = 0;
  static uint32_t max_diff = 0;
  uint64_t start_time = getStartupTimeMs();

  if (!depthData || width <= 0 || height <= 0 || !outputBuffer) {
    return false;
  }

  // 创建 OpenCV 16-bit 灰度图
  cv::Mat depthImage(height, width, CV_16UC1, (void*)depthData);

  // PNG 编码参数（最大压缩率）
  std::vector<uint8_t> encodedData;
  std::vector<int> compressionParams = {cv::IMWRITE_PNG_COMPRESSION, 2};

  // 进行 PNG 编码
  if (!cv::imencode(".png", depthImage, encodedData, compressionParams)) {
    return false;
  }

  // 拷贝到用户 buffer
  std::memcpy(outputBuffer, encodedData.data(), encodedData.size());
  outputSize = encodedData.size();

  uint64_t end_time = getStartupTimeMs();

  frame_count++;
  uint32_t diff = end_time - start_time;
  total_time += diff;

  min_diff = std::min(min_diff, diff);
  max_diff = std::max(max_diff, diff);

  if (frame_count >= 100) {
    printf("Average compression time: %.2f ms, min_diff: %u, max_diff: %u\n",
           (double)total_time / frame_count, min_diff, max_diff);
    frame_count = 0;
    total_time = 0;
  }

  // printf("src size: %u, dst size: %u, compresess rate: %.1f\n", width *
  // height * 2, outputSize, (double)outputSize / (width * height * 2));

  return true;
}

void displayRGBFrame(uint8_t* data, int32_t width, int32_t height) {
  // 创建一个OpenCV的Mat对象
  cv::Mat rgbImage(height, width, CV_8UC3, data);

  // 转换为BGR格式（OpenCV默认的显示格式）
  cv::Mat bgrImage;
  cv::cvtColor(rgbImage, bgrImage, cv::COLOR_RGB2BGR);

  // 显示图像
  cv::imshow("Decoded Video Frame", bgrImage);
  cv::waitKey(1);  // 等待1毫秒
}

uint64_t getStartupTimeMs() {
  struct timespec ts;
  if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
    perror("clock_gettime");
    return 0;
  }
  return (uint64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

uint64_t getCurrentTimeMs() {
  struct timeval t;
  gettimeofday(&t, NULL);
  return (uint64_t)t.tv_sec * 1000 + t.tv_usec / 1000;
}

uint64_t getCurrentTimeUs() {
  struct timeval t;
  gettimeofday(&t, NULL);
  return (uint64_t)t.tv_sec * 1000 * 1000 + t.tv_usec;
}