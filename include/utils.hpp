/*
 * @Description:
 * @Version: V1.0
 * @Author: hongyuan.liu@corenetic.ai
 * @Date: 2025-03-15 10:40:04
 * @LastEditors: hongyuan.liu@corenetic.ai
 * @LastEditTime: 2025-04-22 17:18:22
 * Copyright (C) 2024-2050 Corenetic Technology Inc All rights reserved.
 */
#pragma once

#include <semaphore.h>
#include <stdint.h>
#include <sys/time.h>

#include <atomic>
#include <chrono>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

class TimePerf {
 public:
  TimePerf(const std::string& name = "Timer")
      : m_name(name), m_start(std::chrono::high_resolution_clock::now()) {}

  ~TimePerf() {
    auto end = std::chrono::high_resolution_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - m_start)
            .count();
    std::cout << m_name << " elapsed time: " << duration << " ms" << std::endl;
  }

 private:
  std::string m_name;
  std::chrono::time_point<std::chrono::high_resolution_clock> m_start;
};

void dumpYUV420M(const std::string& filename, uint8_t* yuv, int width,
                 int height);

bool dumpBufferToFile(const uint8_t* buffer, size_t size,
                      const std::string& filename);

void convertRGB2YUV420M(const uint8_t* rgb, int width, int height,
                        uint8_t* yuv);

void convertDepth2YUV420M(const uint16_t* depth_data, int32_t width,
                          int32_t height, int32_t bit_depth,
                          uint8_t* yuv_buffer);

bool encodeRGBToPNG(const uint8_t* rgbData, int width, int height,
                    uint8_t* outputBuffer, uint32_t& outputSize);

bool encodeDepthToPNG(const uint8_t* depthData, int width, int height,
                      uint8_t* outputBuffer, uint32_t& outputSize);

void displayRGBFrame(uint8_t* data, int width, int height);

uint64_t getStartupTimeMs();

uint64_t getCurrentTimeMs();

uint64_t getCurrentTimeUs();
