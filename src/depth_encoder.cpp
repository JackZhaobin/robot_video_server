/*
 * @Description:
 * @Version: V1.0
 * @Author: hongyuan.liu@corenetic.ai
 * @Date: 2025-03-24 06:59:19
 * @LastEditors: hongyuan.liu@corenetic.ai
 * @LastEditTime: 2025-04-21 11:33:46
 * Copyright (C) 2024-2050 Corenetic Technology Inc All rights reserved.
 */
#include "depth_encoder.h"
#include "yllog.h"
#include "utils.hpp"

DepthImageEncoder::DepthImageEncoder(int32_t chn_id, CDataWriter *pWriter)
    : m_chn_id_(chn_id), m_writer_(pWriter) {
  std::string tag = "[Depth chn " + std::to_string(m_chn_id_) + "]";

  m_image_queue_ = std::make_unique<DataQueue<Image_t *>>(
      8,
      [](Image_t *image) {
        if (image) free(image);
      },
      tag);
}

DepthImageEncoder::~DepthImageEncoder() { stop(); }

bool DepthImageEncoder::start() {
  if (!m_writer_) {
    return false;
  }

  m_bRunning_ = true;
  m_enc_thread_ =
      std::make_shared<std::thread>(&DepthImageEncoder::encodeThread, this);

  return true;
}

void DepthImageEncoder::stop() {
  if (m_bRunning_) {
    m_bRunning_ = false;
    if (m_enc_thread_->joinable()) {
      m_enc_thread_->join();
    }
  }
}

bool DepthImageEncoder::putImage(Image_t *image_ptr) {
  return m_image_queue_->enqueue(std::move(image_ptr));
}

bool DepthImageEncoder::encodeDepth2PNG(const uint8_t *depthData, int width,
                                        int height, uint8_t *outputBuffer,
                                        uint32_t &outputSize) {
  uint64_t start_time = getStartupTimeMs();

  if (!depthData || width <= 0 || height <= 0 || !outputBuffer) {
    return false;
  }

  // 创建 OpenCV 16-bit 灰度图
  cv::Mat depthImage(height, width, CV_16UC1, (void *)depthData);

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

#if 1
  // printf("src size: %u, dst size: %u, compresess rate: %.1f\n", width *
  // height * 2, outputSize, (double)outputSize / (width * height * 2));

  uint64_t end_time = getStartupTimeMs();
  test_frame_count_++;
  uint32_t diff = end_time - start_time;
  total_time_ += diff;

  min_diff_ = std::min(min_diff_, diff);
  max_diff_ = std::max(max_diff_, diff);

  if (test_frame_count_ >= 100) {
    YLLOG_DBG(
        "Depth chn %d: Average compression time: %.2f ms, min_diff: %u, "
        "max_diff: %u",
        m_chn_id_, (double)total_time_ / test_frame_count_, min_diff_,
        max_diff_);
        
    test_frame_count_ = 0;
    total_time_ = 0;
  }

#endif

  return true;
}

bool DepthImageEncoder::compressDepth(const uint16_t *depthData, int width,
                                      int height, bool isKeyframe,
                                      std::vector<uint8_t> &compressed) {
  if (!depthData || width <= 0 || height <= 0) {
    return false;
  }

  uint64_t start_time = getStartupTimeMs();

  m_compressor_.Compress(width, height, depthData, compressed, isKeyframe);

#if 1
  // printf("src size: %u, dst size: %lu, compresess rate: %.1f\n", width *
  // height * 2, compressed.size(),
  //       (double)compressed.size() / (width * height * 2));

  uint64_t end_time = getStartupTimeMs();
  test_frame_count_++;
  uint32_t diff = end_time - start_time;
  total_time_ += diff;

  min_diff_ = std::min(min_diff_, diff);
  max_diff_ = std::max(max_diff_, diff);

  if (test_frame_count_ >= 100) {
    YLLOG_DBG(
        "Depth chn %d: Average compression time: %.2f ms, min_diff: %u, "
        "max_diff: %u",
        m_chn_id_, (double)total_time_ / test_frame_count_, min_diff_,
        max_diff_);

    test_frame_count_ = 0;
    total_time_ = 0;
  }

#endif

  return true;
}

void DepthImageEncoder::fps() {
  m_count_++;

  uint64_t cur_time = getStartupTimeMs();
  if (cur_time - m_last_time_ >= 1000) {
    double fps = m_count_ * 1000.0 / (cur_time - m_last_time_);
    YLLOG_DBG("Depth Encoder chn %d: FPS %.1f", m_chn_id_, fps);

    m_count_ = 0;
    m_last_time_ = cur_time;
  }
}

bool DepthImageEncoder::doEncode(Image_t *image_ptr) {
  // TimePerf t("DepthImageEncoder::doEncode");

  DataBufferPtr bufPtr =
      m_writer_->getEmptyDataBuffer(sizeof(Image_t) + image_ptr->m_length);
  if (!bufPtr) {
    YLLOG_ERR("Get empty data buffer failed!");
    return false;
  }

  Image_t *compressedImg = reinterpret_cast<Image_t *>(bufPtr->m_pBufPtr);
  compressedImg->m_id = image_ptr->m_id;
  compressedImg->m_width = image_ptr->m_width;
  compressedImg->m_height = image_ptr->m_height;
  compressedImg->m_timestamp = image_ptr->m_timestamp;

  std::vector<uint8_t> compressed;
  compressDepth((uint16_t *)(image_ptr->m_data), image_ptr->m_width,
                image_ptr->m_height, true, compressed);
  memcpy(compressedImg->m_data, compressed.data(), compressed.size());
  compressedImg->m_length = compressed.size();

  bufPtr->m_iBufLen = sizeof(Image_t) + compressedImg->m_length;
  m_writer_->putFullDataBuffer(bufPtr);

  fps();

  return true;
}

void DepthImageEncoder::encodeThread() {
  while (m_bRunning_) {
    Image_t *pRawImg = nullptr;

    if (!m_image_queue_->dequeue(pRawImg, 100)) {
      continue;
    }

    doEncode(pRawImg);

    free(pRawImg);
  }
}