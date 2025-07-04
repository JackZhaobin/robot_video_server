/*
 * @Description: 
 * @Version: V1.0
 * @Author: hongyuan.liu@corenetic.ai
 * @Date: 2025-03-25 17:33:00
 * @LastEditors: hongyuan.liu@corenetic.ai
 * @LastEditTime: 2025-04-21 10:53:24
 * Copyright (C) 2024-2050 Corenetic Technology Inc All rights reserved.
 */
#pragma once

#include "common_def.h"
#include "data_queue.hpp"
#include "yhdds.h"
#include "zdepth.hpp"

using namespace zdepth;

class DepthImageEncoder 
{
public:
    DepthImageEncoder(int32_t chn_id, CDataWriter *pWriter);

    ~DepthImageEncoder();

    bool start();
    void stop();
    bool putImage(Image_t *image_ptr);
private:
    void fps();
    bool doEncode(Image_t *image_ptr);
    void encodeThread();
    bool encodeDepth2PNG(const uint8_t* depthData, int width, int height, uint8_t* outputBuffer, uint32_t& outputSize);
    bool compressDepth(const uint16_t* depthData, int width, int height, bool isKeyframe, std::vector<uint8_t>& compressed);

private:
    int32_t m_chn_id_ = -1;
    CDataWriter *m_writer_ = nullptr;

    uint32_t m_frame_cnt = 0;

    std::unique_ptr<DataQueue<Image_t *>> m_image_queue_;

    std::atomic_bool m_bRunning_{false};
    std::shared_ptr<std::thread> m_enc_thread_;

    uint32_t test_frame_count_ = 0;
    uint32_t total_time_ = 0;
    uint32_t min_diff_ = 0xFFFFFFFF;
    uint32_t max_diff_ = 0;

    DepthCompressor m_compressor_;

    uint64_t m_last_time_ = 0;
    uint32_t m_count_ = 0;
};