/*
 * @Description: Software Video Encoder using FFmpeg
 * @Version: V1.0
 * @Author: software_encoder
 * @Date: 2025-07-12
 * Copyright (C) 2024-2050 Corenetic Technology Inc All rights reserved.
 */
#pragma once

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

#include "common_def.h"
#include "data_queue.hpp"
#include "rtsp_server_wrapper.h"
#include <atomic>
#include <memory>
#include <thread>

enum class InputPixelFormat {
    RGB24 = 0,    // RGB 24位
    GRAY8 = 1,    // Y8/灰度 8位
    YUV420P = 2   // YUV420P
};

typedef struct {
    uint32_t m_width = 640;
    uint32_t m_height = 480;
    uint32_t m_fps = 30;
    uint32_t m_bitrate = 1000000;  // 1Mbps
    InputPixelFormat m_input_format = InputPixelFormat::RGB24;
    
    // 编码器设置
    std::string m_codec_name = "libx264";  // 可选: libx264, libx265
    std::string m_preset = "ultrafast";    // 编码预设
    std::string m_tune = "zerolatency";    // 调优选项
    int32_t m_gop_size = 30;              // I帧间隔
    int32_t m_max_b_frames = 0;           // B帧数量
    
    // RTSP相关
    int32_t m_rtsp_chn_id = -1;
} SoftwareEncCreateParam_t;

class SoftwareVideoEncoder {
public:
    SoftwareVideoEncoder(uint32_t chn_id, const SoftwareEncCreateParam_t &enc_param);
    ~SoftwareVideoEncoder();

    bool start();
    void stop();
    bool putImage(Image_t *image_ptr);

private:
    bool initEncoder();
    void deinitEncoder();
    void encodeThread();
    bool encodeFrame(Image_t *image_ptr);
    bool convertInputFrame(Image_t *image_ptr, AVFrame *frame);
    bool convertRGB24ToYUV420P(const uint8_t *rgb_data, AVFrame *frame);
    bool convertGRAY8ToYUV420P(const uint8_t *gray_data, AVFrame *frame);
    bool sendEncodedPacket(AVPacket *packet, uint64_t timestamp);
    void fps();

private:
    uint32_t m_chn_id_;
    SoftwareEncCreateParam_t m_params_;
    
    // FFmpeg 相关
    AVCodec *m_codec_;
    AVCodecContext *m_codec_ctx_;
    AVFrame *m_frame_;
    AVPacket *m_packet_;
    SwsContext *m_sws_ctx_;
    
    // 线程和队列
    std::unique_ptr<DataQueue<Image_t *>> m_image_queue_;
    std::atomic_bool m_b_running_{false};
    std::shared_ptr<std::thread> m_encode_thread_;
    
    // 统计信息
    uint64_t m_last_time_ = 0;
    uint32_t m_count_ = 0;
    uint64_t m_frame_index_ = 0;
    
    std::atomic_bool m_b_init_{false};
};