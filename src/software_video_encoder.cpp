/*
 * @Description: Software Video Encoder Implementation
 * @Version: V1.0
 * @Author: software_encoder
 * @Date: 2025-07-12
 */
#include "software_video_encoder.h"
#include "utils.hpp"
#include "yllog.h"

SoftwareVideoEncoder::SoftwareVideoEncoder(uint32_t chn_id, const SoftwareEncCreateParam_t &enc_param)
    : m_chn_id_(chn_id), m_params_(enc_param) {
    
    std::string tag = "[SoftEnc chn " + std::to_string(m_chn_id_) + "]";
    
    m_image_queue_ = std::make_unique<DataQueue<Image_t *>>(
        8, [](Image_t *image_ptr) { free(image_ptr); }, tag);
    
    // 初始化FFmpeg (只需要调用一次)
    static bool ffmpeg_initialized = false;
    if (!ffmpeg_initialized) {
        av_log_set_level(AV_LOG_ERROR); // 设置日志级别
        ffmpeg_initialized = true;
    }
    
    m_codec_ = nullptr;
    m_codec_ctx_ = nullptr;
    m_frame_ = nullptr;
    m_packet_ = nullptr;
    m_sws_ctx_ = nullptr;
}

SoftwareVideoEncoder::~SoftwareVideoEncoder() {
    stop();
}

bool SoftwareVideoEncoder::start() {
    if (!initEncoder()) {
        YLLOG_ERR("chn %d: Failed to initialize software encoder", m_chn_id_);
        return false;
    }
    
    m_b_running_ = true;
    m_encode_thread_ = std::make_shared<std::thread>(&SoftwareVideoEncoder::encodeThread, this);
    
    m_b_init_ = true;
    YLLOG_INFO("chn %d: Software encoder started successfully", m_chn_id_);
    
    return true;
}

void SoftwareVideoEncoder::stop() {
    if (!m_b_init_) {
        return;
    }
    
    m_b_running_ = false;
    if (m_encode_thread_ && m_encode_thread_->joinable()) {
        m_encode_thread_->join();
    }
    
    deinitEncoder();
    m_b_init_ = false;
    
    YLLOG_INFO("chn %d: Software encoder stopped", m_chn_id_);
}

bool SoftwareVideoEncoder::putImage(Image_t *image_ptr) {
    if (!m_b_running_) {
        return false;
    }
    
    return m_image_queue_->enqueue(std::move(image_ptr));
}

bool SoftwareVideoEncoder::initEncoder() {
    // 1. 查找编码器
    m_codec_ = avcodec_find_encoder_by_name(m_params_.m_codec_name.c_str());
    if (!m_codec_) {
        YLLOG_ERR("chn %d: Codec '%s' not found", m_chn_id_, m_params_.m_codec_name.c_str());
        return false;
    }
    
    // 2. 创建编码器上下文
    m_codec_ctx_ = avcodec_alloc_context3(m_codec_);
    if (!m_codec_ctx_) {
        YLLOG_ERR("chn %d: Could not allocate video codec context", m_chn_id_);
        return false;
    }
    
    // 3. 设置编码参数
    m_codec_ctx_->bit_rate = m_params_.m_bitrate;
    m_codec_ctx_->width = m_params_.m_width;
    m_codec_ctx_->height = m_params_.m_height;
    m_codec_ctx_->time_base = (AVRational){1, (int)m_params_.m_fps};
    m_codec_ctx_->framerate = (AVRational){(int)m_params_.m_fps, 1};
    m_codec_ctx_->gop_size = m_params_.m_gop_size;
    m_codec_ctx_->max_b_frames = m_params_.m_max_b_frames;
    m_codec_ctx_->pix_fmt = AV_PIX_FMT_YUV420P;  // 输出格式统一为YUV420P
    
    // 4. 设置编码器选项
    if (m_params_.m_codec_name == "libx264") {
        av_opt_set(m_codec_ctx_->priv_data, "preset", m_params_.m_preset.c_str(), 0);
        av_opt_set(m_codec_ctx_->priv_data, "tune", m_params_.m_tune.c_str(), 0);
        av_opt_set(m_codec_ctx_->priv_data, "profile", "baseline", 0);
    } else if (m_params_.m_codec_name == "libx265") {
        av_opt_set(m_codec_ctx_->priv_data, "preset", m_params_.m_preset.c_str(), 0);
        av_opt_set(m_codec_ctx_->priv_data, "tune", m_params_.m_tune.c_str(), 0);
    }
    
    // 5. 打开编码器
    int ret = avcodec_open2(m_codec_ctx_, m_codec_, nullptr);
    if (ret < 0) {
        char errbuf[AV_ERROR_MAX_STRING_SIZE];
        av_strerror(ret, errbuf, AV_ERROR_MAX_STRING_SIZE);
        YLLOG_ERR("chn %d: Could not open codec: %s", m_chn_id_, errbuf);
        return false;
    }
    
    // 6. 分配帧
    m_frame_ = av_frame_alloc();
    if (!m_frame_) {
        YLLOG_ERR("chn %d: Could not allocate video frame", m_chn_id_);
        return false;
    }
    
    m_frame_->format = m_codec_ctx_->pix_fmt;
    m_frame_->width = m_codec_ctx_->width;
    m_frame_->height = m_codec_ctx_->height;
    
    ret = av_frame_get_buffer(m_frame_, 32);
    if (ret < 0) {
        YLLOG_ERR("chn %d: Could not allocate frame data", m_chn_id_);
        return false;
    }
    
    // 7. 分配数据包
    m_packet_ = av_packet_alloc();
    if (!m_packet_) {
        YLLOG_ERR("chn %d: Could not allocate packet", m_chn_id_);
        return false;
    }
    
    // 8. 如果需要格式转换，初始化SwsContext
    if (m_params_.m_input_format == InputPixelFormat::RGB24) {
        m_sws_ctx_ = sws_getContext(
            m_params_.m_width, m_params_.m_height, AV_PIX_FMT_RGB24,
            m_params_.m_width, m_params_.m_height, AV_PIX_FMT_YUV420P,
            SWS_BICUBIC, nullptr, nullptr, nullptr);
        
        if (!m_sws_ctx_) {
            YLLOG_ERR("chn %d: Could not initialize sws context", m_chn_id_);
            return false;
        }
    }
    
    YLLOG_INFO("chn %d: Software encoder initialized - %dx%d@%dfps, bitrate=%d, format=%d", 
               m_chn_id_, m_params_.m_width, m_params_.m_height, m_params_.m_fps, 
               m_params_.m_bitrate, (int)m_params_.m_input_format);
    
    return true;
}

void SoftwareVideoEncoder::deinitEncoder() {
    if (m_sws_ctx_) {
        sws_freeContext(m_sws_ctx_);
        m_sws_ctx_ = nullptr;
    }
    
    if (m_packet_) {
        av_packet_free(&m_packet_);
    }
    
    if (m_frame_) {
        av_frame_free(&m_frame_);
    }
    
    if (m_codec_ctx_) {
        avcodec_free_context(&m_codec_ctx_);
    }
}

void SoftwareVideoEncoder::encodeThread() {
    while (m_b_running_) {
        Image_t *image_ptr = nullptr;
        
        if (!m_image_queue_->dequeue(image_ptr, 100)) {
            continue;
        }
        
        if (image_ptr) {
            encodeFrame(image_ptr);
            free(image_ptr);
        }
    }
}

bool SoftwareVideoEncoder::encodeFrame(Image_t *image_ptr) {
    if (!image_ptr || !m_frame_ || !m_codec_ctx_) {
        return false;
    }
    
    // 确保帧数据可写
    int ret = av_frame_make_writable(m_frame_);
    if (ret < 0) {
        YLLOG_ERR("chn %d: Could not make frame writable", m_chn_id_);
        return false;
    }
    
    // 转换输入数据到YUV420P格式
    if (!convertInputFrame(image_ptr, m_frame_)) {
        YLLOG_ERR("chn %d: Failed to convert input frame", m_chn_id_);
        return false;
    }
    
    // 设置时间戳
    m_frame_->pts = m_frame_index_++;
    
    // 发送帧到编码器
    ret = avcodec_send_frame(m_codec_ctx_, m_frame_);
    if (ret < 0) {
        char errbuf[AV_ERROR_MAX_STRING_SIZE];
        av_strerror(ret, errbuf, AV_ERROR_MAX_STRING_SIZE);
        YLLOG_ERR("chn %d: Error sending frame to encoder: %s", m_chn_id_, errbuf);
        return false;
    }
    
    // 接收编码后的数据包
    while (ret >= 0) {
        ret = avcodec_receive_packet(m_codec_ctx_, m_packet_);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
            break;
        } else if (ret < 0) {
            char errbuf[AV_ERROR_MAX_STRING_SIZE];
            av_strerror(ret, errbuf, AV_ERROR_MAX_STRING_SIZE);
            YLLOG_ERR("chn %d: Error receiving packet from encoder: %s", m_chn_id_, errbuf);
            return false;
        }
        
        // 发送编码数据到RTSP服务器
        sendEncodedPacket(m_packet_, image_ptr->m_timestamp);
        
        av_packet_unref(m_packet_);
    }
    
    fps();
    return true;
}

bool SoftwareVideoEncoder::convertInputFrame(Image_t *image_ptr, AVFrame *frame) {
    switch (m_params_.m_input_format) {
        case InputPixelFormat::RGB24:
            return convertRGB24ToYUV420P(image_ptr->m_data, frame);
        case InputPixelFormat::GRAY8:
            return convertGRAY8ToYUV420P(image_ptr->m_data, frame);
        case InputPixelFormat::YUV420P:
            // 直接拷贝YUV420P数据
            {
                uint32_t y_size = m_params_.m_width * m_params_.m_height;
                uint32_t uv_size = y_size / 4;
                
                memcpy(frame->data[0], image_ptr->m_data, y_size);
                memcpy(frame->data[1], image_ptr->m_data + y_size, uv_size);
                memcpy(frame->data[2], image_ptr->m_data + y_size + uv_size, uv_size);
            }
            return true;
        default:
            YLLOG_ERR("chn %d: Unsupported input format: %d", m_chn_id_, (int)m_params_.m_input_format);
            return false;
    }
}

bool SoftwareVideoEncoder::convertRGB24ToYUV420P(const uint8_t *rgb_data, AVFrame *frame) {
    if (!m_sws_ctx_) {
        YLLOG_ERR("chn %d: SwsContext not initialized for RGB conversion", m_chn_id_);
        return false;
    }
    
    // 设置RGB数据指针和步长
    const uint8_t *src_data[4] = { rgb_data, nullptr, nullptr, nullptr };
    int src_linesize[4] = { (int)(m_params_.m_width * 3), 0, 0, 0 };
    
    // 执行格式转换
    int ret = sws_scale(m_sws_ctx_, src_data, src_linesize, 0, m_params_.m_height,
                        frame->data, frame->linesize);
    
    if (ret != (int)m_params_.m_height) {
        YLLOG_ERR("chn %d: RGB to YUV conversion failed, expected %d got %d", 
                  m_chn_id_, m_params_.m_height, ret);
        return false;
    }
    
    return true;
}

bool SoftwareVideoEncoder::convertGRAY8ToYUV420P(const uint8_t *gray_data, AVFrame *frame) {
    uint32_t width = m_params_.m_width;
    uint32_t height = m_params_.m_height;
    
    // Y平面：直接拷贝灰度数据
    for (uint32_t y = 0; y < height; y++) {
        memcpy(frame->data[0] + y * frame->linesize[0], 
               gray_data + y * width, width);
    }
    
    // U和V平面：设置为128 (中性值)
    uint32_t uv_width = width / 2;
    uint32_t uv_height = height / 2;
    
    for (uint32_t y = 0; y < uv_height; y++) {
        memset(frame->data[1] + y * frame->linesize[1], 128, uv_width);
        memset(frame->data[2] + y * frame->linesize[2], 128, uv_width);
    }
    
    return true;
}

bool SoftwareVideoEncoder::sendEncodedPacket(AVPacket *packet, uint64_t timestamp) {
    if (m_params_.m_rtsp_chn_id < 0) {
        return true; // RTSP未配置，跳过发送
    }
    
    FrameHeader_t frame_hdr;
    frame_hdr.m_chn_id = m_params_.m_rtsp_chn_id;
    frame_hdr.m_stream_id = 0;
    
    // 根据编码器类型设置视频类型
    if (m_params_.m_codec_name == "libx265") {
        frame_hdr.m_venc_type = RTSP_VIDEO_TYPE_H265;
    } else {
        frame_hdr.m_venc_type = RTSP_VIDEO_TYPE_H264;
    }
    
    // 判断是否为关键帧
    frame_hdr.m_frame_type = (packet->flags & AV_PKT_FLAG_KEY) ? 0 : 1;
    frame_hdr.m_timestamp = timestamp;
    
    // 在数据末尾插入时间戳
    static thread_local std::vector<uint8_t> temp_buffer;
    temp_buffer.resize(packet->size + sizeof(uint64_t));
    
    memcpy(temp_buffer.data(), packet->data, packet->size);
    
    // 插入时间戳到末尾
    uint8_t *ts_ptr = temp_buffer.data() + packet->size;
    ts_ptr[0] = (timestamp >> 56) & 0xFF;
    ts_ptr[1] = (timestamp >> 48) & 0xFF;
    ts_ptr[2] = (timestamp >> 40) & 0xFF;
    ts_ptr[3] = (timestamp >> 32) & 0xFF;
    ts_ptr[4] = (timestamp >> 24) & 0xFF;
    ts_ptr[5] = (timestamp >> 16) & 0xFF;
    ts_ptr[6] = (timestamp >> 8) & 0xFF;
    ts_ptr[7] = timestamp & 0xFF;
    
    // 发送到RTSP服务器
    return RtspServerWrapper::getInstance()->sendFrame(
        frame_hdr, temp_buffer.data(), temp_buffer.size());
}

void SoftwareVideoEncoder::fps() {
    m_count_++;
    
    uint64_t cur_time = getStartupTimeMs();
    if (cur_time - m_last_time_ >= 1000) {
        double fps = m_count_ * 1000.0 / (cur_time - m_last_time_);
        
        YLLOG_DBG("Software Encoder chn %d: FPS %.1f", m_chn_id_, fps);
        
        m_count_ = 0;
        m_last_time_ = cur_time;
    }
}