/*
 * @Description:
 * @Version: V1.0
 * @Author: hongyuan.liu@corenetic.ai
 * @Date: 2025-03-13 05:22:01
 * @LastEditors: hongyuan.liu@corenetic.ai
 * @LastEditTime: 2025-04-22 17:22:28
 * Copyright (C) 2024-2050 Corenetic Technology Inc All rights reserved.
 */
#include "video_encoder.h"

#include "utils.hpp"
#include "yllog.h"

#define TEST_ERROR(cond, str, label) \
  if (cond) {                        \
    YLLOG_ERR(str);                  \
    goto label;                      \
  }

// static void dumpBufferYUV420M(const std::string &filename, NvBuffer &buffer)
// {
//   std::ofstream file(filename, std::ios::binary);
//   if (file.is_open()) {
//     char *data = nullptr;

//     for (uint32_t i = 0; i < buffer.n_planes; i++) {
//       NvBuffer::NvBufferPlane &plane = buffer.planes[i];

//       std::streamsize bytes_to_read = plane.fmt.bytesperpixel *
//       plane.fmt.width;

//       data = (char *)plane.data;

//       for (uint32_t j = 0; j < plane.fmt.height; j++) {
//         file.write(data, bytes_to_read);
//         data += plane.fmt.stride;
//       }
//     }

//     file.close();
//     std::cout << "YUV420M 数据已保存至：" << filename << std::endl;
//   } else {
//     std::cerr << "无法打开文件：" << filename << std::endl;
//   }
// }

// static void printNvBufferPlane(const NvBuffer *nvbuffer) {
//   std::cout << "NvBufferPlane Info (Plane 0):" << std::endl;
//   std::cout << "  Width: " << nvbuffer->planes[0].fmt.width << std::endl;
//   std::cout << "  Height: " << nvbuffer->planes[0].fmt.height << std::endl;
//   std::cout << "  Bytes per Pixel: " << nvbuffer->planes[0].fmt.bytesperpixel
//             << std::endl;
//   std::cout << "  Stride: " << nvbuffer->planes[0].fmt.stride << std::endl;
//   std::cout << "  Size Image: " << nvbuffer->planes[0].fmt.sizeimage
//             << std::endl;
//   std::cout << "  Data Pointer: "
//             << static_cast<void *>(nvbuffer->planes[0].data) << std::endl;
//   std::cout << "  Bytes Used: " << nvbuffer->planes[0].bytesused <<
//   std::endl; std::cout << "  File Descriptor (FD): " <<
//   nvbuffer->planes[0].fd
//             << std::endl;
//   std::cout << "  Memory Offset: " << nvbuffer->planes[0].mem_offset
//             << std::endl;
//   std::cout << "  Length: " << nvbuffer->planes[0].length << std::endl;

//   std::cout << "NvBufferPlane Info (Plane 1):" << std::endl;
//   std::cout << "  Width: " << nvbuffer->planes[1].fmt.width << std::endl;
//   std::cout << "  Height: " << nvbuffer->planes[1].fmt.height << std::endl;
//   std::cout << "  Bytes per Pixel: " << nvbuffer->planes[1].fmt.bytesperpixel
//             << std::endl;
//   std::cout << "  Stride: " << nvbuffer->planes[1].fmt.stride << std::endl;
//   std::cout << "  Size Image: " << nvbuffer->planes[1].fmt.sizeimage
//             << std::endl;
//   std::cout << "  Data Pointer: "
//             << static_cast<void *>(nvbuffer->planes[1].data) << std::endl;
//   std::cout << "  Bytes Used: " << nvbuffer->planes[1].bytesused <<
//   std::endl; std::cout << "  File Descriptor (FD): " <<
//   nvbuffer->planes[1].fd
//             << std::endl;
//   std::cout << "  Memory Offset: " << nvbuffer->planes[1].mem_offset
//             << std::endl;
//   std::cout << "  Length: " << nvbuffer->planes[1].length << std::endl;

//   std::cout << "NvBufferPlane Info (Plane 2):" << std::endl;
//   std::cout << "  Width: " << nvbuffer->planes[2].fmt.width << std::endl;
//   std::cout << "  Height: " << nvbuffer->planes[2].fmt.height << std::endl;
//   std::cout << "  Bytes per Pixel: " << nvbuffer->planes[2].fmt.bytesperpixel
//             << std::endl;
//   std::cout << "  Stride: " << nvbuffer->planes[2].fmt.stride << std::endl;
//   std::cout << "  Size Image: " << nvbuffer->planes[2].fmt.sizeimage
//             << std::endl;
//   std::cout << "  Data Pointer: "
//             << static_cast<void *>(nvbuffer->planes[2].data) << std::endl;
//   std::cout << "  Bytes Used: " << nvbuffer->planes[2].bytesused <<
//   std::endl; std::cout << "  File Descriptor (FD): " <<
//   nvbuffer->planes[2].fd
//             << std::endl;
//   std::cout << "  Memory Offset: " << nvbuffer->planes[2].mem_offset
//             << std::endl;
//   std::cout << "  Length: " << nvbuffer->planes[2].length << std::endl;
// }

// static int32_t write_encoder_output_frame(std::ofstream *stream,
//                                           NvBuffer *buffer) {
//   stream->write((char *)buffer->planes[0].data, buffer->planes[0].bytesused);
//   stream->flush();
//   return 0;
// }

void VideoEncoder::fps() {
  m_count_++;

  uint64_t cur_time = getStartupTimeMs();
  if (cur_time - m_last_time_ >= 1000) {
    double fps = m_count_ * 1000.0 / (cur_time - m_last_time_);

    YLLOG_DBG("Video Encoder chn %d: FPS %.1f", m_chn_id_, fps);

    m_count_ = 0;
    m_last_time_ = cur_time;
  }
}

/**
 * Encoder capture-plane deque buffer callback function.
 *
 * @param v4l2_buf      : v4l2 buffer
 * @param buffer        : NvBuffer
 * @param shared_buffer : shared NvBuffer
 * @param arg           : context pointer
 */
static bool encoderCaptureCallback(struct v4l2_buffer *v4l2_buf,
                                   NvBuffer *buffer, NvBuffer *shared_buffer,
                                   void *arg) {
  VideoEncoder *encoder = (VideoEncoder *)arg;

  //  char enc_cap_plane[16] = "EncCapPlane";
  //  string s = to_string (m_ctx_.thread_num);
  //  strcat (enc_cap_plane, s.c_str());
  //  pthread_setname_np (pthread_self(), enc_cap_plane);

  return encoder->processFrame(v4l2_buf, buffer, shared_buffer);
}

VideoEncoder::VideoEncoder(uint32_t chn_id, const EncCreateParam_t &enc_param)
    : m_chn_id_(chn_id) {
  setDefaults();

  #if 1
  m_ctx_.m_src_pixfmt = enc_param.m_raw_pixfmt;
  m_ctx_.m_width = enc_param.m_width;
  m_ctx_.m_height = enc_param.m_height;
  // m_ctx_.m_encoder_pixfmt = enc_param.m_encoder_pixfmt == 1 ?
  // V4L2_PIX_FMT_H265 : V4L2_PIX_FMT_H264;
  m_ctx_.m_encode_width = enc_param.m_encode_width;
  m_ctx_.m_encode_height = enc_param.m_encode_height;
  m_ctx_.m_bitrate = enc_param.m_bitrate;
  // m_ctx_.m_ratecontrol = enc_param.m_ratecontrol == 1 ?
  // V4L2_MPEG_VIDEO_BITRATE_MODE_CBR : V4L2_MPEG_VIDEO_BITRATE_MODE_VBR;
  m_ctx_.m_fps_n = enc_param.m_fps;
  m_ctx_.m_fps_d = 1;
  // m_ctx_.m_profile = enc_param.m_encoder_pixfmt == 1 ?
  // V4L2_MPEG_VIDEO_H265_PROFILE_MAIN : V4L2_MPEG_VIDEO_H264_PROFILE_MAIN;
  #endif

  m_rtsp_chn_id_ = enc_param.m_rtsp_chn_id;

  std::string tag = "[Video chn " + std::to_string(m_chn_id_) + "]";

  m_image_queue_ = std::make_unique<DataQueue<Image_t *>>(
      8, [](Image_t *image_ptr) { free(image_ptr); }, tag);
}

VideoEncoder::~VideoEncoder() { stop(); }

bool VideoEncoder::start() {
  if (!createEncoder()) {
    YLLOG_ERR("chn %d: Create encoder failed!", m_chn_id_);
    return false;
  }

  m_b_encode_ = true;
  m_encode_thread_ =
      std::make_shared<std::thread>(&VideoEncoder::encodeThread, this);

  m_b_init_ = true;

  return true;
}

void VideoEncoder::stop() {
  if (!m_b_init_) {
    return;
  }

  m_b_encode_ = false;
  if (m_encode_thread_ && m_encode_thread_->joinable()) {
    m_encode_thread_->join();
  }

  stopStream();
  destroyEncoder();

  m_b_init_ = false;
}

bool VideoEncoder::putImage(Image_t *image_ptr) {
  if(m_b_reseting_) {
    return false;
  }

  return m_image_queue_->enqueue(std::move(image_ptr));
}

bool VideoEncoder::insertTimestamp(uint8_t *data, uint64_t timestamp) {
  if (data == nullptr) {
    return false;
  }

  // printf("timestamp: %016x, %llu\n", timestamp, timestamp);
  // fprintf(stdout, "\n");

  data[0] = (timestamp >> 56) & 0xFF;
  data[1] = (timestamp >> 48) & 0xFF;
  data[2] = (timestamp >> 40) & 0xFF;
  data[3] = (timestamp >> 32) & 0xFF;
  data[4] = (timestamp >> 24) & 0xFF;
  data[5] = (timestamp >> 16) & 0xFF;
  data[6] = (timestamp >> 8) & 0xFF;
  data[7] = timestamp & 0xFF;

  return true;
}

bool VideoEncoder::processFrame(struct v4l2_buffer *v4l2_buf, NvBuffer *buffer,
                                NvBuffer *shared_buffer) {
  (void)shared_buffer;

  // uint32_t frame_num = m_ctx_.m_enc->capture_plane.getTotalDequeuedBuffers()
  // - 1;

  if (v4l2_buf == NULL) {
    YLLOG_ERR(
        "chn %d: Error while dequeing buffer from output plane, v4l2_buf is "
        "NULL",
        m_chn_id_);
    return false;
  }

  // std::cout << "ts sec: " << v4l2_buf->timestamp.tv_sec << ", ts usec: " <<
  // v4l2_buf->timestamp.tv_usec << std::endl;

  /* Received EOS from encoder. Stop dqthread. */
  if (buffer->planes[0].bytesused == 0) {
    YLLOG_WARN("chn %d: Got 0 size buffer in capture, frame count %u",
               m_chn_id_, m_frame_count_.load());
    return false;
  }

  m_frame_count_++;

  v4l2_ctrl_videoenc_outputbuf_metadata enc_metadata;
  if (m_ctx_.m_enc->getMetadata(v4l2_buf->index, enc_metadata) == 0 &&
      enc_metadata.KeyFrame == 1) {
    // std::cout << "Video Encoder Output Buffer Metadata:" << std::endl;
    // std::cout << "  KeyFrame: " <<
    // static_cast<int32_t>(enc_metadata.KeyFrame) << std::endl; std::cout << "
    // EndofFrame: " << static_cast<int32_t>(enc_metadata.EndofFrame) <<
    // std::endl; std::cout << "  AvgQP: " << enc_metadata.AvgQP << std::endl;
    // std::cout << "  IsGoldenOrAlternateFrame: " <<
    // static_cast<int32_t>(enc_metadata.bIsGoldenOrAlternateFrame) <<
    // std::endl; std::cout << "  ValidReconCRC: " <<
    // static_cast<int32_t>(enc_metadata.bValidReconCRC) << std::endl; std::cout
    // << "  ReconFrame_Y_CRC: " << enc_metadata.ReconFrame_Y_CRC << std::endl;
    // std::cout << "  ReconFrame_U_CRC: " << enc_metadata.ReconFrame_U_CRC <<
    // std::endl; std::cout << "  ReconFrame_V_CRC: " <<
    // enc_metadata.ReconFrame_V_CRC << std::endl; std::cout << "
    // EncodedFrameBits: " << enc_metadata.EncodedFrameBits << std::endl;
    // std::cout << "  FrameMinQP: " << enc_metadata.FrameMinQP << std::endl;
    // std::cout << "  FrameMaxQP: " << enc_metadata.FrameMaxQP << std::endl;
    // std::cout << "  RPSFeedback_status: " << enc_metadata.bRPSFeedback_status
    // << std::endl; std::cout << "  CurrentRefFrameId: " <<
    // enc_metadata.nCurrentRefFrameId << std::endl; std::cout << "
    // ActiveRefFrames: " << enc_metadata.nActiveRefFrames << std::endl;
  }

  FrameHeader_t frame_hdr;
  frame_hdr.m_chn_id = m_rtsp_chn_id_;
  frame_hdr.m_stream_id = 0;
  frame_hdr.m_venc_type = m_ctx_.m_encoder_pixfmt == V4L2_PIX_FMT_H265
                              ? RTSP_VIDEO_TYPE_H265
                              : RTSP_VIDEO_TYPE_H264;
  frame_hdr.m_frame_type = enc_metadata.KeyFrame == 1 ? 0 : 1;
  frame_hdr.m_timestamp =
      v4l2_buf->timestamp.tv_sec * 1000000 + v4l2_buf->timestamp.tv_usec;

  // printf("Encoder chn %d: timestamp %lu\n", m_rtsp_chn_id_,
  // frame_hdr.m_timestamp);

  // 为了避免拷贝，暂时直接插入到数据最后面，可能存在风险
  insertTimestamp(buffer->planes[0].data + buffer->planes[0].bytesused,
                  frame_hdr.m_timestamp);
  uint32_t data_len = buffer->planes[0].bytesused + sizeof(uint64_t);

  // if(buffer->planes[0].bytesused <= 100)
  // {
  //   printf("Too small frame %u\n", buffer->planes[0].bytesused);
  // }

  RtspServerWrapper::getInstance()->sendFrame(frame_hdr, buffer->planes[0].data,
                                              data_len);

  fps();

  int32_t ret = -1;
  ret = m_ctx_.m_enc->capture_plane.qBuffer(*v4l2_buf, NULL);
  if (ret < 0) {
    YLLOG_ERR("chn %d: Error while enqueuing buffer to capture plane, ret = %d",
              m_chn_id_, -1);
    return false;
  }

  return true;
}

bool VideoEncoder::setupOutputDmaBuf(uint32_t num_buffers) {
  int32_t ret = 0;
  int32_t fd;
  NvBufSurf::NvCommonAllocateParams cParams;

  ret = m_ctx_.m_enc->output_plane.reqbufs(V4L2_MEMORY_DMABUF, num_buffers);
  if (ret) {
    YLLOG_ERR(
        "chn %d: reqbufs failed for output plane V4L2_MEMORY_DMABUF, ret = %d",
        m_chn_id_, ret);
    return false;
  }

  for (uint32_t i = 0; i < m_ctx_.m_enc->output_plane.getNumBuffers(); i++) {
    cParams.width = m_ctx_.m_width;
    cParams.height = m_ctx_.m_height;
    cParams.layout = NVBUF_LAYOUT_PITCH;
    if (m_ctx_.m_enableLossless &&
        m_ctx_.m_encoder_pixfmt == V4L2_PIX_FMT_H264) {
      cParams.colorFormat = NVBUF_COLOR_FORMAT_YUV444;
    } else if (m_ctx_.m_profile == V4L2_MPEG_VIDEO_H265_PROFILE_MAIN10) {
      cParams.colorFormat = NVBUF_COLOR_FORMAT_NV12_10LE;
    } else {
      cParams.colorFormat = m_ctx_.m_enable_extended_colorformat
                                ? NVBUF_COLOR_FORMAT_YUV420_ER
                                : NVBUF_COLOR_FORMAT_YUV420;
    }
    cParams.memtag = NvBufSurfaceTag_VIDEO_ENC;
    cParams.memType = NVBUF_MEM_SURFACE_ARRAY;

    /* Create output plane fd for DMABUF io-mode */
    ret = NvBufSurf::NvAllocate(&cParams, 1, &fd);
    if (ret < 0) {
      YLLOG_ERR(
          "chn %d: Failed to create NvBuffer, ret = %d, width = %d, height = "
          "%d",
          m_chn_id_, ret, cParams.width, cParams.height);
      return false;
    }
    m_ctx_.m_output_plane_fd[i] = fd;
  }

  return true;
}

void VideoEncoder::destroyOutputDmaBuf() {
  int32_t ret = 0;

  for (uint32_t i = 0; i < m_ctx_.m_enc->output_plane.getNumBuffers(); i++) {
    /* Unmap output plane buffer for memory type DMABUF. */
    ret = m_ctx_.m_enc->output_plane.unmapOutputBuffers(
        i, m_ctx_.m_output_plane_fd[i]);
    if (ret < 0) {
      YLLOG_ERR(
          "chn %d: Error while unmapping buffer at output plane, ret = %d",
          m_chn_id_, ret);
      return;
    }

    ret = NvBufSurf::NvDestroy(m_ctx_.m_output_plane_fd[i]);
    if (ret < 0) {
      YLLOG_ERR("chn %d: Failed to destroy NvBuffer, ret = %d, fd = %d", ret,
                m_chn_id_, m_ctx_.m_output_plane_fd[i]);
      return;
    }
    m_ctx_.m_output_plane_fd[i] = -1;
  }
}

bool VideoEncoder::allocImageBuf() {
  int32_t ret = 0;

  /* Query, Export and Map the output plane buffers so that we can read
   * raw data into the buffers
   */
  switch (m_ctx_.m_output_memory_type) {
    case V4L2_MEMORY_MMAP:
      ret = m_ctx_.m_enc->output_plane.setupPlane(V4L2_MEMORY_MMAP, 10, true,
                                                  false);
      if (ret < 0) {
        YLLOG_ERR("chn %d: Could not setup output plane, ret = %d", m_chn_id_,
                  ret);
        return false;
      }
      break;

    case V4L2_MEMORY_USERPTR:
      ret = m_ctx_.m_enc->output_plane.setupPlane(V4L2_MEMORY_USERPTR, 10,
                                                  false, true);
      if (ret < 0) {
        YLLOG_ERR("chn %d: Could not setup output plane, ret = %d", m_chn_id_,
                  ret);
        return false;
      }
      break;

    case V4L2_MEMORY_DMABUF:
      ret = setupOutputDmaBuf(10);
      if (ret < 0) {
        YLLOG_ERR("chn %d: Could not setup output plane, ret = %d", m_chn_id_,
                  ret);
        return false;
      }
      break;
    default:
      YLLOG_ERR("chn %d: Not a valid plane", m_chn_id_);
      return false;
  }

  m_ctx_.m_b_output_buf_alloc = true;

  return true;
}

void VideoEncoder::freeImageBuf() {
  if (m_ctx_.m_output_memory_type == V4L2_MEMORY_DMABUF) {
    destroyOutputDmaBuf();
  }

  m_ctx_.m_b_output_buf_alloc = false;
}

bool VideoEncoder::setupCaptureDmaBuf(uint32_t num_buffers) {
  NvBufSurfaceAllocateParams cParams;
  NvBufSurface *surface = nullptr;
  int32_t ret = 0;

  memset(&cParams, 0, sizeof(NvBufSurfaceAllocateParams));

  ret = m_ctx_.m_enc->capture_plane.reqbufs(V4L2_MEMORY_DMABUF, num_buffers);
  if (ret) {
    YLLOG_ERR(
        "chn %d: reqbufs failed for capture plane V4L2_MEMORY_DMABUF, ret = %d",
        m_chn_id_, ret);
    return false;
  }

  for (uint32_t i = 0; i < m_ctx_.m_enc->capture_plane.getNumBuffers(); i++) {
    ret = m_ctx_.m_enc->capture_plane.queryBuffer(i);
    if (ret) {
      YLLOG_ERR("chn %d: Error in querying for %dth buffer plane, ret = %d",
                m_chn_id_, i, ret);
      return false;
    }

    NvBuffer *buffer = m_ctx_.m_enc->capture_plane.getNthBuffer(i);

    cParams.params.memType = NVBUF_MEM_HANDLE;
    cParams.params.size = buffer->planes[0].length;
    cParams.memtag = NvBufSurfaceTag_VIDEO_ENC;

    ret = NvBufSurfaceAllocate(&surface, 1, &cParams);
    if (ret < 0) {
      YLLOG_ERR("chn %d: Failed to create NvBuffer, ret = %d, size = %d",
                m_chn_id_, ret, cParams.params.size);
      return false;
    }
    surface->numFilled = 1;

    m_ctx_.m_capture_plane_fd[i] = surface->surfaceList[0].bufferDesc;
  }

  return true;
}

void VideoEncoder::destroyCaptureDmaBuf() {
  int32_t ret = 0;

  for (uint32_t i = 0; i < m_ctx_.m_enc->capture_plane.getNumBuffers(); i++) {
    /* Unmap capture plane buffer for memory type DMABUF. */
    ret = m_ctx_.m_enc->capture_plane.unmapOutputBuffers(
        i, m_ctx_.m_capture_plane_fd[i]);
    if (ret < 0) {
      YLLOG_ERR(
          "chn %d: Error while unmapping buffer at capture plane, ret = %d",
          m_chn_id_, ret);
      return;
    }

    NvBufSurface *nvbuf_surf = 0;
    ret = NvBufSurfaceFromFd(m_ctx_.m_capture_plane_fd[i],
                             (void **)(&nvbuf_surf));
    if (ret < 0) {
      YLLOG_ERR("chn %d: Error while NvBufSurfaceFromFd, ret = %d", m_chn_id_,
                ret);
      return;
    }

    ret = NvBufSurfaceDestroy(nvbuf_surf);
    if (ret < 0) {
      YLLOG_ERR("chn %d: Failed to destroy NvBuffer, ret = %d, fd = %d",
                m_chn_id_, ret, m_ctx_.m_capture_plane_fd[i]);
      return;
    }
  }
}

bool VideoEncoder::allocStreamBuf() {
  int32_t ret = 0;

  /* Query, Export and Map the capture plane buffers so that we can write
      encoded bitstream data into the buffers */
  switch (m_ctx_.m_capture_memory_type) {
    case V4L2_MEMORY_MMAP:
      ret = m_ctx_.m_enc->capture_plane.setupPlane(
          V4L2_MEMORY_MMAP, m_ctx_.m_num_output_buffers, true, false);
      if (ret < 0) {
        YLLOG_ERR("chn %d: Could not setup capture plane, ret = %d", m_chn_id_,
                  ret);
        return false;
      }
      break;
    case V4L2_MEMORY_DMABUF:
      ret = setupCaptureDmaBuf(m_ctx_.m_num_output_buffers);
      if (ret < 0) {
        YLLOG_ERR("chn %d: Could not setup capture plane, ret = %d", m_chn_id_,
                  ret);
        return false;
      }
      break;
    default:
      YLLOG_ERR("chn %d: Not a valid plane", m_chn_id_);
      return false;
  }

  m_ctx_.m_b_capture_buf_alloc = true;

  return true;
}

void VideoEncoder::freeStreamBuf() {
  if (m_ctx_.m_capture_memory_type == V4L2_MEMORY_DMABUF) {
    destroyCaptureDmaBuf();
  }

  m_ctx_.m_b_capture_buf_alloc = false;
}

bool VideoEncoder::enqueueStreamBuf() {
  int32_t ret = 0;

  /* Enqueue all the empty capture plane buffers. */
  for (uint32_t i = 0; i < m_ctx_.m_enc->capture_plane.getNumBuffers(); i++) {
    struct v4l2_buffer v4l2_buf;
    struct v4l2_plane planes[MAX_PLANES];

    memset(&v4l2_buf, 0, sizeof(v4l2_buf));
    memset(planes, 0, MAX_PLANES * sizeof(struct v4l2_plane));

    v4l2_buf.index = i;
    v4l2_buf.m.planes = planes;

    if (m_ctx_.m_capture_memory_type == V4L2_MEMORY_DMABUF) {
      v4l2_buf.m.planes[0].m.fd = m_ctx_.m_capture_plane_fd[i];

      /* Map capture plane buffer for memory type DMABUF. */
      ret = m_ctx_.m_enc->capture_plane.mapOutputBuffers(
          v4l2_buf, m_ctx_.m_capture_plane_fd[i]);

      if (ret < 0) {
        YLLOG_ERR(
            "chn %d: Error while mapping buffer at capture plane, ret = %d",
            m_chn_id_, ret);
        return false;
      }
    }

    ret = m_ctx_.m_enc->capture_plane.qBuffer(v4l2_buf, NULL);
    if (ret < 0) {
      YLLOG_ERR(
          "chn %d: Error while queueing buffer at capture plane, ret = %d",
          m_chn_id_, ret);
      return false;
    }
  }

  return true;
}

void VideoEncoder::setDefaults() {
  memset(static_cast<void *>(&m_ctx_), 0, sizeof(EncoderContext_t));

  m_ctx_.m_enc = nullptr;

  m_ctx_.m_encoder_pixfmt = V4L2_PIX_FMT_H264;
  m_ctx_.m_raw_pixfmt = V4L2_PIX_FMT_YUV420M;
  m_ctx_.m_src_pixfmt = V4L2_PIX_FMT_YUV420M;
  m_ctx_.m_width = 1920;
  m_ctx_.m_height = 1080;

  m_ctx_.m_bitrate = 4 * 1024 * 1024;
  m_ctx_.m_enable_ratecontrol = true;
  m_ctx_.m_profile = V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE;
  m_ctx_.m_level = -1;
  m_ctx_.m_ratecontrol = V4L2_MPEG_VIDEO_BITRATE_MODE_CBR;
  m_ctx_.m_iframe_interval = 30;
  m_ctx_.m_idr_interval = 30;
  m_ctx_.m_fps_n = 30;
  m_ctx_.m_fps_d = 1;
  m_ctx_.m_encode_width = 1920;
  m_ctx_.m_encode_height = 1080;
  m_ctx_.m_num_reference_frames = 1;
  m_ctx_.m_num_b_frames = 0;
  m_ctx_.m_hw_preset_type = V4L2_ENC_HW_PRESET_ULTRAFAST;
  m_ctx_.m_output_memory_type = V4L2_MEMORY_DMABUF;
  m_ctx_.m_capture_memory_type = V4L2_MEMORY_MMAP;

  m_ctx_.m_enable_extended_colorformat = false;
  m_ctx_.m_input_frames_queued_count = 0;
  m_ctx_.m_num_output_buffers = 6;

  m_ctx_.m_blocking_mode = true;
  m_ctx_.m_enableLossless = false;
  m_ctx_.m_insert_sps_pps_at_idr = true;
  m_ctx_.m_b_use_enc_cmd = false;

  m_ctx_.m_b_output_buf_alloc = false;
  m_ctx_.m_b_capture_buf_alloc = false;
  m_ctx_.m_b_copy_timestamp = true;

  m_ctx_.m_ouput_buf_idx = 0;
}

bool VideoEncoder::createEncoder() {
  int32_t ret = 0;

  /* Create NvVideoEncoder object for blocking or non-blocking I/O mode. */
  if (m_ctx_.m_blocking_mode) {
    YLLOG_INFO("chn %d: Creating Encoder in blocking mode.", m_chn_id_);
    m_ctx_.m_enc = NvVideoEncoder::createVideoEncoder("enc0");
  } else {
    YLLOG_INFO("chn %d: Creating Encoder in non-blocking mode.", m_chn_id_);
    m_ctx_.m_enc = NvVideoEncoder::createVideoEncoder("enc0", O_NONBLOCK);
  }

  if (!m_ctx_.m_enc) {
    YLLOG_ERR("chn %d: Failed to create encoder", m_chn_id_);
    return false;
  }

  /*
   * Set encoder capture plane format.
   * NOTE: It is necessary that Capture Plane format be set before Output Plane
   * format. It is necessary to set width and height on the capture plane as
   * well
   */

  /* Set encoder capture plane format.
   * NOTE: It is necessary that Capture Plane format be set before Output Plane
   * format. It is necessary to set width and height on the capture plane as
   * well
   */
  ret = m_ctx_.m_enc->setCapturePlaneFormat(m_ctx_.m_encoder_pixfmt,
                                            m_ctx_.m_width, m_ctx_.m_height,
                                            2 * 1024 * 1024);
  if (ret < 0) {
    YLLOG_ERR("chn %d: Could not set capture plane format, ret = %d", m_chn_id_,
              ret);
    goto cleanup;
  }

  switch (m_ctx_.m_profile) {
    case V4L2_MPEG_VIDEO_H265_PROFILE_MAIN10:
      m_ctx_.m_raw_pixfmt = V4L2_PIX_FMT_P010M;
      break;
    case V4L2_MPEG_VIDEO_H265_PROFILE_MAIN:
    default:
      m_ctx_.m_raw_pixfmt = V4L2_PIX_FMT_YUV420M;
  }

  /* Set encoder output plane format */
  if (m_ctx_.m_enableLossless && m_ctx_.m_encoder_pixfmt == V4L2_PIX_FMT_H264) {
    m_ctx_.m_profile = V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_444_PREDICTIVE;
    m_ctx_.m_raw_pixfmt = V4L2_PIX_FMT_YUV444M;
  }
  ret = m_ctx_.m_enc->setOutputPlaneFormat(m_ctx_.m_raw_pixfmt, m_ctx_.m_width,
                                           m_ctx_.m_height);
  if (ret < 0) {
    YLLOG_ERR("chn %d: Could not set output plane format, ret = %d", m_chn_id_,
              ret);
    goto cleanup;
  }

  ret = m_ctx_.m_enc->setBitrate(m_ctx_.m_bitrate);
  if (ret < 0) {
    YLLOG_ERR("chn %d: Could not set bitrate, ret = %d", m_chn_id_, ret);
    goto cleanup;
  }

  if (m_ctx_.m_encoder_pixfmt == V4L2_PIX_FMT_H264) {
    /* Set encoder profile for H264 format */
    ret = m_ctx_.m_enc->setProfile(m_ctx_.m_profile);
    if (ret < 0) {
      YLLOG_ERR("chn %d: Could not set encoder profile, ret = %d", m_chn_id_,
                ret);
      goto cleanup;
    }

    if (m_ctx_.m_level == (uint32_t)-1) {
      m_ctx_.m_level = (uint32_t)V4L2_MPEG_VIDEO_H264_LEVEL_5_1;
    }

    /* Set encoder level for H264 format */
    ret = m_ctx_.m_enc->setLevel(m_ctx_.m_level);
    if (ret < 0) {
      YLLOG_ERR("chn %d: Could not set encoder level, ret = %d", m_chn_id_,
                ret);
      goto cleanup;
    }
  } else if (m_ctx_.m_encoder_pixfmt == V4L2_PIX_FMT_H265) {
    ret = m_ctx_.m_enc->setProfile(m_ctx_.m_profile);
    if (ret < 0) {
      YLLOG_ERR("chn %d: Could not set encoder profile, ret = %d", m_chn_id_,
                ret);
      goto cleanup;
    }

    if (m_ctx_.m_level != (uint32_t)-1) {
      ret = m_ctx_.m_enc->setLevel(m_ctx_.m_level);
      if (ret < 0) {
        YLLOG_ERR("chn %d: Could not set encoder level, ret = %d", m_chn_id_,
                  ret);
        goto cleanup;
      }
    }
  }

  /* Set rate control mode for encoder */
  ret = m_ctx_.m_enc->setRateControlMode(m_ctx_.m_ratecontrol);
  if (ret < 0) {
    YLLOG_ERR("chn %d: Could not set encoder rate control mode, ret = %d",
              m_chn_id_, ret);
    goto cleanup;
  }

  /* Set IDR frame interval for encoder */
  ret = m_ctx_.m_enc->setIDRInterval(m_ctx_.m_idr_interval);
  if (ret < 0) {
    YLLOG_ERR("chn %d: Could not set encoder IDR interval, ret = %d", m_chn_id_,
              ret);
    goto cleanup;
  }

  /* Set I frame interval for encoder */
  ret = m_ctx_.m_enc->setIFrameInterval(m_ctx_.m_iframe_interval);
  if (ret < 0) {
    YLLOG_ERR("chn %d: Could not set encoder I-Frame interval, ret = %d",
              m_chn_id_, ret);
    goto cleanup;
  }

  /* Set framerate for encoder */
  ret = m_ctx_.m_enc->setFrameRate(m_ctx_.m_fps_n, m_ctx_.m_fps_d);
  if (ret < 0) {
    YLLOG_ERR("chn %d: Could not set encoder framerate, ret = %d", m_chn_id_,
              ret);
    goto cleanup;
  }

  if (m_ctx_.m_hw_preset_type) {
    /* Set hardware preset value for encoder */
    ret = m_ctx_.m_enc->setHWPresetType(
        V4L2_ENC_HW_PRESET_ULTRAFAST /*m_ctx_.m_hw_preset_type*/);
    if (ret < 0) {
      YLLOG_ERR("chn %d: Could not set encoder HW Preset Type, ret = %d",
                m_chn_id_, ret);
      goto cleanup;
    }
  }

  if (m_ctx_.m_num_reference_frames) {
    /* Set number of reference frame configuration value for encoder */
    ret = m_ctx_.m_enc->setNumReferenceFrames(m_ctx_.m_num_reference_frames);
    if (ret < 0) {
      YLLOG_ERR("chn %d: Could not set encoder num reference frames, ret = %d",
                m_chn_id_, ret);
      goto cleanup;
    }
  }

  if (m_ctx_.m_insert_sps_pps_at_idr) {
    /* Enable insert of SPSPPS at IDR frames */
    ret = m_ctx_.m_enc->setInsertSpsPpsAtIdrEnabled(true);
    if (ret < 0) {
      YLLOG_ERR("chn %d: Could not set insert SPSPPS at IDR, ret = %d",
                m_chn_id_, ret);
      goto cleanup;
    }
  }

  // if (m_ctx_.m_num_b_frames != (uint32_t) -1)
  // {
  //     /* Set number of B-frames to to be used by encoder */
  //     ret = m_ctx_.m_enc->setNumBFrames(m_ctx_.m_num_b_frames);
  //     TEST_ERROR(ret < 0, "Could not set number of B Frames", cleanup);
  // }

  /* Query, Export and Map the output plane buffers so that we can read
   * raw data into the buffers
   */
  switch (m_ctx_.m_output_memory_type) {
    case V4L2_MEMORY_MMAP:
      ret = m_ctx_.m_enc->output_plane.setupPlane(V4L2_MEMORY_MMAP, 10, true,
                                                  false);
      if (ret < 0) {
        YLLOG_ERR(
            "chn %d: Could not setup V4L2_MEMORY_MMAP output plane, ret = %d",
            m_chn_id_, ret);
        goto cleanup;
      }
      break;

    case V4L2_MEMORY_USERPTR:
      ret = m_ctx_.m_enc->output_plane.setupPlane(V4L2_MEMORY_USERPTR, 10,
                                                  false, true);
      if (ret < 0) {
        YLLOG_ERR(
            "chn %d: Could not setup V4L2_MEMORY_USERPTR output plane, ret = %d",
            m_chn_id_, ret);
        goto cleanup;
      }
      break;

    case V4L2_MEMORY_DMABUF:
      ret = setupOutputDmaBuf(10);
      if (ret < 0) {
        YLLOG_ERR("chn %d: Could not setup output dma buf, ret = %d", m_chn_id_,
                  ret);
        goto cleanup;
      }
      break;
    default:
      YLLOG_ERR("chn %d: Not a valid plane", m_chn_id_);
      goto cleanup;
  }

  /* Query, Export and Map the capture plane buffers so that we can write
   * encoded bitstream data into the buffers
   */
  ret = m_ctx_.m_enc->capture_plane.setupPlane(
      V4L2_MEMORY_MMAP, m_ctx_.m_num_output_buffers, true, false);
  if (ret < 0) {
    YLLOG_ERR(
        "chn %d: Could not setup V4L2_MEMORY_MMAP capture plane, ret = %d",
        m_chn_id_, ret);
    goto cleanup;
  }

  /* Subscibe for End Of Stream event */
  ret = m_ctx_.m_enc->subscribeEvent(V4L2_EVENT_EOS, 0, 0);
  if (ret < 0) {
    YLLOG_ERR("chn %d: Could not subscribe EOS event, ret = %d", m_chn_id_,
              ret);
    goto cleanup;
  }

  /* set encoder output plane STREAMON */
  ret = m_ctx_.m_enc->output_plane.setStreamStatus(true);
  if (ret < 0) {
    YLLOG_ERR("chn %d: Error in output plane streamon, ret = %d", m_chn_id_,
              ret);
    goto cleanup;
  }

  /* set encoder capture plane STREAMON */
  ret = m_ctx_.m_enc->capture_plane.setStreamStatus(true);
  if (ret < 0) {
    YLLOG_ERR("chn %d: Error in capture plane streamon, ret = %d", m_chn_id_,
              ret);
    goto cleanup;
  }

  if (m_ctx_.m_blocking_mode) {
    /* Set encoder capture plane dq thread callback for blocking io mode */
    m_ctx_.m_enc->capture_plane.setDQThreadCallback(encoderCaptureCallback);

    /* startDQThread starts a thread internally which calls the
     * encoderCaptureCallback whenever a buffer is dequeued
     * on the plane
     */
    m_ctx_.m_enc->capture_plane.startDQThread(this);
  }

  /* Enqueue all the empty capture plane buffers. */
  for (uint32_t i = 0; i < m_ctx_.m_enc->capture_plane.getNumBuffers(); i++) {
    struct v4l2_buffer v4l2_buf;
    struct v4l2_plane planes[MAX_PLANES];

    memset(&v4l2_buf, 0, sizeof(v4l2_buf));
    memset(planes, 0, MAX_PLANES * sizeof(struct v4l2_plane));

    v4l2_buf.index = i;
    v4l2_buf.m.planes = planes;

    ret = m_ctx_.m_enc->capture_plane.qBuffer(v4l2_buf, NULL);
    if (ret < 0) {
      YLLOG_ERR(
          "chn %d: Error while queueing buffer at capture plane, ret = %d",
          m_chn_id_, ret);
      m_ctx_.m_enc->abort();
      goto cleanup;
    }
  }

  return true;

cleanup:

  /* Release encoder configuration specific resources. */
  if (m_ctx_.m_enc) {
    delete m_ctx_.m_enc;
    m_ctx_.m_enc = nullptr;
  }

  return false;
}

void VideoEncoder::destroyEncoder() {
  if (m_ctx_.m_b_output_buf_alloc) {
    freeImageBuf();
  }

  if (m_ctx_.m_b_capture_buf_alloc) {
    freeStreamBuf();
  }

  if (m_ctx_.m_enc) {
    delete m_ctx_.m_enc;
    m_ctx_.m_enc = nullptr;
  }
}

void VideoEncoder::asyncResetEncoder() {
  if (m_b_reseting_) {
    YLLOG_ERR("chn %d: Encoder is resetting, skip reset", m_chn_id_);
  }

  m_b_reseting_ = true;

  std::thread([this]() {
    destroyEncoder();
    createEncoder();
    m_image_queue_->clear();
    m_b_reseting_ = false;
  }).detach();
}

bool VideoEncoder::startStream() {
  int32_t ret = 0;

  /* set encoder output plane STREAMON */
  ret = m_ctx_.m_enc->output_plane.setStreamStatus(true);
  if (ret < 0) {
    YLLOG_ERR("chn %d: Error in output plane streamon, ret = %d", m_chn_id_,
              ret);
    return false;
  }

  /* set encoder capture plane STREAMON */
  ret = m_ctx_.m_enc->capture_plane.setStreamStatus(true);
  if (ret < 0) {
    YLLOG_ERR("chn %d: Error in capture plane streamon, ret = %d", m_chn_id_,
              ret);
    return false;
  }

  /* Set encoder capture plane dq thread callback for blocking io mode */
  m_ctx_.m_enc->capture_plane.setDQThreadCallback(encoderCaptureCallback);

  /* startDQThread starts a thread internally which calls the
   * encoderCaptureCallback whenever a buffer is dequeued
   * on the plane
   */
  m_ctx_.m_enc->capture_plane.startDQThread((void *)this);

  return true;
}

void VideoEncoder::stopStream() {
  int32_t ret = 0;

  ret = m_ctx_.m_enc->abort();
  if (ret < 0) {
    YLLOG_ERR("chn %d: Error in output plane streamOFF, ret = %d", m_chn_id_,
              ret);
  }

  m_ctx_.m_enc->capture_plane.waitForDQThread(-1);

  // /* set encoder output plane STREAMOFF */
  // ret = m_ctx_.m_enc->output_plane.setStreamStatus(false);
  // if(ret < 0)
  // {
  //     std::cout << "Error in output plane streamon" << std::endl;
  // }

  // /* set encoder capture plane STREAMOFF */
  // ret = m_ctx_.m_enc->capture_plane.setStreamStatus(false);
  // if(ret < 0)
  // {
  //     std::cout << "Error in output plane streamon" << std::endl;
  // }

  // m_ctx_.m_enc->capture_plane.stopDQThread();
}

void VideoEncoder::encodeThread() {
  while (m_b_encode_) {
    if(m_b_reseting_) {
      // YLLOG_ERR("chn %d: Encoder is resetting, skip encode", m_chn_id_);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    Image_t *image_ptr = nullptr;

    if (m_image_queue_->dequeue(image_ptr, 100)) {

      if(image_ptr->m_width != m_ctx_.m_width ||
         image_ptr->m_height != m_ctx_.m_height)
      {
        YLLOG_ERR("chn %d: Image size not match, %dx%d != %dx%d", m_chn_id_,
                  image_ptr->m_width, image_ptr->m_height, m_ctx_.m_width,
                  m_ctx_.m_height);
        m_ctx_.m_width = image_ptr->m_width;
        m_ctx_.m_height = image_ptr->m_height;
        m_ctx_.m_encode_width = m_ctx_.m_encode_width > m_ctx_.m_width ? m_ctx_.m_width : m_ctx_.m_encode_width;
        m_ctx_.m_encode_height = m_ctx_.m_encode_height > m_ctx_.m_height ? m_ctx_.m_height : m_ctx_.m_encode_height;
        // asyncResetEncoder();
      }
      else {
        doEncode(image_ptr);
      }

      free(image_ptr);
    }
  }
}

bool VideoEncoder::fillVideoFrame(NvBuffer &buffer, uint8_t *src_data) {
  // TimePerf t("fill video frame");

  if (m_ctx_.m_src_pixfmt == V4L2_PIX_FMT_YUV420M) {
    uint32_t read_offset = 0;
    char *data = nullptr;

    for (uint32_t i = 0; i < buffer.n_planes; i++) {
      NvBuffer::NvBufferPlane &plane = buffer.planes[i];

      std::streamsize bytes_to_read = plane.fmt.bytesperpixel * plane.fmt.width;

      // std::cout << "stride: " << plane.fmt.stride << ", width: " <<
      // plane.fmt.width << ", height: " << plane.fmt.height << std::endl;
      // std::cout << "bytes_to_read: " << bytes_to_read << ", read_offset: " <<
      // read_offset << std::endl;

      data = (char *)plane.data;
      plane.bytesused = 0;

      for (uint32_t j = 0; j < plane.fmt.height; j++) {
        memcpy(data, src_data + read_offset, bytes_to_read);
        data += plane.fmt.stride;
        read_offset += bytes_to_read;
      }

      plane.bytesused = plane.fmt.stride * plane.fmt.height;
    }
  } else if (m_ctx_.m_src_pixfmt == V4L2_PIX_FMT_NV12) {
    uint32_t read_offset = 0;
    char *data = nullptr;

    // 1. 处理 Y 平面（直接拷贝）
    {
      NvBuffer::NvBufferPlane &plane = buffer.planes[0];
      uint32_t width = plane.fmt.width;
      uint32_t height = plane.fmt.height;
      uint32_t stride = plane.fmt.stride;

      data = (char *)plane.data;
      plane.bytesused = 0;

      for (uint32_t j = 0; j < height; j++) {
        memcpy(data, src_data + read_offset, width);
        data += stride;        // 目标数据按 stride 递增
        read_offset += width;  // 源数据按 width 递增
      }

      plane.bytesused = stride * height;
    }

    // 2. 处理 UV 平面（转换为 U/V 分离）
    {
      NvBuffer::NvBufferPlane &plane_u = buffer.planes[1];  // U 平面
      NvBuffer::NvBufferPlane &plane_v = buffer.planes[2];  // V 平面

      uint32_t uv_width = plane_u.fmt.width;    // U/V 平面宽度
      uint32_t uv_height = plane_u.fmt.height;  // U/V 平面高度（H/2）
      uint32_t uv_stride =
          plane_u.fmt.stride;  // U/V 平面 stride（可能比 uv_width 大）

      uint8_t *src_uv = src_data + read_offset;  // NV12 UV 起始地址
      char *data_u = (char *)plane_u.data;
      char *data_v = (char *)plane_v.data;

      plane_u.bytesused = 0;
      plane_v.bytesused = 0;

      for (uint32_t j = 0; j < uv_height; j++) {
        for (uint32_t i = 0; i < uv_width; i++) {
          data_u[i] = src_uv[i * 2];      // U (Cb)
          data_v[i] = src_uv[i * 2 + 1];  // V (Cr)
        }
        src_uv += uv_width * 2;  // NV12 UV 交错数据，每行 `width` 字节
        data_u += uv_stride;
        data_v += uv_stride;
      }

      plane_u.bytesused = uv_stride * uv_height;
      plane_v.bytesused = uv_stride * uv_height;
    }
  }

  return true;
}

bool VideoEncoder::enqueueImageBuf(uint32_t idx, Image_t *image_ptr) {
  int32_t ret = 0;

  struct v4l2_buffer v4l2_buf;
  struct v4l2_plane planes[MAX_PLANES];
  NvBuffer *buffer = m_ctx_.m_enc->output_plane.getNthBuffer(idx);

  memset(&v4l2_buf, 0, sizeof(v4l2_buf));
  memset(planes, 0, MAX_PLANES * sizeof(struct v4l2_plane));

  v4l2_buf.index = idx;
  v4l2_buf.m.planes = planes;

  if (m_ctx_.m_output_memory_type == V4L2_MEMORY_DMABUF) {
    v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    v4l2_buf.memory = V4L2_MEMORY_DMABUF;
    /* Map output plane buffer for memory type DMABUF. */
    ret = m_ctx_.m_enc->output_plane.mapOutputBuffers(
        v4l2_buf, m_ctx_.m_output_plane_fd[idx]);

    if (ret < 0) {
      YLLOG_ERR("chn %d: Error while mapping buffer at output plane, ret = %d",
                m_chn_id_, ret);
      return false;
    }

    // std::cout << "map outpout buffers index " << idx << ", fd " <<
    // m_ctx_.m_output_plane_fd[idx] << std::endl;
  }

  fillVideoFrame(*buffer, image_ptr->m_data);

  if (m_ctx_.m_b_copy_timestamp) {
    v4l2_buf.flags |= V4L2_BUF_FLAG_TIMESTAMP_COPY;
    v4l2_buf.timestamp.tv_sec = image_ptr->m_timestamp / 1000000;
    v4l2_buf.timestamp.tv_usec = image_ptr->m_timestamp % 1000000;
  }

  if (m_ctx_.m_output_memory_type == V4L2_MEMORY_DMABUF ||
      m_ctx_.m_output_memory_type == V4L2_MEMORY_MMAP) {
    for (uint32_t j = 0; j < buffer->n_planes; j++) {
      NvBufSurface *nvbuf_surf = 0;
      ret = NvBufSurfaceFromFd(buffer->planes[j].fd, (void **)(&nvbuf_surf));
      if (ret < 0) {
        YLLOG_ERR("chn %d: Error while NvBufSurfaceFromFd, ret = %d", m_chn_id_,
                  ret);
        return false;
      }
      ret = NvBufSurfaceSyncForDevice(nvbuf_surf, 0, j);
      if (ret < 0) {
        YLLOG_ERR(
            "chn %d: Error while NvBufSurfaceSyncForDevice at output plane for "
            "V4L2_MEMORY_DMABUF, ret = %d",
            m_chn_id_, ret);
        return false;
      }

      // printf("sync for device, plane %d @%s,%d\n", j, __func__, __LINE__);
    }
  }

  if (m_ctx_.m_output_memory_type == V4L2_MEMORY_DMABUF) {
    for (uint32_t j = 0; j < buffer->n_planes; j++) {
      v4l2_buf.m.planes[j].bytesused = buffer->planes[j].bytesused;
    }
  }

  // if(idx == 4)
  // {
  //     dumpBufferYUV420M("/home/corenetic/robot_workspace/test4_yuv420m.yuv",
  //     *buffer);
  // }

  /* encoder qbuffer for output plane */
  ret = m_ctx_.m_enc->output_plane.qBuffer(v4l2_buf, NULL);
  if (ret < 0) {
    YLLOG_ERR("chn %d: Error while queueing buffer at output plane, ret = %d",
              m_chn_id_, ret);
    return false;
  }

  return true;
}

bool VideoEncoder::pushImage(Image_t *image_ptr) {
  int32_t ret = 0;
  struct v4l2_buffer v4l2_buf;
  struct v4l2_plane planes[MAX_PLANES];
  NvBuffer *buffer = nullptr;

  memset(&v4l2_buf, 0, sizeof(v4l2_buf));
  memset(planes, 0, MAX_PLANES * sizeof(struct v4l2_plane));

  v4l2_buf.m.planes = planes;

  ret = m_ctx_.m_enc->output_plane.dqBuffer(v4l2_buf, &buffer, NULL, 10);
  if (ret < 0) {
    YLLOG_ERR("chn %d: Error while DQing buffer at output plane, ret = %d",
              m_chn_id_, ret);
    return false;
  }

  fillVideoFrame(*buffer, image_ptr->m_data);

  // static uint32_t count = 0;
  // if(++count == 5)
  // {
  //     dumpBufferYUV420M("/home/corenetic/robot_workspace/test4_yuv420m.yuv",
  //     *buffer);
  // }

  if (m_ctx_.m_b_copy_timestamp) {
    v4l2_buf.flags |= V4L2_BUF_FLAG_TIMESTAMP_COPY;
    v4l2_buf.timestamp.tv_sec = image_ptr->m_timestamp / 1000000;
    v4l2_buf.timestamp.tv_usec = image_ptr->m_timestamp % 1000000;
  }

  if (m_ctx_.m_output_memory_type == V4L2_MEMORY_DMABUF ||
      m_ctx_.m_output_memory_type == V4L2_MEMORY_MMAP) {
    for (uint32_t j = 0; j < buffer->n_planes; j++) {
      NvBufSurface *nvbuf_surf = 0;
      ret = NvBufSurfaceFromFd(buffer->planes[j].fd, (void **)(&nvbuf_surf));
      // std::cout << "buffer->planes[j].fd : " << buffer->planes[j].fd <<
      // std::endl;
      if (ret < 0) {
        YLLOG_ERR("chn %d: Error while NvBufSurfaceFromFd, ret = %d", m_chn_id_,
                  ret);
        return false;
      }
      ret = NvBufSurfaceSyncForDevice(nvbuf_surf, 0, j);
      if (ret < 0) {
        YLLOG_ERR(
            "chn %d: Error while NvBufSurfaceSyncForDevice at output plane for "
            "V4L2_MEMORY_DMABUF, ret = %d",
            m_chn_id_, ret);
        return false;
      }
      // printf("sync for device, plane %d @%s,%d\n", j, __func__, __LINE__);
    }
  }

  if (m_ctx_.m_output_memory_type == V4L2_MEMORY_DMABUF) {
    for (uint32_t j = 0; j < buffer->n_planes; j++) {
      v4l2_buf.m.planes[j].bytesused = buffer->planes[j].bytesused;
      // std::cout << "buffer->planes[j].bytesused: " <<
      // buffer->planes[j].bytesused << std::endl;
    }
  }

  /* encoder qbuffer for output plane */
  ret = m_ctx_.m_enc->output_plane.qBuffer(v4l2_buf, NULL);
  if (ret < 0) {
    YLLOG_ERR("chn %d: Error while queueing buffer at output plane, ret = %d",
              m_chn_id_, ret);
    return false;
  }

  return true;
}

bool VideoEncoder::doEncode(Image_t *image_ptr) {
  // Queue all the output plane buffers.
  if (m_ctx_.m_ouput_buf_idx < m_ctx_.m_enc->output_plane.getNumBuffers()) {
    enqueueImageBuf(m_ctx_.m_ouput_buf_idx, image_ptr);
    m_ctx_.m_ouput_buf_idx++;
    // std::cout << "m_ouput_buf_idx: " << m_ctx_.m_ouput_buf_idx <<std::endl;
  } else {
    pushImage(image_ptr);
  }

  return true;
}
