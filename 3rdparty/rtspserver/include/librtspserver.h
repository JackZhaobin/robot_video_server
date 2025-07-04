/*
 * @Description:
 * @Version: V1.0
 * @Author: hongyuan.liu@corenetic.ai
 * @Date: 2025-03-07 14:12:26
 * @LastEditors: hongyuan.liu@corenetic.ai
 * @LastEditTime: 2025-03-15 12:33:11
 * Copyright (C) 2024-2050 Corenetic Technology Inc All rights reserved.
 */
#ifndef _LIBRTSPSERVER_H
#define _LIBRTSPSERVER_H

#ifdef __cplusplus
extern "C" {
#endif

enum
{
    RTSP_VIDEO_TYPE_H264,
    RTSP_VIDEO_TYPE_H265,
    RTSP_VIDEO_TYPE_MJPEG,
};

enum
{
    RTSP_AUDIO_TYPE_AAC,
    RTSP_AUDIO_TYPE_G711A,
};

typedef int(*RtspServerVerifyUserCallback) (const char *_pcUserName, const char *_pcPassWord);


int StartRTSPServer(int _iRtspServerPort, int _iChnCnt, int _iSubChnSupport, RtspServerVerifyUserCallback VerifyUserCallback);


int SendVideoStreamToRtspServer(int _iChn, int _iStreamId, int _iVencType, int _iFrameType, unsigned long long _ullTimeStamp,
	                                        int _iFrameSize, unsigned char *_pucFrameBuf);


int SendAudioStreamToRtspServer(int _iChn, int _iStreamId, int _iAencType, int _iSampleRate, int _iAudioChnCnt, unsigned long long _ullTimeStamp,
                                        int _iAudioLen, unsigned char *_pucAudoDataBuf);


int GetRtspServerConnectCnt(int _iChn, int _iStreamId);


int SetRtspServerPort(int _iPort);

int StopRtspServer();

#ifdef __cplusplus
}
#endif

#endif