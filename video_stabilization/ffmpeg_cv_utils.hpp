#pragma once
#include "stdafx.h"

typedef struct {
	AVFormatContext* in_ctx;
	AVStream* v_stream;
	int v_stream_idx;
	AVStream* a_stream;
	int a_stream_idx;
	AVCodecContext* v_codec_ctx;
	AVCodecContext* a_codec_ctx;
} InputVideoState;

static int open_codec_context(InputVideoState* ivs, enum AVMediaType type);

void openVideo(InputVideoState* ivs, const char* src);

int decodeFrame(InputVideoState* ivs, AVFrame* frame);
/**
* Write cv::Mat into media file with ffmpeg API
* Note. (Video Only)
*/
void encodeCVMatByFFmpeg(cv::Mat image, SwsContext* swsctx, AVCodecContext* v_codec_ctx, AVFormatContext* outctx, AVFrame* frame, AVStream* vstream);

/**
* Encode Audio
*/
void encodeAudio(AVFrame *src, SwrContext* swr_ctx, AVCodecContext* a_codec_ctx, AVFormatContext* outctx, AVFrame* dst, AVStream* astream);

/**
* AVFrame to cv::Mat
*/
cv::Mat avframeToCvmat(const AVFrame* frame);


/**
*	cv::Mat to AVFrame
*/
AVFrame* cvmatToAvframe(cv::Mat* image, AVFrame* frame);