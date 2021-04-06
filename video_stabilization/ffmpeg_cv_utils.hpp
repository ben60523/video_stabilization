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


struct TransformParam
{
	TransformParam() {}
	TransformParam(double _dx, double _dy, double _da)
	{
		dx = _dx;
		dy = _dy;
		da = _da;
	}

	double dx;
	double dy;
	double da; // angle

	void getTransform(cv::Mat& T)
	{
		// Reconstruct transformation matrix accordingly to new values
		T.at<double>(0, 0) = cos(da);
		T.at<double>(0, 1) = -sin(da);
		T.at<double>(1, 0) = sin(da);
		T.at<double>(1, 1) = cos(da);

		T.at<double>(0, 2) = dx;
		T.at<double>(1, 2) = dy;
	}
};


struct Trajectory
{
	Trajectory() {}
	Trajectory(double _x, double _y, double _a) {
		x = _x;
		y = _y;
		a = _a;
	}

	double x;
	double y;
	double a; // angle
};

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
void encodeAudio(AVFrame* src, SwrContext* swr_ctx, AVCodecContext* a_codec_ctx, AVFormatContext* outctx, AVFrame* dst, AVStream* astream);

/**
* AVFrame to cv::Mat
*/
cv::Mat avframeToCvmat(const AVFrame* frame);

/**
* Cumulative Sum
*/
std::vector<Trajectory> cumsum(std::vector<TransformParam>& transforms);

/**
* Smooth
*/
std::vector<Trajectory> smooth(std::vector <Trajectory>& trajectory, int radius);

/**
* Video Stabilization with average smoothing
*/

std::vector<TransformParam>video_stabilization_with_average(cv::VideoCapture cap, int SMOOTHING_RADIUS);

std::vector<TransformParam>video_stabilization_with_kalman_filter(cv::VideoCapture cap, double Q1, double R1, double E1, int SMOOTHING_RADIUS);

cv::Mat getTransformMatrix(cv::Mat curr_gray, std::vector<cv::Point2f> curr_pts, cv::Mat prev_gray, std::vector<cv::Point2f> prev_pts);
/**
*	cv::Mat to AVFrame
*/
AVFrame* cvmatToAvframe(cv::Mat* image, AVFrame* frame);