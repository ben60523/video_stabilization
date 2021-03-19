/*
Copyright (c) 2014, Nghia Ho
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Modified by Satya Mallick, Big Vision LLC (Jan 2019)

*/

#include "stdafx.h"
#include "ffmpeg_cv_utils.hpp"

using namespace std;
using namespace cv;

const int SMOOTHING_RADIUS = 25; // In frames. The larger the more stable the video, but less reactive to sudden panning

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

	void getTransform(Mat& T)
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


vector<Trajectory> cumsum(vector<TransformParam>& transforms)
{
	vector <Trajectory> trajectory; // trajectory at all frames
	// Accumulated frame to frame transform
	double a = 0;
	double x = 0;
	double y = 0;

	for (size_t i = 0; i < transforms.size(); i++)
	{
		x += transforms[i].dx;
		y += transforms[i].dy;
		a += transforms[i].da;

		trajectory.push_back(Trajectory(x, y, a));

	}

	return trajectory;
}

vector <Trajectory> smooth(vector <Trajectory>& trajectory, int radius)
{
	vector <Trajectory> smoothed_trajectory;
	for (size_t i = 0; i < trajectory.size(); i++) {
		double sum_x = 0;
		double sum_y = 0;
		double sum_a = 0;
		int count = 0;

		for (int j = -radius; j <= radius; j++) {
			if (i + j >= 0 && i + j < trajectory.size()) {
				sum_x += trajectory[i + j].x;
				sum_y += trajectory[i + j].y;
				sum_a += trajectory[i + j].a;

				count++;
			}
		}

		double avg_a = sum_a / count;
		double avg_x = sum_x / count;
		double avg_y = sum_y / count;

		smoothed_trajectory.push_back(Trajectory(avg_x, avg_y, avg_a));
	}

	return smoothed_trajectory;
}

void fixBorder(Mat& frame_stabilized)
{
	Mat T = getRotationMatrix2D(Point2f(frame_stabilized.cols, frame_stabilized.rows), 0, 1.1);
	warpAffine(frame_stabilized, frame_stabilized, T, frame_stabilized.size());
}

int main(int argc, char** argv)
{
	// Read input video
	VideoCapture cap("video.mp4");
	// Get frame count
	int n_frames = int(cap.get(CAP_PROP_FRAME_COUNT));
	// Get frames per second (fps)
	double fps = cap.get(cv::CAP_PROP_FPS);
	// Get width and height of video stream
	int w = int(cap.get(CAP_PROP_FRAME_WIDTH));
	int h = int(cap.get(CAP_PROP_FRAME_HEIGHT));
	Mat curr, curr_gray;
	Mat prev, prev_gray;
	int nb_v_frame = 0, nb_a_frame = 0;
	cap >> prev;
	int ret = 0;
	InputVideoState* ivs = (InputVideoState*)malloc(sizeof(InputVideoState));
	AVFrame* input_frame = av_frame_alloc();
	openVideo(ivs, "video.mp4");
	// Convert frame to grayscale
	cvtColor(prev, prev_gray, COLOR_BGR2GRAY);
	vector <TransformParam> transforms;

	Mat last_T;
	for (int i = 0; i <= n_frames; i++)
	{
		// Vector from previous and current feature points
		vector <Point2f> prev_pts, curr_pts;
		goodFeaturesToTrack(prev_gray, prev_pts, 100, 0.01, 30);
		bool success = cap.read(curr);
		if (!success) break;
		cvtColor(curr, curr_gray, COLOR_BGR2GRAY);
		// Calculate optical flow (i.e. track feature points)
		vector <uchar> status;
		vector <float> err;
		calcOpticalFlowPyrLK(prev_gray, curr_gray, prev_pts, curr_pts, status, err);

		// Filter only valid points
		auto prev_it = prev_pts.begin();
		auto curr_it = curr_pts.begin();
		for (size_t k = 0; k < status.size(); k++)
		{
			if (status[k])
			{
				prev_it++;
				curr_it++;
			}
			else
			{
				prev_it = prev_pts.erase(prev_it);
				curr_it = curr_pts.erase(curr_it);
			}
		}

		// Find transformation matrix
		Mat T = estimateAffinePartial2D(prev_pts, curr_pts);
		// In rare cases no transform is found. 
		// We'll just use the last known good transform.
		if (T.data == NULL) last_T.copyTo(T);
		T.copyTo(last_T);
		// Extract traslation and rotation angle
		double dx = T.at<double>(0, 2);
		double dy = T.at<double>(1, 2);
		double da = atan2(T.at<double>(1, 0), T.at<double>(0, 0));
		transforms.push_back(TransformParam(dx, dy, da));
		curr_gray.copyTo(prev_gray);
		std::cout << "Frame: " << i << "/" << n_frames << " -  Tracked points : " << prev_pts.size() << '\r' << flush;
	}
	cout << endl;
	// Compute trajectory using cumulative sum of transformations
	vector <Trajectory> trajectory = cumsum(transforms);
	vector <Trajectory> smoothed_trajectory = smooth(trajectory, SMOOTHING_RADIUS);
	vector <TransformParam> transforms_smooth;
	for (size_t i = 0; i < transforms.size(); i++)
	{
		// Calculate difference in smoothed_trajectory and trajectory
		double diff_x = smoothed_trajectory[i].x - trajectory[i].x;
		double diff_y = smoothed_trajectory[i].y - trajectory[i].y;
		double diff_a = smoothed_trajectory[i].a - trajectory[i].a;
		// Calculate newer transformation array
		double dx = transforms[i].dx + diff_x;
		double dy = transforms[i].dy + diff_y;
		double da = transforms[i].da + diff_a;

		transforms_smooth.push_back(TransformParam(dx, dy, da));
	}
	cap.set(cv::CAP_PROP_POS_FRAMES, 0);
	Mat T(2, 3, CV_64F);
	Mat frame_mat, frame_stabilized, frame_out;
	// ========================= Open output format context ===================================//
	const AVRational dst_fps = { fps, 1 };
	AVFormatContext* outctx = nullptr;
	ret = avformat_alloc_output_context2(&outctx, nullptr, nullptr, "video_out.mp4");
	if (ret < 0) {
		cerr << "fail to avformat_alloc_output_context2(video_out.mp4): ret=" << ret;
		return 2;
	}
	// ===================== Find Encoder ======================================= //
	AVCodec* vcodec = avcodec_find_encoder(outctx->oformat->video_codec);
	AVCodec* acodec = avcodec_find_encoder(outctx->oformat->audio_codec);
	// ===================== open output IO context ============================= //
	ret = avio_open2(&outctx->pb, "video_out.mp4", AVIO_FLAG_WRITE, nullptr, nullptr);
	if (ret < 0) {
		cerr << "fail to avio_open2: ret=" << ret;
		return 2;
	}
	// ======================== Allocate codec context ========================== //
	AVCodecContext* v_codec_ctx = avcodec_alloc_context3(vcodec);
	v_codec_ctx->width = ivs->v_stream->codecpar->width;
	v_codec_ctx->height = ivs->v_stream->codecpar->height;
	v_codec_ctx->pix_fmt = *vcodec->pix_fmts;
	v_codec_ctx->time_base = av_inv_q(dst_fps);
	v_codec_ctx->bit_rate = ivs->v_stream->codecpar->bit_rate;
	v_codec_ctx->codec_id = outctx->oformat->video_codec;
	av_opt_set(v_codec_ctx->priv_data, "preset", "ultrafast", 0);
	AVCodecContext* a_codec_ctx = avcodec_alloc_context3(acodec);
	a_codec_ctx->channel_layout = ivs->a_stream->codecpar->channel_layout;
	a_codec_ctx->channels = ivs->a_stream->codecpar->channels;
	a_codec_ctx->sample_fmt = *acodec->sample_fmts;
	a_codec_ctx->time_base = av_inv_q({ ivs->a_stream->codecpar->sample_rate, 1 });
	a_codec_ctx->sample_rate = ivs->a_stream->codecpar->sample_rate;
	av_opt_set(a_codec_ctx->priv_data, "preset", "ultrafast", 0);
	if (outctx->oformat->flags & AVFMT_GLOBALHEADER)
		v_codec_ctx->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
	// ======================= Open video encoder ================================ //
	ret = avcodec_open2(v_codec_ctx, vcodec, nullptr);
	if (ret < 0) {
		char a[64];
		cerr << "[Video Encode] fail to avcodec_open2: ret=" << av_make_error_string(a, AV_ERROR_MAX_STRING_SIZE, ret);
		return 2;
	}
	// ======================= Open audio encoder ================================ //
	ret = avcodec_open2(a_codec_ctx, acodec, nullptr);
	if (ret < 0) {
		char a[64];
		cerr << "[Audio Encode] fail to avcodec_open2: ret=" << av_make_error_string(a, AV_ERROR_MAX_STRING_SIZE, ret);
		return 2;
	}
	// ======================= Allocate streams ================================== //
	AVStream* vstr = avformat_new_stream(outctx, vcodec);
	AVStream* astr = avformat_new_stream(outctx, acodec);
	avcodec_parameters_from_context(vstr->codecpar, v_codec_ctx);
	avcodec_parameters_from_context(astr->codecpar, a_codec_ctx);
	cout
		<< "format:  " << outctx->oformat->name << "\n"
		<< "vcodec:  " << vcodec->name << "\n"
		<< "acodec:  " << acodec->name << "\n"
		<< "size:    " << w << 'x' << h << "\n"
		<< "fps:     " << av_q2d(dst_fps) << "\n"
		<< "pixfmt:  " << av_get_pix_fmt_name(v_codec_ctx->pix_fmt) << "\n"
		<< flush;
	// ======================= Initialize sample scaler ========================== //
	SwrContext* swr_ctx = swr_alloc_set_opts(NULL,
		a_codec_ctx->channel_layout, a_codec_ctx->sample_fmt, a_codec_ctx->sample_rate,
		a_codec_ctx->channel_layout, a_codec_ctx->sample_fmt, a_codec_ctx->sample_rate, 0, NULL);
	SwsContext* swsctx = sws_getCachedContext(
		nullptr, w, h, AV_PIX_FMT_BGR24,
		w, h, v_codec_ctx->pix_fmt, SWS_BICUBIC, nullptr, nullptr, nullptr);
	if (!swsctx) {
		std::cerr << "fail to sws_getCachedContext";
		return 2;
	}
	// ======================= Allocate frame buffer for encoding ================ //
	AVFrame* vframe = av_frame_alloc();
	vector<uint8_t> vframebuf(av_image_get_buffer_size(v_codec_ctx->pix_fmt, w, h, 1));
	av_image_fill_arrays(vframe->data, vframe->linesize, vframebuf.data(), v_codec_ctx->pix_fmt, w, h, 1);
	vframe->width = w;
	vframe->height = h;
	vframe->format = v_codec_ctx->pix_fmt;
	AVFrame* aframe = av_frame_alloc();
	vector<uint8_t> aframebuf(av_samples_get_buffer_size(NULL, a_codec_ctx->channels, a_codec_ctx->sample_rate, a_codec_ctx->sample_fmt, 1));
	av_samples_fill_arrays(aframe->data, aframe->linesize, aframebuf.data(), aframe->channels, aframe->nb_samples, a_codec_ctx->sample_fmt, 1);
	avformat_write_header(outctx, NULL);
	cap.release();
	ret = 0;
	// ========================= Encoding ========================================= //
	while (ret >= 0)
	{
		ret = decodeFrame(ivs, input_frame);
		if (ret >= 0) {
			// Video frame
			if (ret == 2) {
				frame_mat = avframeToCvmat(input_frame);
				// Extract transform from translation and rotation angle. 
				transforms_smooth[nb_v_frame].getTransform(T);
				// Apply affine wrapping to the given frame
				warpAffine(frame_mat, frame_stabilized, T, frame_mat.size());
				// Scale image to remove black border artifact
				fixBorder(frame_stabilized);
				// Now draw the original and stablised side by side for coolness
				// If the image is too big, resize it.
				if (frame_out.cols > 1920)
				{
					resize(frame_out, frame_out, Size(frame_out.cols / 2, frame_out.rows / 2));
				}
				encodeCVMatByFFmpeg(frame_stabilized, swsctx, v_codec_ctx, outctx, vframe, vstr);
				nb_v_frame++;
			}
			else if (ret == 3) {
				encodeAudio(input_frame, swr_ctx, a_codec_ctx, outctx, aframe, astr);
				nb_a_frame++;
			}
		}
		cout << "video: " << nb_v_frame << " frames encoded, " << "audio: " << nb_a_frame << " frame encoded" << '\r' << flush;
	}

	// =================== Release video and Free memory ======================= //
end:
	cap.release();
	av_write_trailer(outctx);
	cout << endl;
	cout << "video: " << nb_v_frame << " frames encoded" << endl;
	cout << "audio: " << nb_a_frame << " frames encoded" << endl;
	av_frame_free(&vframe);
	av_frame_free(&aframe);
	av_frame_free(&input_frame);
	avcodec_close(v_codec_ctx);
	avcodec_close(a_codec_ctx);
	avio_close(outctx->pb);
	avformat_free_context(outctx);
	avcodec_close(ivs->a_codec_ctx);
	avcodec_close(ivs->v_codec_ctx);
	avformat_free_context(ivs->in_ctx);
	free(ivs);
	return 0;
}