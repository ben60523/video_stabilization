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
#define PREVIEW 0
#define KALMAN_FILTER 0

#include "stdafx.h"
#include "ffmpeg_cv_utils.hpp"

using namespace std;
using namespace cv;

const int SMOOTHING_RADIUS = 75; // In frames. The larger the more stable the video, but less reactive to sudden panning
const char* FILENAME = "video_6.mp4";

void fixBorder(Mat& frame_stabilized)
{
	Mat T = getRotationMatrix2D(Point2f(frame_stabilized.cols / 2, frame_stabilized.rows / 2), 0, 1.1);
	warpAffine(frame_stabilized, frame_stabilized, T, frame_stabilized.size());
}

int main(int argc, char** argv)
{
	// Read input video
	VideoCapture cap(FILENAME);
	// Get frame count
	// Get frames per second (fps)
	double fps = cap.get(cv::CAP_PROP_FPS);
	// Get width and height of video stream
	int w = int(cap.get(CAP_PROP_FRAME_WIDTH));
	w = w * 2; // concat image 
	int h = int(cap.get(CAP_PROP_FRAME_HEIGHT));
	int nb_v_frame = 0, nb_a_frame = 0;
	int ret = 0;
	InputVideoState* ivs = (InputVideoState*)malloc(sizeof(InputVideoState));
	AVFrame* input_frame = av_frame_alloc();
	openVideo(ivs, FILENAME);
	vector <TransformParam> transforms_smooth;
	if (KALMAN_FILTER == 1) {
		transforms_smooth = video_stabilization_with_kalman_filter(cap, 0.004, 0.5, 1, SMOOTHING_RADIUS);
	}
	else {
		transforms_smooth = video_stabilization_with_average(cap, SMOOTHING_RADIUS);
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
	v_codec_ctx->width = ivs->v_stream->codecpar->width * 2;
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
				double x = T.at<double>(0, 2);
				double y = T.at<double>(1, 2);
				double arctan = atan2(T.at<double>(1, 0), T.at<double>(1, 1));
				// if concat, w / 200
				// or w /100
				if (1) {
					/*cout <<
						"-cos = " << setprecision(3) << T.at<double>(0, 0) <<
						", -sin = " << setprecision(3) << T.at<double>(0, 1) <<
						", sin = " << setprecision(3) << T.at<double>(1, 0) <<
						", cos = " << setprecision(3) << T.at<double>(1, 1) <<
						", dx = " << setprecision(3) << T.at<double>(0, 2) <<
						", dy = " << setprecision(3) << T.at<double>(1, 2) << endl;*/
					// Apply affine wrapping to the given frame
					warpAffine(frame_mat, frame_stabilized, T, frame_mat.size());
					// Scale image to remove black border artifact

				}
				else {
					//frame_stabilized.copyTo(frame_mat);
					frame_stabilized = frame_mat.clone();
				}
				fixBorder(frame_stabilized);
				hconcat(frame_mat, frame_stabilized, frame_out); // concat image
				// Now draw the original and stablised side by side for coolness
				// If the image is too big, resize it.
#if PREVIEW
				if (frame_out.cols > 1920)
				{
					resize(frame_out, frame_out, Size(frame_out.cols / 4, frame_out.rows / 4));
				}
				moveWindow("preview", 0, 0);
				imshow("preview", frame_out);

				int keyboard = waitKey(10);
				if (keyboard == 'q' || keyboard == 27)
					break;
#else				
				encodeCVMatByFFmpeg(frame_out, swsctx, v_codec_ctx, outctx, vframe, vstr);
#endif
				nb_v_frame++;

			}
#if !PREVIEW
			else if (ret == 3) {
				encodeAudio(input_frame, swr_ctx, a_codec_ctx, outctx, aframe, astr);
				nb_a_frame++;
			}
#endif // !PERVIEW

		}
		cout << "video: " << nb_v_frame << " frames encoded, " << "audio: " << nb_a_frame << " frame encoded" << '\r' << flush;
	}

	// =================== Release video and Free memory ======================= //
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
#ifdef PREVIEW
	destroyWindow("preview");
#endif // PREVIEW
	return 0;
}