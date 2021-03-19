#include "stdafx.h"
#include "ffmpeg_cv_utils.hpp"


using namespace std;
using namespace cv;





static int open_codec_context(InputVideoState* ivs, enum AVMediaType type)
{
	int ret;
	AVStream* st;
	AVCodec* dec = NULL;
	ret = av_find_best_stream(ivs->in_ctx, type, -1, -1, NULL, 0);
	if (ret < 0) {
		return ret;
	}
	if (type == AVMEDIA_TYPE_VIDEO) {
		ivs->v_codec_ctx = avcodec_alloc_context3(NULL);
		avcodec_parameters_to_context(ivs->v_codec_ctx, ivs->in_ctx->streams[ret]->codecpar);
		dec = avcodec_find_decoder(ivs->in_ctx->streams[ret]->codecpar->codec_id);
		ivs->v_stream_idx = ret;
		ivs->v_codec_ctx->time_base = ivs->in_ctx->streams[ret]->r_frame_rate;
		if ((ret = avcodec_open2(ivs->v_codec_ctx, dec, NULL)) < 0) {
			return ret;
		}
	}
	else if (type == AVMEDIA_TYPE_AUDIO) {
		ivs->a_codec_ctx = avcodec_alloc_context3(NULL);
		avcodec_parameters_to_context(ivs->a_codec_ctx, ivs->in_ctx->streams[ret]->codecpar);
		dec = avcodec_find_decoder(ivs->in_ctx->streams[ret]->codecpar->codec_id);
		ivs->a_stream_idx = ret;
		ivs->a_codec_ctx->time_base = ivs->in_ctx->streams[ret]->r_frame_rate;
		if ((ret = avcodec_open2(ivs->a_codec_ctx, dec, NULL)) < 0) {
			return ret;
		}
	}
	return 0;
}
// Open file and allocate input video state
void openVideo(InputVideoState* ivs, const char* src) {
	// Open file and allocate format context
	ivs->in_ctx = avformat_alloc_context();
	int ret = avformat_open_input(&ivs->in_ctx, src, NULL, NULL);
	if (ret < 0) {
		char a[60];
		cerr << "fail to avformat_open_input: ret=" << av_make_error_string(a, 64, ret) << "\n";
		exit(ret);
	}
	// Retrieve stream info
	ret = avformat_find_stream_info(ivs->in_ctx, NULL);
	if (ret < 0) {
		char a[60];
		cerr << "fail to avformat_find_stream_info: ret=" << av_make_error_string(a, 64, ret) << "\n";
		exit(ret);
	}
	if (open_codec_context(ivs, AVMEDIA_TYPE_VIDEO) >= 0) {
		ivs->v_stream = ivs->in_ctx->streams[ivs->v_stream_idx];
		//avcodec_parameters_from_context(ivs->v_stream->codecpar, ivs->v_codec_ctx);
	}

	if (open_codec_context(ivs, AVMEDIA_TYPE_AUDIO) >= 0) {
		ivs->a_stream = ivs->in_ctx->streams[ivs->a_stream_idx];
		//avcodec_parameters_from_context(ivs->a_stream->codecpar, ivs->a_codec_ctx);
	}
	// dump input info
	av_dump_format(ivs->in_ctx, 0, src, 0);

	if (!ivs->a_stream && !ivs->v_stream) {
		cerr << "Cannot find any audio or video stream in the input" << endl;
		if (ivs->v_codec_ctx)
			avcodec_close(ivs->v_codec_ctx);
		if (ivs->a_codec_ctx)
			avcodec_close(ivs->a_codec_ctx);
		avformat_close_input(&ivs->in_ctx);
		exit(ret);
	}
}

int decodeFrame(InputVideoState* ivs, AVFrame* frame) {
	int got_frame;
	AVPacket* pkt = av_packet_alloc();
	pkt->data = NULL;
	pkt->size = 0;
	// Read frame from the input file
	int ret = av_read_frame(ivs->in_ctx, pkt);
	if (ret < 0) {
		char a[60];
		cerr << "fail to av_read_frame: ret=" << av_make_error_string(a, 64, ret) << "\n";
		return ret;
	}
	// Decode video frame
	if (pkt->stream_index == ivs->v_stream_idx) {
		ret = avcodec_send_packet(ivs->v_codec_ctx, pkt);
		if (ret < 0) {
			char a[60];
			cerr << "[Video] fail to avcodec_send_packet: ret=" << av_make_error_string(a, 64, ret) << "\n";
			return ret;
		}
		ret = avcodec_receive_frame(ivs->v_codec_ctx, frame);
		if (ret < 0) {
			if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
				return ret;
			}
			char a[60];
			cerr << "[Video] fail to avcodec_receive_frame: ret=" << av_make_error_string(a, 64, ret) << "\n";
			return ret;
		}
		return 2;
	}
	// Decode audio frame
	else if (pkt->stream_index == ivs->a_stream_idx) {
		ret = avcodec_send_packet(ivs->a_codec_ctx, pkt);
		if (ret < 0) {
			char a[60];
			cerr << "[Audio] fail to avcodec_send_packet: ret=" << av_make_error_string(a, 64, ret) << "\n";
			return ret;
		}
		ret = avcodec_receive_frame(ivs->a_codec_ctx, frame);
		if (ret < 0) {
			if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
				return ret;
			}
			char a[60];
			cerr << "[Audio] fail to avcodec_receive_frame: ret=" << av_make_error_string(a, 64, ret) << "\n";
			return ret;
		}
		return 3;
	}
}

void encodeCVMatByFFmpeg(Mat image, SwsContext* swsctx, AVCodecContext* v_codec_ctx, AVFormatContext* outctx, AVFrame* frame, AVStream* vstream) {
	const int stride[] = { static_cast<int>(image.step[0]) };
	static int64_t frame_pts = 0;
	sws_scale(swsctx, &image.data, stride, 0, image.rows, frame->data, frame->linesize);
	frame->pts = frame_pts++;
	int ret = avcodec_send_frame(v_codec_ctx, frame);
	if (ret < 0) {
		char a[60];
		cerr << "fail to avcodec_encode_video2: ret=" << av_make_error_string(a, 64, ret) << "\n";
	}
	while (ret >= 0) {
		AVPacket* pkt = av_packet_alloc();
		ret = avcodec_receive_packet(v_codec_ctx, pkt);
		if (ret < 0) {
			if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
				av_packet_unref(pkt);
				av_packet_free(&pkt);
				return;
			}
			char a[60];
			cerr << "fail to avcodec_receive_packet: ret=" << av_make_error_string(a, 64, ret) << "\n";
			av_packet_unref(pkt);
			av_packet_free(&pkt);
			break;
		}
		// rescale packet timestamp
		av_packet_rescale_ts(pkt, v_codec_ctx->time_base, vstream->time_base);
		// write packet
		ret = av_interleaved_write_frame(outctx, pkt);
		if (ret < 0) {
			char a[60];
			cerr << "fail to av_interleaved_write_frame: ret=" << av_make_error_string(a, 64, ret) << "\n";
			av_packet_unref(pkt);
			av_packet_free(&pkt);
			break;
		}
		av_packet_unref(pkt);
		av_packet_free(&pkt);
		//cout << frame_pts << "/" << totoal_frames << '\r' << flush;  // dump progress
	}
}

void encodeAudio(AVFrame* src, SwrContext* swr_ctx, AVCodecContext* a_codec_ctx, AVFormatContext* outctx, AVFrame* frame, AVStream* astream) {
	int ret = 0;
	/*ret = swr_convert(swr_ctx, dst->data, dst>nb_samples, (const uint8_t**)src->data, src->nb_samples);
				if (ret < 0) {
					char a[64];
					cerr << "swr_convert error with " << av_make_error_string(a, 64, ret);
				}*/
	ret = avcodec_send_frame(a_codec_ctx, src);
	if (ret < 0) {
		char a[60];
		cerr << "[Audio Encode] fail to avcodec_send_frame: ret=" << av_make_error_string(a, 64, ret) << "\n";
	}
	while (ret >= 0) {
		AVPacket* a_pkt = av_packet_alloc();
		ret = avcodec_receive_packet(a_codec_ctx, a_pkt);
		if (ret < 0) {
			if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
				av_packet_unref(a_pkt);
				av_packet_free(&a_pkt);
				ret = 0;
				return;
			}
			char a[60];
			cerr << "[Audio Encode] fail to avcodec_receive_packet: ret=" << av_make_error_string(a, 64, ret) << "\n";
			av_packet_unref(a_pkt);
			av_packet_free(&a_pkt);
			break;
		}
		// rescale packet timestamp
		av_packet_rescale_ts(a_pkt, a_codec_ctx->time_base, astream->time_base);
		a_pkt->stream_index = 1;
		// write packet
		ret = av_interleaved_write_frame(outctx, a_pkt);
		if (ret < 0) {
			char a[60];
			cerr << "[Audio Encode] fail to av_interleaved_write_frame: ret=" << av_make_error_string(a, 64, ret) << "\n";
			av_packet_unref(a_pkt);
			av_packet_free(&a_pkt);
			break;
		}
		av_packet_unref(a_pkt);
		av_packet_free(&a_pkt);
	}
}

/**
* AVFrame to cv::Mat
*/
Mat avframeToCvmat(const AVFrame* frame)
{
	int width = frame->width;
	int height = frame->height;
	Mat image(height, width, CV_8UC3);
	int cvLinesizes[1];
	cvLinesizes[0] = image.step1();
	SwsContext* conversion = sws_getContext(width, height, (AVPixelFormat)frame->format, width, height, AVPixelFormat::AV_PIX_FMT_BGR24, SWS_FAST_BILINEAR, NULL, NULL, NULL);
	sws_scale(conversion, frame->data, frame->linesize, 0, height, &image.data, cvLinesizes);
	sws_freeContext(conversion);
	return image;
}


/**
*	cv::Mat to AVFrame
*/
AVFrame* cvmatToAvframe(Mat* image, AVFrame* frame) {
	int width = image->cols;
	int height = image->rows;
	int cvLinesizes[1];
	cvLinesizes[0] = image->step1();
	if (frame == NULL) {
		frame = av_frame_alloc();
		av_image_alloc(frame->data, frame->linesize, width, height, AVPixelFormat::AV_PIX_FMT_YUV420P, 1);
	}
	SwsContext* conversion = sws_getContext(width, height, AVPixelFormat::AV_PIX_FMT_BGR24, width, height, (AVPixelFormat)frame->format, SWS_FAST_BILINEAR, NULL, NULL, NULL);
	sws_scale(conversion, &image->data, cvLinesizes, 0, height, frame->data, frame->linesize);
	sws_freeContext(conversion);
	return  frame;
}