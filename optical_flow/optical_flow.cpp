#include "stdafx.h"

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
#include "../video_stabilization/ffmpeg_cv_utils.hpp"
using namespace cv;
using namespace std;
int main(int argc, char** argv)
{
    const string about =
        "This sample demonstrates Lucas-Kanade Optical Flow calculation.\n"
        "The example file can be downloaded from:\n"
        "  https://www.bogotobogo.com/python/OpenCV_Python/images/mean_shift_tracking/slow_traffic_small.mp4";
    const string keys =
        "{ h help |      | print this help message }"
        "{ @image | vtest.avi | path to image file }";
    CommandLineParser parser(argc, argv, keys);
    parser.about(about);
    if (parser.has("help"))
    {
        parser.printMessage();
        return 0;
    }
    string filename = "video_6.mp4";
    if (!parser.check())
    {
        parser.printErrors();
        return 0;
    }
    VideoCapture capture(filename);
    if (!capture.isOpened()) {
        //error in opening the video input
        cerr << "Unable to open file!" << endl;
        return 0;
    }

    // Create some random colors
    vector<Scalar> colors;
    RNG rng;
    for (int i = 0; i < 100; i++)
    {
        int r = rng.uniform(0, 256);
        int g = rng.uniform(0, 256);
        int b = rng.uniform(0, 256);
        colors.push_back(Scalar(r, g, b));
    }
    Mat old_frame, old_gray;
    vector<Point2f> p0, p1;
    // Take first frame and find corners in it
    capture >> old_frame;
    cvtColor(old_frame, old_gray, COLOR_BGR2GRAY);
    goodFeaturesToTrack(old_gray, p0, 1500, 0.01, 0.3, Mat(), 7, true, 0.04);
    // Create a mask image for drawing purposes
    Mat mask = Mat::zeros(old_frame.size(), old_frame.type());

	// ========================= Open output format context ===================================//
	const AVRational dst_fps = { 30, 1 };
	AVFormatContext* outctx = nullptr;
	int ret = avformat_alloc_output_context2(&outctx, nullptr, nullptr, "video_out.mp4");
	if (ret < 0) {
		cerr << "fail to avformat_alloc_output_context2(video_out.mp4): ret=" << ret;
		return 2;
	}
	// ===================== Find Encoder ======================================= //
	AVCodec* vcodec = avcodec_find_encoder(outctx->oformat->video_codec);
	// ===================== open output IO context ============================= //
	ret = avio_open2(&outctx->pb, "video_out.mp4", AVIO_FLAG_WRITE, nullptr, nullptr);
	if (ret < 0) {
		cerr << "fail to avio_open2: ret=" << ret;
		return 2;
	}
    int w = capture.get(CAP_PROP_FRAME_WIDTH);
    int h = capture.get(CAP_PROP_FRAME_HEIGHT);
	// ======================== Allocate codec context ========================== //
	AVCodecContext* v_codec_ctx = avcodec_alloc_context3(vcodec);
	v_codec_ctx->width = capture.get(CAP_PROP_FRAME_WIDTH);
	v_codec_ctx->height = capture.get(CAP_PROP_FRAME_HEIGHT);
	v_codec_ctx->pix_fmt = *vcodec->pix_fmts;
	v_codec_ctx->time_base = av_inv_q(dst_fps);
	v_codec_ctx->bit_rate = 12350000;
	v_codec_ctx->codec_id = outctx->oformat->video_codec;
	av_opt_set(v_codec_ctx->priv_data, "preset", "ultrafast", 0);
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
	// ======================= Allocate streams ================================== //
	AVStream* vstr = avformat_new_stream(outctx, vcodec);
	avcodec_parameters_from_context(vstr->codecpar, v_codec_ctx);
	cout
		<< "format:  " << outctx->oformat->name << "\n"
		<< "vcodec:  " << vcodec->name << "\n"
		<< "fps:     " << av_q2d(dst_fps) << "\n"
		<< "pixfmt:  " << av_get_pix_fmt_name(v_codec_ctx->pix_fmt) << "\n"
		<< flush;

	// ======================= Initialize sample scaler ========================== //
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
    avformat_write_header(outctx, NULL);

    while (true) {
        Mat frame, frame_gray;
        capture >> frame;
        if (frame.empty())
            break;
        cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
        // calculate optical flow
        vector<uchar> status;
        vector<float> err;
        vector<Point2f> good_new;
        TermCriteria criteria = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 10, 0.03);
        if (p0.size() == 0) {
            old_gray = frame_gray.clone();
            p0 = good_new;
            continue;
        }
        calcOpticalFlowPyrLK(old_gray, frame_gray, p0, p1, status, err, Size(15, 15), 2, criteria);
        for (uint i = 0; i < p0.size(); i++)
        {
            // Select good points
            if (status[i] == 1) {
                good_new.push_back(p1[i]);
                // draw the tracks
                line(mask, p1[i], p0[i], colors[i], 2);
                circle(frame, p1[i], 5, colors[i], -1);
            }
        }
        Mat img, img_resized;
        add(frame, mask, img);
        encodeCVMatByFFmpeg(img, swsctx, v_codec_ctx, outctx, vframe, vstr);
        resize(img, img_resized, Size(640, 360));
        //imshow("Frame", img_resized);
        int keyboard = waitKey(30);
        if (keyboard == 'q' || keyboard == 27)
            break;
        // Now update the previous frame and previous points
        old_gray = frame_gray.clone();
        p0 = good_new;
    }

    capture.release();
    av_write_trailer(outctx);    
    av_frame_free(&vframe);
    avcodec_close(v_codec_ctx);
    avio_close(outctx->pb);
}