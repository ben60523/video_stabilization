# Video Stabilization

__This repository provides a way to post-process media file to decrease the vibration of the video which is caused by other external factors(e.g. hands).__

__This repository is Microsoft Visual Studio Project and consist of two parts.__ 

***One is to show the optical flow of each frames.***

***The other is to estimate the motion of camera and smooth it, and then genrate the media file which each frame is tranformed by the motion.***

:::info
___Unzip ffmpeg_opencv_lib_h.7z before compile___
:::

## Structure
```
|
+-- ffmpeg (the library and header files of FFmpeg)
|    |___ lib
|    |___ include
|
+-- opencv (the library and header files of OpenCV)
|    |___ lib
|    |___ include
|
+-- optical_flow (the code files of optical flow)
|    |___ optical_flow.cpp (the main code is here)
|    |___  optical_flow.vcxproj/optical_flow.vcxproj.filter
|    |    (the projects configures)
|    |___ stdafx.h (pre-compiled header files)  
|
+-- video_stabilization (the code files of video stabilization)
     |___ ffmpeg_cv_utils.cpp/hpp (some self-defined functions and structures)
     |___ video_stabilization (the main code is here)
     |___ video_stabilization.vcxproj/.vcxproj.filter
     |    (the projects configures)
     |___ stdafx.h (pre-compiled header files)  
```

## Optical Flow

* The sample code which is provided by OpenCV. More info, please refer [here](https://docs.opencv.org/3.4/d4/dee/tutorial_optical_flow.html)

## Video Stabilization

* The concept of the algorithms is inspired by [here](https://learnopencv.com/video-stabilization-using-point-feature-matching-in-opencv/)
    * Brief Concept 
        1. Use corner detection by the minimal eigen-value of gradient to get the corners we want to track.
        2. Track the motions of these corners with Lucas-Kanade optical flow algorithm.
        3. To sum the motions vector to get the uniform motion of the frame.
        4. Smooth the motion with moving-average filter.
        5. Transform the frame with affine model with the motion.
    * The input media file is read by OpenCV and the the results of affine transformation of every frames are stored.
    * We need to read input again with FFmpeg library and decode every frames because the media which is read by OpenCV is lack of audio and exist more limitaion about writing output file(e.g. size, bit-rate and format).
        * Get video/audio frames of input media after decoding
            * If video frames, Convert `AVFrame` into `cv::Mat`, apply the tranformation and fix the border of the frame and finally encode it with FFmpeg and write to the output media files.
            * If audio frames, Just encoding and write to the output media files.