#pragma once
#include "linux/videodev2.h"

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>



#include "opencv2/opencv.hpp"

#include <iostream>
class RMVideoCapture {
public:
    RMVideoCapture(const char * device, int size_buffer = 1);
    ~RMVideoCapture();
    bool startStream();
    bool closeStream();
    bool setExposureTime(bool auto_exp, int t);
    bool setVideoFormat(int width, int height, bool mjpg = 1);
    bool changeVideoFormat(int width, int height, bool mjpg = 1);
    bool getVideoSize(int & width, int & height);

    bool setVideoFPS(int fps);
    bool setBufferSize(int bsize);
    void restartCapture();
    int getFrameCount(){
        return cur_frame;
    }
    int getVideoDevice();
    void info();
    void getDefaultSetting();
    void getCurrentSetting();
    void setpara();
   
    struct CAM_PARA
    {
        int gain;
        int exposure;
        int brightness;
        int whiteness;
        int saturation;
        int contrast;
    }cam_para;

    RMVideoCapture& operator >> (cv::Mat & image);

private:
    void cvtRaw2Mat(const void * data, cv::Mat & image);
    bool refreshVideoFormat();
    bool initMMap();
    int xioctl(int fd, int request, void *arg);

private:
    struct MapBuffer {
        void * ptr;
        unsigned int size;
    };
    unsigned int capture_width;
    unsigned int capture_height;
    unsigned int format;
    int fd;
    unsigned int buffer_size;
    unsigned int buffr_idx;
    unsigned int cur_frame;
    MapBuffer * mb;
    const char * video_path;
};





