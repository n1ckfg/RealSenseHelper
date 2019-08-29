//MIT License
//
//Copyright (c) 2018 Marco Visentini-Scarzanella
//
//Permission is hereby granted, free of charge, to any person obtaining a copy
//of this software and associated documentation files (the "Software"), to deal
//in the Software without restriction, including without limitation the rights
//to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//copies of the Software, and to permit persons to whom the Software is
//furnished to do so, subject to the following conditions:
//
//The above copyright notice and this permission notice shall be included in all
//copies or substantial portions of the Software.
//
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//SOFTWARE.


#ifndef CRealSenseHelper_hpp
#define CRealSenseHelper_hpp

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <thread>
#include <chrono>

//Intel Realsense
#include <librealsense2/rs.hpp>
#include "print_helpers.h"
//OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
//Glog
#include <glog/logging.h>
//rosbag
#include "rosbag/bag.h"
#include "rosbag/view.h"


class CRealSenseHelper {
public:
    CRealSenseHelper();
    ~CRealSenseHelper();
    
    void startStreaming(const std::string &bagFile);
    
    void stopStreaming();
    bool nextFrame(cv::Mat &rgbFrame, cv::Mat &depthFrame, int &rgbFrameNo, int &depthFrameNo, long long &rgbTOA, long long &depthTOA);
    
    cv::Matx33f getRGBIntrinsics() {return _rgbInfo.K;};
    cv::Matx33f getDepthIntrinsics() {return _depthInfo.K;};
    cv::Size getRGBResolution() {return _rgbInfo.resolution;};
    cv::Size getDepthResolution() {return _depthInfo.resolution;};
    
private:
    
//----------- Structures and enums ---------------//
    typedef struct sensorIntrinsics {
        cv::Matx33f K;
        float d[5];
        cv::Size resolution;
        float scale;
    } sensorIntrinsics;
    
    typedef struct sensorExtrinsics {
        cv::Matx33f R;
        cv::Matx31f t;
    } sensorExtrinsics;
    
    typedef struct sensorView {
        rosbag::View meta;
        rosbag::View img;
        rosbag::View::iterator meta_iter;
        rosbag::View::iterator img_iter;
        int nMetadata;
    } sensorView;
    
    enum openMode{
        NOT_OPEN,
        LIVE,
        BAG
    };
    
//----------- Member functions ---------------//
    bool nextFrameFromFile(cv::Mat &rgbFrame, cv::Mat &depthFrame, int &rgbFrameNo, int &depthFrameNo, long long &rgbTOA, long long &depthTOA);
    
    //camera paramters setters
    sensorIntrinsics setStreamIntrinsics(const rs2::video_stream_profile &s);
    sensorIntrinsics setStreamIntrinsics(const rosbag::MessageInstance &m);
    sensorExtrinsics setStreamExtrinsics(const rs2::stream_profile &a, const rs2::stream_profile &b);
    sensorExtrinsics setStreamExtrinsics(const rosbag::MessageInstance &m);
    
    //geometry functions
    cv::Point2i project(cv::Matx31f pt, cv::Matx33f K);
    cv::Matx31f backproject(cv::Point2f x, cv::Matx33f K, float depth);
    void alignDepthFrame(const cv::Mat &depth, cv::Mat &depthAligned);
    
    //image conversions
    cv::Mat frameToMat(const rs2::frame &f);
    cv::Mat frameToMat(const rosbag::MessageInstance &m);

//----------- Member variables ---------------//

    openMode _streamMode;
    
    //camera parameters
    sensorIntrinsics _rgbInfo;
    sensorIntrinsics _depthInfo;
    sensorExtrinsics _d2rgb;
    
    //structures for bagfile reading
    rosbag::Bag _bag;
    sensorView _rgbBagStream;
    sensorView _depthBagStream;
};
#endif /* CRealSenseHelper_hpp */
