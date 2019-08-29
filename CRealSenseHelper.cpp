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


#include "CRealSenseHelper.hpp"


CRealSenseHelper::CRealSenseHelper() {
    _streamMode = NOT_OPEN;
}


CRealSenseHelper::~CRealSenseHelper() {
    if (_streamMode == BAG)
        stopStreaming();
}

void CRealSenseHelper::stopStreaming() {
    if (_streamMode == BAG)
        _bag.close();
}

void CRealSenseHelper::startStreaming(const std::string &bagFile) {
    _bag.open(bagFile);
    _streamMode = BAG;
    
    //set intrinsics + extrinsics
    {
        rosbag::View bagView(_bag, rosbag::TopicQuery("/device_0/sensor_0/Depth_0/info/camera_info"));
        _depthInfo = setStreamIntrinsics(*bagView.begin());
    }
    {
        //set scale of depth units
        rosbag::View bagView(_bag, rosbag::TopicQuery("/device_0/sensor_0/option/Depth Units/value"));
        auto data = try_instantiate<std_msgs::Float32>(*bagView.begin());
        _depthInfo.scale = data->data;
    }
    {
        rosbag::View bagView(_bag, rosbag::TopicQuery("/device_0/sensor_1/Color_0/info/camera_info"));
        _rgbInfo = setStreamIntrinsics(*bagView.begin());
    }
    {
        rosbag::View bagView(_bag, rosbag::TopicQuery("/device_0/sensor_1/Color_0/tf/0"));
        _d2rgb = setStreamExtrinsics(*bagView.begin());
    }
    
    
    //set image data
    _depthBagStream.img.addQuery(_bag, rosbag::TopicQuery("/device_0/sensor_0/Depth_0/image/data"));
    _depthBagStream.meta.addQuery(_bag,rosbag::TopicQuery("/device_0/sensor_0/Depth_0/image/metadata"));
    _rgbBagStream.img.addQuery(_bag, rosbag::TopicQuery("/device_0/sensor_1/Color_0/image/data"));
    _rgbBagStream.meta.addQuery(_bag, rosbag::TopicQuery("/device_0/sensor_1/Color_0/image/metadata"));
    
    //initialise iterators
    _depthBagStream.img_iter = _depthBagStream.img.begin();
    _depthBagStream.meta_iter = _depthBagStream.meta.begin();
    _rgbBagStream.img_iter = _rgbBagStream.img.begin();
    _rgbBagStream.meta_iter = _rgbBagStream.meta.begin();
    _depthBagStream.nMetadata = _depthBagStream.meta.size()/_depthBagStream.img.size();
    _rgbBagStream.nMetadata = _rgbBagStream.meta.size()/_rgbBagStream.img.size();
}

bool CRealSenseHelper::nextFrame(cv::Mat &rgbFrame, cv::Mat &depthFrame, int &rgbFrameNo, int &depthFrameNo, long long &rgbTOA, long long &depthTOA) {
    
    bool success = false;
    switch (_streamMode) {
        case BAG:
            success = nextFrameFromFile(rgbFrame, depthFrame, rgbFrameNo, depthFrameNo, rgbTOA, depthTOA);
            break;
        case LIVE:
            break;
        case NOT_OPEN:
            break;
        default:
            break;
    }
    
    return success;
}

bool CRealSenseHelper::nextFrameFromFile(cv::Mat &rgbFrame, cv::Mat &depthFrame, int &rgbFrameNo, int &depthFrameNo, long long &rgbTOA, long long &depthTOA) {
    bool success = false;
    if (_rgbBagStream.img_iter != _rgbBagStream.img.end()) {
        //retrieve rgb image
        auto rgbData = try_instantiate<sensor_msgs::Image>(*_rgbBagStream.img_iter);
        ros::Time rgbStamp = rgbData->header.stamp;
        
        //retrieve depth image
        auto depthData = try_instantiate<sensor_msgs::Image>(*_depthBagStream.img_iter);
        ros::Time depthStamp = depthData->header.stamp;
        
        double diff = (rgbStamp - depthStamp).toSec();
        
        //if depth image was acquired before rgb image, advance until a matching frame is found
        if (diff >= 0) {
            rosbag::View::iterator currDepthIterator = _depthBagStream.img_iter;
            int advanceBy = -1;
            double bestTimeDiff = fabs(diff)+1;
            while (_depthBagStream.img_iter != _depthBagStream.img.end()) {
                auto data = try_instantiate<sensor_msgs::Image>(*_depthBagStream.img_iter);
                ros::Time stamp = data->header.stamp;
                double timeDiff = fabs((stamp-rgbStamp).toSec());
                if (timeDiff < bestTimeDiff) {
                    bestTimeDiff = timeDiff;
                    advanceBy++;
                    _depthBagStream.img_iter++;
                } else {
                    break;
                }
            }
            
            //advance depth and depth metadata iterators
            _depthBagStream.img_iter = currDepthIterator;
            for (int i = 0; i < advanceBy; i++) {
                _depthBagStream.img_iter++;
                for (int j = 0; j < _depthBagStream.nMetadata; j++) {
                    _depthBagStream.meta_iter++;
                }
            }
        }
        //if rgb image was acquired before depth image, advance until a matching frame is found
        else {
            rosbag::View::iterator currRgbIterator = _rgbBagStream.img_iter;
            int advanceBy = -1;
            double bestTimeDiff = fabs(diff)+1;
            while (_rgbBagStream.img_iter != _rgbBagStream.img.end()) {
                auto data = try_instantiate<sensor_msgs::Image>(*_rgbBagStream.img_iter);
                ros::Time stamp = data->header.stamp;
                double timeDiff = fabs((stamp-depthStamp).toSec());
                if (timeDiff < bestTimeDiff) {
                    bestTimeDiff = timeDiff;
                    advanceBy++;
                    _rgbBagStream.img_iter++;
                } else {
                    break;
                }
            }
            
            //advance depth and depth metadata iterators
            _rgbBagStream.img_iter = currRgbIterator;
            for (int i = 0; i < advanceBy; i++) {
                _rgbBagStream.img_iter++;
                for (int j = 0; j < _rgbBagStream.nMetadata; j++) {
                    _rgbBagStream.meta_iter++;
                }
            }
        }
        
        //retrieve rgb metadata
        for (int i = 0; i < _rgbBagStream.nMetadata; i++) {
            auto data = try_instantiate<diagnostic_msgs::KeyValue>(*_rgbBagStream.meta_iter);
            //save frame number
            if (data->key == "Frame Counter") {
                rgbFrameNo = std::stoi(data->value);
            }
            if (data->key == "Time Of Arrival") {
                rgbTOA = std::stoll(data->value);
            }
            _rgbBagStream.meta_iter++;
        }
        
        //retrieve depth metadata
        for (int i = 0; i < _depthBagStream.nMetadata; i++) {
            auto data = try_instantiate<diagnostic_msgs::KeyValue>(*_depthBagStream.meta_iter);
            //save frame number
            if (data->key == "Frame Counter") {
                depthFrameNo = std::stoi(data->value);
            }
            if (data->key == "Time Of Arrival") {
                depthTOA = std::stoll(data->value);
            }
            _depthBagStream.meta_iter++;
        }
        
        //get frames
        rgbFrame = frameToMat(*_rgbBagStream.img_iter).clone();
        
        cv::Mat depth = frameToMat(*_depthBagStream.img_iter);
        alignDepthFrame(depth, depthFrame);
        //advance iterators
        _rgbBagStream.img_iter++;
        _depthBagStream.img_iter++;
        
        success = true;
    }
    return success;
}

cv::Mat CRealSenseHelper::frameToMat(const rosbag::MessageInstance &m) {
    auto data = try_instantiate<sensor_msgs::Image>(m);
    if (data->encoding == "rgb8") {
        std::vector<uchar> matData = data->data;
        cv::Mat rgb(_rgbInfo.resolution.height, _rgbInfo.resolution.width, CV_8UC3, matData.data());
        cv::cvtColor(rgb, rgb, cv::COLOR_BGR2RGB);
        return rgb;
    } else if (data->encoding == "bgr8") {
        std::vector<uchar> matData = data->data;
        cv::Mat rgb(_rgbInfo.resolution.height, _rgbInfo.resolution.width, CV_8UC3, matData.data());
        return rgb;
    } else if (data->encoding == "mono16") {
        cv::Mat z16(_depthInfo.resolution.height, _depthInfo.resolution.width, CV_16UC1, std::vector<uchar>(data->data).data());
        return z16;
    }
    
    LOG(FATAL) << "Frame format is not supported yet!";
}

cv::Mat CRealSenseHelper::frameToMat(const rs2::frame& f)
{
    auto vf = f.as<rs2::video_frame>();
    const int w = vf.get_width();
    const int h = vf.get_height();
    
    if (f.get_profile().format() == RS2_FORMAT_BGR8)
    {
        return cv::Mat(cv::Size(w, h), CV_8UC3, (void*)f.get_data(), cv::Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_RGB8)
    {
        auto r = cv::Mat(cv::Size(w, h), CV_8UC3, (void*)f.get_data(), cv::Mat::AUTO_STEP);
        cv::cvtColor(r, r, cv::COLOR_BGR2RGB);
        return r;
    }
    else if (f.get_profile().format() == RS2_FORMAT_Z16)
    {
        return cv::Mat(cv::Size(w, h), CV_16UC1, (void*)f.get_data(), cv::Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_Y8)
    {
        return cv::Mat(cv::Size(w, h), CV_8UC1, (void*)f.get_data(), cv::Mat::AUTO_STEP);;
    }
    
    LOG(FATAL) << "Frame format is not supported yet!";
}

CRealSenseHelper::sensorIntrinsics CRealSenseHelper::setStreamIntrinsics(const rosbag::MessageInstance& m) {
    
    sensorIntrinsics out;
    cv::Matx33f K = cv::Matx33f::eye();
    auto data = try_instantiate<sensor_msgs::CameraInfo>(m);
    out.resolution = cv::Size(data->width, data->height);
    K.val[0] = data->K[0];
    K.val[2] = data->K[2];
    K.val[4] = data->K[4];
    K.val[5] = data->K[5];
    out.K = K;
    out.d[0] = data->D[0];
    out.d[1] = data->D[1];
    out.d[2] = data->D[2];
    out.d[3] = data->D[3];
    out.d[4] = data->D[4];
    
    return out;
}

CRealSenseHelper::sensorIntrinsics CRealSenseHelper::setStreamIntrinsics(const rs2::video_stream_profile &s) {
    sensorIntrinsics out;
    cv::Matx33f K = cv::Matx33f::eye();
    rs2_intrinsics intrinsics = s.get_intrinsics();
    K.val[0] = intrinsics.fx;
    K.val[2] = intrinsics.ppx;
    K.val[4] = intrinsics.fy;
    K.val[5] = intrinsics.ppy;
    out.K = K;
    for (int i =0; i < 5; i++) {
        out.d[i] = intrinsics.coeffs[i];
    }
    out.resolution = cv::Size(intrinsics.width, intrinsics.height);
    return out;
}

CRealSenseHelper::sensorExtrinsics CRealSenseHelper::setStreamExtrinsics(const rosbag::MessageInstance& m) {
    sensorExtrinsics out;
    auto data = try_instantiate<geometry_msgs::Transform>(m);
    
    cv::Matx33f R;
    float s, v1, v2, v3;
    s = data->rotation.w;
    v1 = data->rotation.x;
    v2 = data->rotation.y;
    v3 = data->rotation.z;
    
    float invnorm = 1.0 / sqrt( s*s + v1*v1 + v2*v2 + v3*v3 );
    s *= invnorm; v1 *= invnorm; v2 *= invnorm; v3 *= invnorm;
    R.val[0]  = s*s + v1*v1 - v2*v2 - v3*v3;
    R.val[1] = 2*( v1*v2 - s*v3 );
    R.val[2] = 2*( v1*v3 + s*v2 );
    R.val[3] = 2*( v1*v2 + s*v3 );
    R.val[4] = s*s - v1*v1 + v2*v2 - v3*v3;
    R.val[5] = 2*( v2*v3 - s*v1 );
    R.val[6] = 2*( v1*v3 - s*v2 );
    R.val[7] = 2*( v2*v3 + s*v1 );
    R.val[8] = s*s - v1*v1 - v2*v2 + v3*v3;

    cv::Matx31f t;
    t.val[0] = data->translation.x;
    t.val[1] = data->translation.y;
    t.val[2] = data->translation.z;
    
    out.R = R.t();
    out.t = t;
    
    return out;
}

CRealSenseHelper::sensorExtrinsics CRealSenseHelper::setStreamExtrinsics(const rs2::stream_profile &a, const rs2::stream_profile &b) {
    sensorExtrinsics out;
    rs2_extrinsics ext = a.get_extrinsics_to(b);
    out.t = cv::Matx31f(ext.translation[0], ext.translation[1], ext.translation[2]);
    out.R = cv::Matx33f(ext.rotation[0], ext.rotation[1], ext.rotation[2] ,ext.rotation[3] ,ext.rotation[4], ext.rotation[5], ext.rotation[6], ext.rotation[7], ext.rotation[8]);
    
    return out;
}

cv::Point2i CRealSenseHelper::project(cv::Matx31f pt, cv::Matx33f K) {
    cv::Point2f m = cv::Point2f(pt.val[0]/pt.val[2],pt.val[1]/pt.val[2]);
    cv::Point2f mi;
    mi.x = round(K.val[0]*m.x + K.val[2]);
    mi.y = round(K.val[4]*m.y + K.val[5]);
    return mi;
}

cv::Matx31f CRealSenseHelper::backproject(cv::Point2f x, cv::Matx33f K, float depth) {
    cv::Matx31f m = depth*cv::Matx31f( (x.x - K.val[2])/K.val[0],(x.y - K.val[5])/K.val[4],1);
    return m;
}

void CRealSenseHelper::alignDepthFrame(const cv::Mat &depth, cv::Mat &depthAligned) {
    
    depthAligned = cv::Mat(depth.rows, depth.cols, CV_32FC1);
    depthAligned.setTo(0);
    for (int i = 0; i < depth.rows; i++) {
        for (int j = 0; j < depth.cols; j++) {
            float d = _depthInfo.scale*depth.ptr<ushort>(i)[j];
            if (d != 0) {
                cv::Matx31f pt3d = _d2rgb.R.t()*backproject(cv::Point2f(j,i), _depthInfo.K, d)+_d2rgb.t;
                cv::Point2i pt2d = project(pt3d, _rgbInfo.K);
                if ((pt2d.x >= 0) && (pt2d.x < depth.cols) && (pt2d.y >= 0) && (pt2d.y < depth.rows))
                    depthAligned.ptr<float>(pt2d.y)[pt2d.x] = d;
            }
        }
    }
}

