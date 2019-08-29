// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.


#include <string>
#include <sstream>
#include <chrono>
#include <iomanip>
#include <array>

#include "rosbag/bag.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Image.h"
#include "diagnostic_msgs/KeyValue.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "realsense_msgs/StreamInfo.h"
#include "realsense_msgs/ImuIntrinsic.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/TimeReference.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Accel.h"
#include "realsense_legacy_msgs/legacy_headers.h"

//    struct tmpstringstream
//    {
//        std::ostringstream ss;
//        template<class T> tmpstringstream & operator << (const T & val) { ss << val; return *this; }
//        operator std::string() const { return ss.str(); }
//    };

    inline std::string pretty_time(std::chrono::nanoseconds d);


    // Streamers


    template <typename Container>
    std::ostream& print_container(std::ostream& os, const Container& c);

    template <size_t N, typename T>
    std::ostream& operator<<(std::ostream& os, const std::array<T, N>& arr);

    template <typename T>
    std::ostream& operator<<(std::ostream& os, const std::vector<T>& v);

    std::ostream& operator<<(std::ostream& os, rosbag::compression::CompressionType c);

    template <typename ROS_TYPE>
    inline typename ROS_TYPE::ConstPtr try_instantiate(const rosbag::MessageInstance& m);
    
    std::ostream& operator<<(std::ostream& os, const rosbag::MessageInstance& m);


