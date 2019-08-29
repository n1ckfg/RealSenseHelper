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


#include <iostream>
#include "CRealSenseHelper.hpp"

int main() {
    std::string outFolder = "/Users/Documents/Data/Test/";
    
    CRealSenseHelper rs;
    rs.startStreaming("/Users/Documents/20180726_122631.bag");

    cv::FileStorage depthOut;
    cv::Mat rgbFrame, depthFrame;
    int rgbFrameNo, depthFrameNo;
    long long rgbTOA, depthTOA;
    while(true) {
        //get next frame if there are any left
        bool ok = rs.nextFrame(rgbFrame, depthFrame, rgbFrameNo, depthFrameNo, rgbTOA, depthTOA);
        if (!ok)
            break;
        
        //output frame number
        std::cout << "Depth frame number: " << depthFrameNo << " RGB frame number: " << rgbFrameNo << std::endl;
        
        //output system timestamp
        std::cout << "Depth timestamp: " << depthTOA << " RGB timestamp: " << rgbTOA << std::endl;
        
        //write rgb frame
        cv::imwrite(std::string(outFolder) + std::to_string(rgbFrameNo) + "_rgb.png", rgbFrame);
        
        //write depth frame
        depthOut.open(std::string(outFolder) + std::to_string(depthFrameNo) + "_depth.xml", cv::FileStorage::WRITE);
        depthOut << "Depth" << depthFrame;
        depthOut.release();
    }
    return 0;
}
