#include "GPSBundleAdjuster.h"
#include <vector>
#include <iostream>

using namespace cv::detail;
using namespace std;

void GPSBundleAdjuster::setUpInitialCameraParams(const vector<CameraParams>& cameras) {
    BundleAdjusterBase::cam_params_.create(0,0,CV_64F);
    cerr << "In setUpInitialCameraParams\n";
}

void GPSBundleAdjuster::obtainRefinedCameraParams(vector<CameraParams>& cameras) const {
    cerr << "In obtainRefinedCameraParams\n";
    throw "Woah";
    cameras = gpsCameras;
}
