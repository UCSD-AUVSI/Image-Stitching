#ifndef GPS_BUNDLE_ADJUSTER
#define GPS_BUNDLE_ADJUSTER

#include <cv.h>
#include <opencv2/stitching/stitcher.hpp>
#include <vector>
#include <iostream>

class GPSBundleAdjuster: public cv::detail::BundleAdjusterBase{

public:

  GPSBundleAdjuster(const std::vector<cv::detail::CameraParams> cameras): gpsCameras(cameras),
                                                                     BundleAdjusterBase(0,0) {}
  
protected:

  virtual void setUpInitialCameraParams(const std::vector<cv::detail::CameraParams>& cameras) 
  override;

  virtual void obtainRefinedCameraParams(std::vector<cv::detail::CameraParams>& cameras) const 
  override;

  virtual void calcError(cv::Mat&) {}

  virtual void calcJacobian(cv::Mat& ) {}

private:

  std::vector<cv::detail::CameraParams> gpsCameras;

};

#endif
