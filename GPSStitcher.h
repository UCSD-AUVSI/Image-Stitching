#ifndef GPS_STITCHER_H
#define GPS_STITCHER_H

#include <cv.h>
#include <opencv2/stitching/stitcher.hpp>

class GPSStitcher : public cv::Stitcher {
public:
  cv::Stitcher::Status gpsStitch(cv::InputArray images,
                                 cv::OutputArray pano,
                                 std::vector<cv::detail::CameraParams> cameras);
  GPSStitcher();
  Stitcher::Status gpsComposePanorama(cv::InputArray images, cv::OutputArray pano);
};

#endif
