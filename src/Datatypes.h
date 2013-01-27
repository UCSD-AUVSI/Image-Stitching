#ifndef DATATYPES_H
#define DATATYPES_H

#include <cv.h>
#include <ostream>
#include <string>
#include "AdjacentFeaturesMatcher.h"
#include <opencv2/stitching/stitcher.hpp>
#include <opencv2/stitching/warpers.hpp>

struct GPSStitcherArgs{

  double registrationResolution = 0.5;
  double seamEstimationResolution = 0.01;
  double compositingResolution = 0.5;
  double confidenceThreshold = 0.4;

  bool doWaveCorrect = false;
  bool doBundleAdjust = true;
  bool useFeatures = true;

  cv::detail::FeaturesMatcher* featuresMatcher = new AdjacentFeaturesMatcher();
  cv::detail::FeaturesFinder* featuresFinder = new cv::detail::SurfFeaturesFinder(1000);
  cv::WarperCreator* warperCreator = new cv::PlaneWarper();

  cv::detail::SeamFinder* seamFinder =
    new cv::detail::GraphCutSeamFinder(cv::detail::GraphCutSeamFinderBase::COST_COLOR);

  cv::detail::ExposureCompensator* exposureCompensator = new cv::detail::NoExposureCompensator();

  cv::detail::Blender* blender = new cv::detail::MultiBandBlender(false);

  cv::detail::BundleAdjusterBase* bundleAdjuster = new cv::detail::BundleAdjusterReproj();
};

struct CameraArgs {
  double groundLevel, latScale, lonScale, altScale, focalLength;
  std::string telemOrder;
};

struct ImageWithPlaneData {
  cv::Mat image;
  double latitude;
  double longitude;
  double altitude;
  double roll;
  double pitch;
  double yaw;
  double gimbalRoll;
  double gimbalPitch;
  ImageWithPlaneData(){}
  ImageWithPlaneData( cv::Mat image, double latitude, double longitude,
                      double altitude, double roll, double pitch, double yaw, 
                      double gimbalRoll, double gimbalPitch):
                        image(image),
                        latitude(latitude),
                        longitude(longitude),
                        altitude(altitude),
                        roll(roll),
                        pitch(pitch),
                        yaw(yaw),
                        gimbalRoll(gimbalRoll),
                        gimbalPitch(gimbalPitch){}

  cv::detail::CameraParams getCameraParams(double minLat, double minLon, CameraArgs args) const;
};

#endif
