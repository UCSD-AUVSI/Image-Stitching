#ifndef DATATYPES_H
#define DATATYPES_H

#include <cv.h>
#include <ostream>
#include "gpc.h"
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

  cv::detail::ExposureCompensator* exposureCompensator = 
    cv::detail::ExposureCompensator::createDefault(cv::detail::ExposureCompensator::GAIN);

  cv::detail::Blender* blender = new cv::detail::FeatherBlender(false);

  cv::detail::BundleAdjusterBase* bundleAdjuster = new cv::detail::BundleAdjusterReproj();
};

struct Pixel {
  int x;
  int y;
  Pixel(int x, int y): x(x),y(y){}

  /**
   * Converts this pixel to an OpenCV Point2f
   */
  cv::Point2f toPoint2f();
  cv::KeyPoint toKeyPoint(double scale);


};

/**
std::ostream& operator<<(std::ostream &strm, Pixel &pixel) {
  return strm << "Pixel(" << pixel.x << ", " << pixel.y <<")";
};
**/

struct LatLon {
  double lat;
  double lon;

  LatLon(gpc_vertex vertex);
  LatLon(double lat, double lon): lat(lat),lon(lon){}
  /**
   * Converts this GPS point to a gpc_vertex
   */
  gpc_vertex toGPCVertex();

  /**
   * Converts this GPS point to an openCV point2i for use as a descriptor
   */
  cv::Point2i toPoint2i();
};

/**
std::ostream& operator<<(std::ostream &strm, const LatLon &latlon) {
  return strm << "LatLon(" << latlon.lat << ", " << latlon.lon <<")";
};
**/

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
  gpc_polygon* toGPCPolygon();

  /**
   * Returns the pixel in the image that is closest to the given point
   */
  Pixel getPixelFor(LatLon latlon);

  cv::detail::CameraParams getCameraParams(double minLat, double minLon) const;
};

struct GPSExtremes {
  double minLat;
  double minLon;
  double maxLat;
  double maxLon;
  GPSExtremes(gpc_polygon* polygon);
  GPSExtremes(double minLat, double minLon, double maxLat, double maxLon):
                minLat(minLat),
                minLon(minLon),
                maxLat(maxLat),
                maxLon(maxLon) {}

};

struct PixelExtremes {
  double minX;
  double minY;
  double maxX;
  double maxY;
  PixelExtremes(double minX, double minY, double maxX, double maxY):
                minX(minX),
                minY(minY),
                maxX(maxX),
                maxY(maxY) {}

};

#endif
