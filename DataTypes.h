#ifndef DATATYPES_H
#define DATATYPES_H

#include <cv.h>
#include <ostream>
#include "gpc.h"
#include <opencv2/stitching/stitcher.hpp>

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

  cv::detail::CameraParams getCameraParams() const;
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
