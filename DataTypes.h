#ifndef DATATYPES_H
#define DATATYPES_H
#include <cv.h>
#include "gpc.h"

struct Pixel {
  int x;
  int y;
  Pixel(int x, int y): x(x),y(y){}

  /**
   * Converts this pixel to an OpenCV Point2f
   */
  cv::Point2f toPoint2f();
};

struct LatLon {
  double lat;
  double lon;
  LatLon(gpc_vertex vertex);
  LatLon(double lat, double lon): lat(lat),lon(lon){}
  /**
   * Converts this GPS point to a gpc_vertex
   */
  gpc_vertex toGPCVertex();
};

#endif
