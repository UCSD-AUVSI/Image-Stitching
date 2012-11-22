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

  /**
   * Converts this GPS point to an openCV point2i for use as a descriptor
   */
  cv::Point2i toPoint2i(){
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
  gpc_polygon* toGPCPolygon();

  /**
   * Returns the pixel in the image that is closest to the given point
   */
  Pixel getPixelFor(LatLon latlon);
  }
};

struct GPSExtremes {
  double minLat;
  double minLon;
  double maxLat;
  double maxLon;
  GPSExtremes(gpc_polygon polygon);
  GPSExtremes(double minLat, double minLon, double maxLat, double maxLon):
                minLat(minLat),
                minLon(minLon),
                maxLat(maxLat),
                maxLon(maxLon) {}

};


#endif
