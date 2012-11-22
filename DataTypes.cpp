#include <cv.h>
#include "DataTypes.h"
#include "gpc.h"

cv::Point2f Pixel::toPoint2f(){
  return cv::Point2f(x,y);
}

gpc_vertex LatLon::toGPCVertex(){
  gpc_vertex vertex;
  vertex.x = lat;
  vertex.y = lon;
  return vertex;
}

LatLon::LatLon(gpc_vertex vertex){
  lat = vertex.x;
  lon = vertex.y;
}
