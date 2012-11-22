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

LatLon::toPoint2i(){
  return cv::Point2i(lat * 1000,lon*1000);
}

GPSExtremes::GPSExtremes(gpc_polygon polygon){
    gpc_vertex* vertices = polygon.contour->vertex;
	
	minLat = INT_MAX;
	minLon = INT_MAX;
	maxLat = INT_MIN;
	maxLon = INT_MIN;

	for(int i = 0; i < 4; i++){
		if (vertices[i].y < minLon ) minLon = vertices[i].y;
		if (vertices[i].y > maxLon ) maxLon = vertices[i].y;
		if (vertices[i].x < minLat ) minLat = vertices[i].x;
	    if (vertices[i].x > maxLat ) maxLat = vertices[i].x;
	}
}

//TODO: GeoReferencing
Pixel ImageWithPlaneData::getPixelFor(LatLon latlon){
  
}
