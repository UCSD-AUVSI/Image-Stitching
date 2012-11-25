#include <cv.h>
#include "DataTypes.h"
#include "gpc.h"
#include "GeoReference.h"
#include <iostream>
using namespace std;

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

cv::Point2i LatLon::toPoint2i(){
  return cv::Point2i(lat * 1000,lon*1000);
}

GPSExtremes::GPSExtremes(gpc_polygon* polygon){
    if (polygon->num_contours == 0 ){
      throw "Cannot calculate extremes for an empty polygon.";
    }
    gpc_vertex* vertices = polygon->contour->vertex;
    int numVertices = polygon->contour->num_vertices;
	
	minLat = INT_MAX;
	minLon = INT_MAX;
	maxLat = INT_MIN;
	maxLon = INT_MIN;

	for(int i = 0; i < numVertices; i++){
		if (vertices[i].y < minLon ) minLon = vertices[i].y;
		if (vertices[i].y > maxLon ) maxLon = vertices[i].y;
		if (vertices[i].x < minLat ) minLat = vertices[i].x;
	    if (vertices[i].x > maxLat ) maxLat = vertices[i].x;
	}
}

//TODO: GeoReferencing
Pixel ImageWithPlaneData::getPixelFor(LatLon latlon){
  GPSExtremes extremes(this->toGPCPolygon());
  double dLat = extremes.maxLat - extremes.minLat;
  double dLon = extremes.maxLon - extremes.minLon;
  double latPart = (latlon.lat - extremes.minLat) / dLat;
  double lonPart = (latlon.lon - extremes.minLon) / dLon;
  double x = image.cols * lonPart;
  double y = image.rows * latPart;
  return Pixel(x,y);
}

cv::KeyPoint Pixel::toKeyPoint(double scale){
  return cv::KeyPoint((float)x/(float)scale,(float)y/(float)scale,100.0);
}

gpc_polygon* ImageWithPlaneData::toGPCPolygon(){
  double vertexLatitude,vertexLongitude,vertexAltitude; 

  Vision::GeoReference::forwardGeoreferencing(latitude,
                                      longitude,
                                      altitude,
                                      roll,
                                      pitch,
                                      yaw,
                                      gimbalRoll,
                                      gimbalPitch,
                                      0, // gimbal yaw,
                                      0, // targetX
                                      0, // targetY
                                      image.cols,
                                      image.rows,
                                      1.0, // zoom,
                                      vertexLatitude,
                                      vertexLongitude,
                                      vertexAltitude);
  LatLon bottomLeftPoint = LatLon(vertexLatitude,vertexLongitude);

  Vision::GeoReference::forwardGeoreferencing(latitude,
                                      longitude,
                                      altitude,
                                      roll,
                                      pitch,
                                      yaw,
                                      gimbalRoll,
                                      gimbalPitch,
                                      0, // gimbal yaw,
                                      image.cols-1, // targetX
                                      0, // targetY
                                      image.cols,
                                      image.rows,
                                      1.0, // zoom,
                                      vertexLatitude,
                                      vertexLongitude,
                                      vertexAltitude);
  LatLon bottomRightPoint = LatLon(vertexLatitude,vertexLongitude);

  Vision::GeoReference::forwardGeoreferencing(latitude,
                                      longitude,
                                      altitude,
                                      roll,
                                      pitch,
                                      yaw,
                                      gimbalRoll,
                                      gimbalPitch,
                                      0, // gimbal yaw,
                                      image.cols-1, // targetX
                                      image.rows-1, // targetY
                                      image.cols,
                                      image.rows,
                                      1.0, // zoom,
                                      vertexLatitude,
                                      vertexLongitude,
                                      vertexAltitude);
  LatLon topRightPoint = LatLon(vertexLatitude,vertexLongitude);

  Vision::GeoReference::forwardGeoreferencing(latitude,
                                      longitude,
                                      altitude,
                                      roll,
                                      pitch,
                                      yaw,
                                      gimbalRoll,
                                      gimbalPitch,
                                      0, // gimbal yaw,
                                      0, // targetX
                                      image.rows-1, // targetY
                                      image.cols,
                                      image.rows,
                                      1.0, // zoom,
                                      vertexLatitude,
                                      vertexLongitude,
                                      vertexAltitude);
  LatLon topLeftPoint = LatLon(vertexLatitude,vertexLongitude);

  /**
    cout <<"bottomLeftPoint: ("<<bottomLeftPoint.lat<<", "<<bottomLeftPoint.lon<<")\n";
    cout <<"topLeftPoint: ("<<topLeftPoint.lat<<", "<<topLeftPoint.lon<<")\n";
    cout <<"bottomRightPoint: ("<<bottomRightPoint.lat<<", "<<bottomRightPoint.lon<<")\n";
    cout <<"topRightPoint: ("<<topRightPoint.lat<<", "<<topRightPoint.lon<<")\n";
  **/

  gpc_vertex topLeftVertex = topLeftPoint.toGPCVertex();
  gpc_vertex topRightVertex = topRightPoint.toGPCVertex();
  gpc_vertex bottomRightVertex = bottomRightPoint.toGPCVertex();
  gpc_vertex bottomLeftVertex = bottomLeftPoint.toGPCVertex();

  gpc_vertex* vertices = (gpc_vertex*)malloc(sizeof(gpc_vertex) * 4);

  vertices[0] = topLeftVertex;
  vertices[1] = topRightVertex;
  vertices[2] = bottomRightVertex;
  vertices[3] = bottomLeftVertex;

  gpc_vertex_list* list = new gpc_vertex_list();
  list->num_vertices=4;
  list->vertex = vertices;
  gpc_polygon* polygon = new gpc_polygon();
  polygon->num_contours = 1;
  polygon->hole=0;
  polygon->contour = list;

  return polygon;
}
