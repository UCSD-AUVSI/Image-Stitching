#include <cv.h>
#include "DataTypes.h"
#include "gpc.h"
#include "camera.h"
#include "GeoReference.h"
#include <iostream>
#include <cmath>

using namespace Vision;
using namespace std;
using namespace cv::detail;

float sinDegrees(float degrees){
  return sin(2.0f * M_PI * degrees / 360.0);
}

float cosDegrees(float degrees){
  return cos(2.0f * M_PI * degrees / 360.0);
}

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

cv::detail::CameraParams ImageWithPlaneData::getCameraParams(double minLat,
                                                             double minLon) const {
  assert(!image.empty());
  CameraParams cParams;

  cParams.focal = 500; //CAMERA_FOCAL_MM * 72.0 / 25.4;
  cParams.aspect = (double)image.cols / (double)image.rows; // CAMERA_V_FOV / CAMERA_H_FOV;
  cParams.ppx = image.cols / 2;
  cParams.ppy = image.rows / 2;
  
  cout <<"Prinicipal point x: " << cParams.ppx << endl;
  cout <<"Prinicipal point y: " << cParams.ppy << endl;

  /**
   * See http://planning.cs.uiuc.edu/node102.html for information about how the
   * rotation matrix is constructed
   */

  float cosR = cosDegrees(roll);
  float cosP = cosDegrees(pitch);
  float cosY = cosDegrees(yaw);

  float sinR = sinDegrees(roll);
  float sinP = sinDegrees(pitch);
  float sinY = sinDegrees(yaw);


  float rotationMatrix[3][3] = 
    { 
      { cosR * cosP, cosR * sinP * sinY - sinR * cosP, cosR * sinP * cosY + sinR * sinY } ,
      { sinR * cosP, sinR * sinP * sinY + cosR * cosY, sinR * sinP * cosY - cosR * sinY } ,
      { 0 - sinP, cosP * sinY, cosP * cosY }
    }; 

  /**
   * Using `clone()` to copy from rotationMatrix. Otherwise the data will be invalidated
   * when this stack frame is collapsed
   */
  cParams.R = cv::Mat(3,3,CV_32F, rotationMatrix).clone();

  float lat = (float)((latitude - minLat));
  float lon = (float)((longitude - minLon));
  float alt = (float)altitude;

  float x = lon * cosDegrees(latitude) * 40e6 / 360.0;
  float y = lat * 40e6 / 360;
  float z = alt * -6.0;
  
  float translationMatrix[3][1] = {x,y,z};

  /**
   * Using `clone()` to copy from translationMatrix. Otherwise the data will be invalidated
   * when this stack frame is collapsed
   */
  cParams.t = cv::Mat(3,1,CV_32F,translationMatrix).clone();

  cout << "Rotation Matrix:\n";
  for (int i = 0; i < 3; i ++){
    auto matPtr = cParams.R.ptr<float>(i);
    cout << "{";
    for (int j = 0; j < 3; j++){
      cout << matPtr[j] << ", ";
    }
    cout << "}\n";
  }
  
  cout << "Translation Matrix:\n";
  cout << "{";
  for (int i = 0; i < 3; i ++){
    cout << cParams.t.at<float>(i,0) << ", ";
  }
  cout <<"}\n";
  return cParams;
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

Pixel ImageWithPlaneData::getPixelFor(LatLon latlon){
  double pixelX, pixelY;
  GeoReference::pixelGeoreference(latitude,
                    longitude,
                    100,
                    roll,
                    pitch,
                    yaw,
                    gimbalRoll,
                    gimbalPitch,
                    latlon.lat,
                    latlon.lon,
                    CAMERA_H_FOV,
                    CAMERA_V_FOV,
                    image.cols,
                    image.rows,
                    pixelX,
                    pixelY);
  Pixel result = Pixel(pixelX,pixelY);

  if ( pixelX < 0 || pixelX > image.cols || pixelY < 0 || pixelY > image.rows){
    // cerr << result << " is not valid for " << latlon << endl;
    throw "Error";
  }

  //cout << "The pixel for " << latlon << " is " << result << endl;
  return result;
}

cv::KeyPoint Pixel::toKeyPoint(double scale){
  return cv::KeyPoint((float)x*(float)scale, // x
                      (float)y*(float)scale, // y
                      1.0,                   // size
                      -1,                    // angle
                      100);                     // response

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
