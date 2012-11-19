#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <vector>
#include "opencv2/stitching/stitcher.hpp"
#include "opencv2/stitching/detail/matchers.hpp"
#include "stitch.h"
#include "GPSFeaturesFinder.h"
#include "gpc.h"

#ifdef __WIN32__
#include "gpc.c"
#endif

#include "math.h"

#define PI 3.14159

using namespace std;
using namespace cv;
using namespace cv::detail;

double toRadians(double degrees);
bool nearly(double a, double b, double epsilon = 0.001);
Mat rotateImage(const Mat &source, double angle);
void testGetExtremes();
void testFindScale();
double distance(double x1, double y1, double x2, double y2);
void testDistance();
double findAngleGPS(double lat1, double lon1, double lat2, double lon2);
void testFindAngleGPS();
vector<ImageWithGPS> getTestDataForImage(Mat image,
    int rows,
    int columns,
    double horizontalOverlap,
    double verticalOverlap,
    double scale);

/**
 * Takes an angle in degrees as input and returns the same angle in radians
 */
double toRadians(double degrees){
  return degrees / 180 * PI;
}

/**
 * Returns true if two values are within `epsilon` of each other
 */
bool nearly(double a, double b, double epsilon){
  if (fabs(a-b) > epsilon){
    cerr << a << " != " << b << endl; 
    return false;
  }
  return true;
}

/**
 * Returns a Matrix that is rotated by `angle` degrees
 */
Mat rotateImage(const Mat &source, double angle) {
  Point2f src_center(source.cols/2.0F, source.rows/2.0F);
  Mat rot_mat = getRotationMatrix2D(src_center,angle, 1.0);
  Mat dst;
  warpAffine(source, dst, rot_mat, source.size());
  return dst;
}

vector<double> getExtremes (gpc_polygon polygon){
  gpc_vertex* vertices = polygon.contour->vertex;
  double minLat = INT_MAX;
  double minLon = INT_MAX;
  double maxLat = INT_MIN;
  double maxLon = INT_MIN;
  vector<double> result;
  for(int i = 0; i < 4; i++){
    if (vertices[i].y < minLon ) minLon = vertices[i].y;
    if (vertices[i].y > maxLon ) maxLon = vertices[i].y;
    if (vertices[i].x < minLat ) minLat = vertices[i].x;
    if (vertices[i].x > maxLat ) maxLat = vertices[i].x;
  }
  result.push_back(minLat);
  result.push_back(minLon);
  result.push_back(maxLat);
  result.push_back(maxLon);
  return result;
}

void testGetExtremes(){
  cerr << "Testing getExtremes...";
  gpc_vertex bottomLeft; bottomLeft.x = 32; bottomLeft.y = -116;
  gpc_vertex bottomRight; bottomRight.x = 32;bottomRight.y = -116;
  gpc_vertex topRight; topRight.x = 33; topRight.y = -116;
  gpc_vertex topLeft; topLeft.x = 33; topLeft.y = -117;
  gpc_vertex vertices[] = {topLeft,topRight,bottomRight,bottomLeft};
  gpc_vertex_list* list = new gpc_vertex_list();
  list->num_vertices = 4;
  list->vertex = vertices;
  gpc_polygon polygon;
  polygon.num_contours = 1;
  polygon.hole=0;
  polygon.contour=list;
  vector<double> extremes = getExtremes(polygon);
  assert(extremes[0] == 32); // Min Lat
  assert(extremes[1] == -117); // Min Lon
  assert(extremes[2] == 33); // Max Lat
  assert(extremes[3] == -116); // Max Lon
  cerr <<"Complete\n";
}


void testFindScale(){
  cout <<"Testing findScale...";
  Mat image = imread("image.jpg");
  double maxLat = (double)image.cols / 1000.0;
  double maxLon = (double)image.rows / 1000.0;
  gpc_vertex bottomLeft; bottomLeft.x = 0; bottomLeft.y = 0;
  gpc_vertex bottomRight; bottomRight.x = 0; bottomRight.y = maxLon;
  gpc_vertex topRight; topRight.x = maxLon; topRight.y = maxLat;
  gpc_vertex topLeft; topLeft.x = maxLon; topLeft.y = 0;
  gpc_vertex vertices[] = {topLeft,topRight,bottomRight,bottomLeft};
  gpc_vertex_list* list = new gpc_vertex_list();
  list->num_vertices = 4;
  list->vertex = vertices;
  gpc_polygon polygon;
  polygon.num_contours = 1;
  polygon.hole=0;
  polygon.contour=list;
  assert(nearly(findScale(image,polygon),0.001,0.0000001));
  cout <<"Complete\n";
}



double toDegrees(double radians){
  return radians / PI * 180.0;
}


double distance(double x1, double y1, double x2, double y2){
  return sqrt(pow(x2-x1,2)+pow(y2-y1,2));
}

void testDistance(){
  cerr <<"Testing distance...";
  assert(nearly(distance(0,0,3,4),5));
  cerr<<"Complete\n";
}

double findAngleGPS(double lat1, double lon1, double lat2, double lon2){
  double dlat = lat2-lat1;
  double dlon = lon2-lon1;
  return toDegrees(atan(dlat/dlon));
}

void testFindAngleGPS(){
  cerr <<"Testing findAngle...";
  assert(nearly(findAngleGPS(0,0,5,5),45.0));
  assert(nearly(findAngleGPS(0,0,3,10),16.699));
  cerr <<"Complete\n";
}

vector<ImageWithGPS> getTestDataForImage(Mat image,
    int rows,
    int columns,
    double horizontalOverlap,
    double verticalOverlap,
    double scale){
  vector<ImageWithGPS> resultImages = vector<ImageWithGPS>(rows * columns);

  int normalWidth = image.cols / columns;
  int normalHeight = image.rows / rows;
  int overlapWidth = (int)((double)(normalWidth) * horizontalOverlap);
  int overlapHeight = (int)((double)(normalHeight) * verticalOverlap); 

  cout << "Original Width: " << image.cols << "\n";
  cout << "Original Height: " << image.rows << "\n";
  cout << "Normal Width: " << normalWidth <<"\n";
  cout << "Normal Height: " << normalHeight <<"\n";
  cout << "Overlap Width: " << overlapWidth <<"\n";
  cout << "Overlap Height: " << overlapHeight <<"\n";
  cout << endl;

  int imageX, imageY, imageWidth, imageHeight;
  for (int j = 0; j < rows; j++){
    for (int i = 0; i < columns; i++){
      imageX = max(i * normalWidth - overlapWidth,0);
      if (i == 0){
        imageWidth = min(normalWidth + overlapWidth, image.cols);
      } else {
        imageWidth = min(normalWidth + 2 * overlapWidth,image.cols - imageX);
      }
      imageY = max(j * normalHeight - overlapHeight,0);
      if (i == 0){
        imageHeight = min(normalHeight + overlapHeight, image.rows);
      } else {
        imageHeight = min(normalHeight + 2 * overlapHeight ,image.rows - imageY);
      }
      cout <<"Image "<<j * rows + i<<"\n";
      cout <<"X: "<<imageX<<"\n";
      cout <<"Y: "<<imageY<<"\n";
      cout <<"Width: "<<imageWidth<<"\n";
      cout <<"Height: "<<imageHeight<<"\n";
      cout <<endl;
      Mat result = Mat(image,Range(imageY, imageY+imageHeight),Range(imageX,imageX +imageWidth));
      gpc_polygon coords;
      coords.num_contours = 1;
      coords.hole = 0;
      coords.contour = new gpc_vertex_list(); 
      coords.contour->vertex = (gpc_vertex*)malloc(sizeof(gpc_vertex)*4);  
      coords.contour->vertex[0].x = imageX*scale;
      coords.contour->vertex[0].y = imageY*scale;
      coords.contour->vertex[1].x = (imageX+imageWidth)*scale;
      coords.contour->vertex[1].y = imageY*scale;
      coords.contour->vertex[2].x = imageX+imageWidth*scale;
      coords.contour->vertex[2].y = (imageY+imageHeight)*scale;
      coords.contour->vertex[3].x = imageX*scale;
      coords.contour->vertex[3].y = (imageY+imageHeight)*scale;
      resultImages[rows * j + i] = ImageWithGPS(result,coords);	  
    }
  }
  return resultImages;
}

int main(){

  testGetExtremes();
  testFindAngleGPS();
  testDistance();
  testFindScale();

  Mat pano;
  vector<ImageWithGPS> images = getTestDataForImage(imread("image.jpg"),2,2,0.2,0.2,0.9);
  imwrite("a.jpg",images[0].image);
  imwrite("b.jpg",images[1].image);
  imwrite("c.jpg",images[2].image);
  imwrite("d.jpg",images[3].image);
  vector<Mat> _images;
  _images.push_back(images[0].image);
  _images.push_back(images[1].image);
  _images.push_back(images[2].image);
  _images.push_back(images[3].image);
  Stitcher stitcher = stitcher.createDefault(true);
  stitcher.setFeaturesFinder(cv::Ptr<FeaturesFinder>(new GPSFeaturesFinder(images)));
  stitcher.stitch(_images,pano);
  imwrite("result.jpg",pano);
  getchar();
}
