#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <vector>
#include <opencv2/stitching/stitcher.hpp>
#include <opencv2/stitching/detail/matchers.hpp>
#include "stitch.h"
#include "GPSFeaturesFinder.h"
#include "gpc.h"
#include "DataTypes.h"
#include "camera.h"

#ifdef __WIN32__
#include "gpc.c"
#endif

using namespace std;
using namespace cv;
using namespace cv::detail;

double metersToGPS(double meters){
  return meters / 111222.0;
}

double toRadians(double degrees){
  return degrees / 180.0 * M_PI;
}

double toDegrees(double radians){
  return radians * 180.0 / M_PI;
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
  GPSExtremes extremes = GPSExtremes(&polygon);
  assert(extremes.minLat == 32);    // Min Lat
  assert(extremes.minLon == -117);  // Min Lon
  assert(extremes.maxLat== 33);     // Max Lat
  assert(extremes.maxLon == -116);  // Max Lon
  cerr <<"Complete\n";
}

vector<ImageWithPlaneData> getTestDataForImage(Mat image,
    int rows,
    int columns,
    double horizontalOverlap,
    double pixelsPerMeter,
    double minLat,
    double minLon) {
  vector<ImageWithPlaneData> resultImages = vector<ImageWithPlaneData>(rows * columns);
  
  double heightMeters = (double)image.rows / pixelsPerMeter;
  double widthMeters = (double)image.cols / pixelsPerMeter;
  double maxLat = minLat + metersToGPS(heightMeters); 
  double maxLon = minLon + metersToGPS(widthMeters); 
  double verticalOverlap = CAMERA_V_FOV / CAMERA_H_FOV * horizontalOverlap;

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
  cout << "Pixels per Meter: " << pixelsPerMeter <<"\n";
  cout << "MinLat: " << minLat << "\n";
  cout << "MaxLat: " << maxLat << "\n";
  cout << "MinLon: " << minLon << "\n";
  cout << "MaxLon: " << maxLon << "\n";
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

      double imageCenterX = imageX + imageWidth / 2;
      double imageCenterY = imageY + imageHeight / 2;
      double planeLat = minLat + metersToGPS(imageCenterY / pixelsPerMeter);
      double planeLon = minLon + metersToGPS(imageCenterX / pixelsPerMeter);
      double planeAlt = 2 * (imageCenterY / pixelsPerMeter) * tan(0.5 * toRadians(CAMERA_H_FOV));
      
      cout <<"Image "<<j * rows + i<<"\n";
      cout <<"X: "<<imageX<<"\n";
      cout <<"Y: "<<imageY<<"\n";
      cout <<"Width: "<<imageWidth<<"\n";
      cout <<"Height: "<<imageHeight<<"\n";
      cout <<"Image Center X: "<<imageCenterX<<"\n";
      cout <<"Image Center Y: "<<imageCenterY<<"\n";
      cout <<"Plane Latitude: "<<planeLat<<"\n";
      cout <<"Plane Longitude: "<<planeLon<<"\n";
      cout <<"Plane Altitude: "<<planeAlt<<"\n";
      cout <<endl;

      Mat result = Mat(image,Range(imageY, imageY+imageHeight),Range(imageX,imageX +imageWidth));
      if ( result.cols == 0 || result.rows == 0){
        cout << "Test data failed, the image is empty\n"; 
        assert(false);
      }
      resultImages[rows * j + i] = ImageWithPlaneData(
        result,
        planeLat,
        planeLon,
        planeAlt,
        0.0, // roll
        0.0, // pitch
        0.0, // yaw
        0.0, // gimbalRoll
        0.0  // gimbalYaw
      ); 
      gpc_write_polygon(stdout, 0, resultImages[rows * j + i].toGPCPolygon());
    }
  }
  return resultImages;
}

int main(){

  testGetExtremes();

  Mat pano;
  
  vector<ImageWithPlaneData> imagesWithData = getTestDataForImage(
    imread("image.jpg"),      // image
    1,                        // rows
    2,                        // columns
    0.4,                      // horizontal overlap,
    1.0,                      // pixels per meter
    32.0,                     // minimum latitude
    -117.0);                  // minimum longitude

  imwrite("a.jpg",imagesWithData[0].image);
  imwrite("b.jpg",imagesWithData[1].image);
  //imwrite("c.jpg",imagesWithData[2].image);
  //imwrite("d.jpg",imagesWithData[3].image);

  vector<Mat> images;
  images.push_back(imagesWithData[0].image);
  images.push_back(imagesWithData[1].image);
  //images.push_back(imagesWithData[2].image);
  //images.push_back(imagesWithData[3].image);

  Stitcher stitcher = stitcher.createDefault(true);
  stitcher.setFeaturesFinder(cv::Ptr<FeaturesFinder>(new GPSFeaturesFinder(imagesWithData)));
  stitcher.setFeaturesMatcher(new BestOf2NearestMatcher(true, 0.1f,1,1));

  stitcher.stitch(images,pano);
  imwrite("result.jpg",pano);
  getchar();
}
