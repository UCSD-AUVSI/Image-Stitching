#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <vector>
#include <opencv2/stitching/stitcher.hpp>
#include <opencv2/stitching/detail/matchers.hpp>
#include "stitch.h"
#include "GPSFeaturesFinder.h"
#include "gpc.h"

#ifdef __WIN32__
#include "gpc.c"
#endif

using namespace std;
using namespace cv;
using namespace cv::detail;

double metersToGPS(double meters){
  return meters * 111222.0;
}

double toRadians(double degrees){
  return degrees / 180.0 * M_PI;
}

vector<ImageWithGPS> getTestDataForImage(Mat image,
    int rows,
    int columns,
    double horizontalOverlap,
    double pixelsPerMeter,
    double minLat,
    double minLon) {
  vector<ImageWithPlaneData> resultImages = vector<ImageWithPlaneData>(rows * columns);
  
  double heightMeters = image.rows / pixelsPerMeter;
  double widthMeters = image.rows / pixelsPerMeter;
  double maxLat = minLat + metersToGPS(heightMeters); 
  double maxLon = minLon + metersToGPS(widhtMeters); 
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
      double planeLat = minLat + metersToGPS((imageCenterX / image.cols) / pixelsPerMeter)
      double planeLon = minLon + metersToGPS((imageCenterY / image.rows) / pixelsPerMeter)
      double planeAlt = 2 * (imageCenterY / pixelsPerMeter) * tan(0.5 * toDegrees(CAMERA_H_FOV));
      
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
      resultImages[rows * j + i] = {
        result,
        planeLat,
        planeLon,
        planeAlt,
        0.0, // roll
        0.0, // pitch
        0.0, // yaw
        0.0, // gimbalRoll
        0.0, // gimbalYaw
      }
    }
  }
  return resultImages;
}

int main(){

  testGetExtremes();
  testFindAngleGPS();
  testDistance();
  //testFindScale();

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
