#include "util.h"
#include <cv.h>
#include <iostream>
#include "camera.h"
#include "GeoReference.h"

using namespace std;
using namespace cv;
using namespace Vision;


#define M_PI 3.1415926

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
  double maxLat = minLat + GeoReference::metersToGPS(heightMeters); 
  double maxLon = minLon + GeoReference::metersToGPS(widthMeters); 
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
      double planeLat = minLat + GeoReference::metersToGPS(imageCenterY / pixelsPerMeter);
      double planeLon = minLon + GeoReference::metersToGPS(imageCenterX / pixelsPerMeter);
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
void printKeyPoint(cv::KeyPoint keyPoint){
  cout << "KeyPoint(" << keyPoint.pt << " size: " << keyPoint.size << " response: " 
       << keyPoint.response << ")\n";
}

double toRadians(double degrees){
  return degrees / 180.0 * M_PI;
}

double toDegrees(double radians){
  return radians * 180.0 / M_PI;
}
