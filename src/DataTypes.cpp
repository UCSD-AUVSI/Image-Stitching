#include <cv.h>
#include "DataTypes.h"
#include "camera.h"
#include <iostream>
#include <cmath>

#define FEET_PER_METER 3.28084

using namespace std;
using namespace cv::detail;

float sinDegrees(float degrees){
  return sin(2.0f * M_PI * degrees / 360.0);
}

float cosDegrees(float degrees){
  return cos(2.0f * M_PI * degrees / 360.0);
}

cv::detail::CameraParams ImageWithPlaneData::getCameraParams(double minLat,
                                                             double minLon,
                                                             CameraArgs args) const {
  assert(!image.empty());
  CameraParams cParams;

  cParams.focal = args.focalLength; //CAMERA_FOCAL_MM * 72.0 / 25.4;
  cParams.aspect = (double)image.cols / (double)image.rows; // CAMERA_V_FOV / CAMERA_H_FOV;
  cParams.ppx = image.cols / 2;
  cParams.ppy = image.rows / 2;
  
  /**
    cout <<"Principal point x: " << cParams.ppx << endl;
    cout <<"Principal point y: " << cParams.ppy << endl;
  */

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

  float x = lon * cosDegrees(latitude) * args.lonScale;
  float y = lat * args.latScale; 

  /**
   * The altitude must be greater than 0. Multiplies by the user-input scale
   */
  float z = max(0.0,(alt - args.groundLevel)) / FEET_PER_METER * args.altScale+1; 

  cout << "Alt: " << z << endl;
  
  float translationMatrix[3][1] = {x,y,-z};

  /**
   * Using `clone()` to copy from translationMatrix. Otherwise the data will be invalidated
   * when this stack frame is collapsed
   */
  cParams.t = cv::Mat(3,1,CV_32F,translationMatrix).clone();

  /**
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
  */
  return cParams;
}
