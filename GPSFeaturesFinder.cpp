#include "GPSFeaturesFinder.h"
#include "util.h"
#include "DataTypes.h"
#include <iostream>
using namespace std;

void GPSFeaturesFinder::operator()(const Mat &image, ImageFeatures &features) {
  imageIndex++;

  vector<KeyPoint> keyPoints;
  vector<LatLon> gpsData;

  /* Associate this image with the data */
  ImageWithPlaneData imageWithData = imagesWithData[imageIndex];

  if (imageWithData.image.rows == 0 || imageWithData.image.cols == 0){
    cout << "GPSFeatureFinder failed: imageWithData is empty\n";
    cout << "Image index: "<< imageIndex << endl;
    assert(false);
  }

  /**
    cout<<"Image "<<imageIndex<<"\n";
    cout<<"ImageWithPlaneData Rows: "<<imageWithData.image.rows<<"\n"; 
    cout<<"ImageWithPlaneData Cols: "<<imageWithData.image.cols<<"\n";
    cout<<"Image Rows: "<<image.rows<<"\n"; 
    cout<<"Image Cols: "<<image.cols<<"\n";
   **/

  /* Determine the scale of the image */
  double scale = (double) image.rows / (double)imageWithData.image.rows;
  cout << "Scale: " << scale << endl;

  for (unsigned int i = 0; i< imagesWithData.size(); i++){
    if(imageWithData.image.data == imagesWithData.at(i).image.data) continue;

    /* Compute the intersection between the two GPS polygons */
    gpc_polygon* intersection = new gpc_polygon();
    gpc_polygon_clip( GPC_INT,
        imageWithData.toGPCPolygon(),
        imagesWithData[i].toGPCPolygon(),
        intersection);
    if (intersection->num_contours == 0 ) continue;

    GPSExtremes extremes(intersection);

    double maxLon = extremes.maxLon;
    double maxLat = extremes.maxLat;
    double minLon = extremes.minLon;
    double minLat = extremes.minLat;
    double dLat = maxLat - minLat;
    double dLon = maxLon - minLon;
    int max = 2;
    cout<<"Image Rows: "<<image.rows<<"\n"; 
    cout<<"Image Cols: "<<image.cols<<"\n";
    for(int _i = 1; _i<= max; _i++){
      for(int _j = 1; _j<= max; _j++){
        LatLon point(minLat+ _i*dLat/(max+1) , minLon+ _j*dLon/(max+1) );
        cout << point.lat << " " << point.lon <<endl;
        gpsData.push_back(point);
        Pixel pixel = imageWithData.getPixelFor(point);
        KeyPoint keyPoint = pixel.toKeyPoint(scale);
        keyPoints.push_back(keyPoint);
        printKeyPoint(keyPoint);
      }
    }

    Mat descriptors = Mat(keyPoints.size(),128,CV_32FC1); 

    for(unsigned int i =0; i < gpsData.size(); i++){
      float* Mi = descriptors.ptr<float>(i);
      Mi[0] = (float)gpsData[i].lat;
      Mi[1] = (float)gpsData[i].lon;
      for (int j = 2; j < 128; j++){
        Mi[j] = 0;
      }
      cout <<Mi[0] << " "<<Mi[1] << endl;
    }

    features.img_idx = imageIndex;
    features.img_size = image.size();
    //features.keypoints = keyPoints;
    //features.descriptors = descriptors;
    features.keypoints.insert(features.keypoints.end(),keyPoints.begin(),keyPoints.end());
    features.descriptors.push_back(descriptors);
  }
}
