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
    if (intersection->num_contours == 0 ) continue; // No intersection
    //gpc_write_polygon(stdout, 1, intersection);
 
    //cout <<"Intersection "<<i<< " extremes :\n";
    GPSExtremes extremes(intersection);
    
    double maxLon = extremes.maxLon;
    double maxLat = extremes.maxLat;
    double minLon = extremes.minLon;
    double minLat = extremes.minLat;
    double dLat = maxLat - minLat;
    double dLon = maxLon - minLon;
    
    /**
      cout <<"minLat: "<<minLat<<endl;
      cout <<"maxLat: "<<maxLat<<endl;
      cout <<"minLon: "<<minLon<<endl;
      cout <<"maxLon: "<<maxLon<<endl;
    **/

    /* Find the GPS locations of the four points */
    LatLon point1(minLat + dLat / 3, minLon + dLon / 3);
    LatLon point2(minLat + 2 * dLat / 3, minLon + dLon / 3);
    LatLon point3(minLat + 2 * dLat / 3, minLon + 2 * dLon / 3);
    LatLon point4(minLat + dLat / 3, minLon + 2 * dLon / 3);
    // LatLon point5(rand(),rand());

    gpsData.push_back(point1);
    gpsData.push_back(point2);
    gpsData.push_back(point3);
    gpsData.push_back(point4);
    // gpsData.push_back(point5);

    /* Convert the GPS locations of the points to pixels in the original image */
    Pixel pixel1 = imageWithData.getPixelFor(point1);
    Pixel pixel2 = imageWithData.getPixelFor(point2);
    Pixel pixel3 = imageWithData.getPixelFor(point3);
    Pixel pixel4 = imageWithData.getPixelFor(point4);
    // Pixel pixel5 = Pixel(5,5);

    /* Convert the pixels in the original images to keypoints in the resized image */
    KeyPoint keyPoint1 = pixel1.toKeyPoint(scale);
    KeyPoint keyPoint2 = pixel2.toKeyPoint(scale);
    KeyPoint keyPoint3 = pixel3.toKeyPoint(scale);
    KeyPoint keyPoint4 = pixel4.toKeyPoint(scale);
    // KeyPoint keyPoint5 = pixel5.toKeyPoint(scale);

    keyPoints.push_back(keyPoint1);
    keyPoints.push_back(keyPoint2);
    keyPoints.push_back(keyPoint3);
    keyPoints.push_back(keyPoint4);
    // keyPoints.push_back(keyPoint5);

    
    cout <<"Pixel1: ("<<pixel1.x<<","<<pixel1.y<<")   ";
    printKeyPoint(keyPoint1);
    cout <<"LatLon1: " << point1.lat <<", "<<point1.lon<<endl;
    cout <<"Pixel2: ("<<pixel2.x<<","<<pixel2.y<<")   ";
    printKeyPoint(keyPoint2);
    cout <<"LatLon2: " << point2.lat <<", "<<point2.lon<<endl;
    cout <<"Pixel3: ("<<pixel3.x<<","<<pixel3.y<<")   ";
    printKeyPoint(keyPoint3);
    cout <<"LatLon3: " << point3.lat <<", "<<point3.lon<<endl;
    cout <<"Pixel4: ("<<pixel4.x<<","<<pixel4.y<<")   ";
    printKeyPoint(keyPoint4);
    cout <<"LatLon4: " << point4.lat <<", "<<point4.lon<<endl;
    /**
    cout <<"Pixel5: ("<<pixel5.x<<","<<pixel5.y<<")   ";
    printKeyPoint(keyPoint5);
    cout <<"LatLon5: " << point5.lat <<", "<<point5.lon<<endl;
    **/
    cout <<"\n\n";

  }

  Mat descriptors;
  if (features.keypoints.size() > 0 ){
    descriptors = features.descriptors.reshape(0,   // Channel count. 0 indicates the same number of channels
      features.keypoints.size() + keyPoints.size()); // Allocate space for the other descriptors
  } else {
    descriptors = Mat(keyPoints.size(),2,CV_32FC1); 
  }

  for(unsigned int i =0; i < gpsData.size(); i++){
    float* Mi = descriptors.ptr<float>(features.keypoints.size() + i);
    Mi[0] = (float)gpsData[i].lat;
    Mi[1] = (float)gpsData[i].lon;
    cout <<Mi[0] << " "<<Mi[1] << endl;
  }

  features.img_idx = imageIndex;
  features.img_size = image.size();
  features.keypoints.insert(features.keypoints.end(),keyPoints.begin(),keyPoints.end());
  features.descriptors = descriptors;
}
