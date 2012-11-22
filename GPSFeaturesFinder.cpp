#include "GPSFeaturesFinder.h"

void GPSFeaturesFinder::operator()(const Mat &image, ImageFeatures &features) {
  
  vector<KeyPoint> keyPoints;

  /* Associate this image with the data */
  ImageWithPlaneData imageWithData = imagesWithData[imageIndex];

  cout<<"ImageWithGPS Rows: "<<imageWithData.image.rows<<"\n"; 
  cout<<"ImageWithGPS Cols: "<<imageWithData.image.cols<<"\n";
  cout<<"Image Rows: "<<image.rows<<"\n"; 
  cout<<"Image Cols: "<<image.cols<<"\n";

  /* Determine the scale of the image */
  double scale = (double)imageWithData.image.rows / (double)image.rows;
  cout << "Scale: " << scale << endl;

  for (unsigned int i = 0; i< otherImages.size(); i++){
    if(data.image.data == otherImages.at(i).image.data) continue;

    /* Compute the intersection between the two GPS polygons */
    gpc_polygon* intersection = new gpc_polygon();
    gpc_polygon_clip( GPC_INT, data.toGPCPolygon(), otherImages[i].toGPCPolygon(), intersection);
    if (!intersection) continue; // No intersection

    GPSExtremes extremes = getGPSExtremes(data.gpsPolygon);
    
    double maxLon = extremes.maxLon;
    double maxLat = extremes.maxLat;
    double minLon = extremes.minLon;
    double minLat = extremes.minLat;
    double dLat = maxLat - minLat;
    double dLon = maxLon - minLon;

    /* Find the GPS locations of the four points */
    LatLon point1 = (minLat + dLat / 3, minLon + dLon / 3);
    LatLon point2 = (minLat + 2 * dLat / 3, minLon + dLon / 3);
    LatLon point3 = (minLat + 2 * dLat / 3, minLon + 2 * dLon / 3);
    LatLon point4 = (minLat + dLat / 3, minLon + 2 * dLon / 3);

    gpsData.push_back(point1.toPoint2i());
    gpsData.push_back(point2.toPoint2i());
    gpsData.push_back(point3.toPoint2i());
    gpsData.push_back(point4.toPoint2i());

    /* Convert the GPS locations of the points to pixels in the original image */
    Pixel pixel1 = imageWithData.getPixelFor(point1);
    Pixel pixel2 = imageWithData.getPixelFor(point2);
    Pixel pixel3 = imageWithData.getPixelFor(point3);
    Pixel pixel4 = imageWithData.getPixelFor(point4);

    /* Convert the pixels in the original images to keypoints in the resized image */
    KeyPoint keyPoint1 = pixel1.toKeyPoint(scale);
    KeyPoint keyPoint2 = pixel2.toKeyPoint(scale);
    KeyPoint keyPoint3 = pixel3.toKeyPoint(scale);
    KeyPoint keyPoint4 = pixel4.toKeyPoint(scale);

    keyPoints.push_back(keyPoint1);
    keyPoints.push_back(keyPoint2);
    keyPoints.push_back(keyPoint3);
    keyPoints.push_back(keyPoint4);

    cout <<"KeyPoint1: "<<keyPoint1<<endl;
    cout <<"KeyPoint2: "<<keyPoint2<<endl;
    cout <<"KeyPoint3: "<<keyPoint3<<endl;
    cout <<"KeyPoint4: "<<keyPoint4<<endl;

  }

  Mat descriptors(keyPoints.size(),2,CV_32FC1);

  /* Add descriptors */
  for(unsigned int i =0; i < gpsData.size(); i++){
    int* Mi = descriptors.ptr<int>(i);
    Mi[0] = gpsData[i].x;
    Mi[1] = gpsData[i].y;
  }

  for(unsigned int i =0; i < gpsData.size(); i++){
    int* Mi = descriptors.ptr<int>(i);
    cout <<Mi[0]<<" "<<Mi[1]<<endl;
  }

  features.img_idx = img_idx;
  features.img_size =  image.size();
  features.keypoints = keyPoints;
  features.descriptors = descriptors;
}
