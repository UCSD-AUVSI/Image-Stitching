#include "GPSFeaturesFinder.h"

void GPSFeaturesFinder::operator()(const Mat &image, ImageFeatures &features) {
  vector<Point2f> gpsData;
  vector<KeyPoint> all;
  ImageWithGPS data;
  img_idx++;
  data = otherImages[img_idx];

  cout<<"ImageWithGPS Rows: "<<data.image.rows<<"\n"; 
  cout<<"ImageWithGPS Cols: "<<data.image.cols<<"\n";
  cout<<"Image Rows: "<<image.rows<<"\n"; 
  cout<<"Image Cols: "<<image.cols<<"\n";

  double scale = (double)image.rows / (double)data.image.rows;
  cout << "Scale: " << scale << endl;

  for (unsigned int i = 0; i< otherImages.size(); i++){
    if(data.image.data == otherImages.at(i).image.data) continue;

    gpc_polygon* intersection = new gpc_polygon();
    gpc_polygon_clip( GPC_INT, &data.gpsPolygon, &otherImages[i].gpsPolygon,intersection);
    GPSExtremes coord = getGPSExtremes(data.gpsPolygon);
    float maxLon = (float) coord.maxLon;
    float maxLat = (float) coord.maxLat;
    float minLon = (float) coord.minLon;
    float minLat = (float) coord.minLat;
    vector<int> ul = data.gpsToPixels(maxLon, minLat);
    vector<int> ur = data.gpsToPixels(maxLon, maxLat);
    vector<int> bl = data.gpsToPixels(minLon, minLat);
    vector<int> br = data.gpsToPixels(minLon, maxLat);

    /*
       ul[0] *= scale;
       ul[1] *= scale;
       ur[0] *= scale;
       ur[1] *= scale;
       bl[0] *= scale;
       bl[1] *= scale;
       br[0] *= scale;
       br[1] *= scale;
     */

    Point2f ulPoint = Point2f((float)ul[0], (float)ul[1]);
    Point2f urPoint = Point2f((float)ur[0], (float)ur[1]);
    Point2f blPoint = Point2f((float)bl[0], (float)bl[1]);
    Point2f brPoint = Point2f((float)br[0], (float)br[1]);

    KeyPoint ulKeyPt = KeyPoint(ulPoint, 1);
    KeyPoint urKeyPt = KeyPoint(urPoint, 1);
    KeyPoint blKeyPt = KeyPoint(blPoint, 1);
    KeyPoint brKeyPt = KeyPoint(brPoint, 1);

    cout <<"UlPt: "<<ulPoint<<endl;
    cout <<"UrPt: "<<urPoint<<endl;
    cout <<"BlPt: "<<blPoint<<endl;
    cout <<"BrPt: "<<brPoint<<endl;

    all.push_back(ulKeyPt);
    all.push_back(urKeyPt);
    all.push_back(blKeyPt);
    all.push_back(brKeyPt);

    gpsData.push_back(Point2f (maxLon,minLat));
    gpsData.push_back(Point2f (maxLon,maxLat));
    gpsData.push_back(Point2f (minLon, minLat));
    gpsData.push_back(Point2f (minLon,maxLat));
  }

  Mat descriptors(all.size(),2,CV_32FC1);

  for(unsigned int i =0; i < gpsData.size(); i++){
    int* Mi = descriptors.ptr<int>(i);
    Mi[0] = 5; // gpsData[i].x * 1000.0;
    Mi[1] = 5; // gpsData[i].y * 1000.0;
  }
  for(unsigned int i =0; i < gpsData.size(); i++){
    int* Mi = descriptors.ptr<int>(i);
    cout <<Mi[0]<<" "<<Mi[1]<<endl;
  }

  features.img_idx = img_idx;
  features.img_size =  image.size();
  features.keypoints = all;
  features.descriptors = descriptors;
}
