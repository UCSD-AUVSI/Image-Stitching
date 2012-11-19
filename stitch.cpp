
#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <vector>
#include "opencv2/stitching/stitcher.hpp"
#include "opencv2/stitching/detail/matchers.hpp"
#include "stitch.h"
#include "gpc.h"
#include "gpc.c"
#include "math.h"

#define PI 3.14159

using namespace std;
using namespace cv;
using namespace cv::detail;

ImageWithGPS::ImageWithGPS(){}

ImageWithGPS::ImageWithGPS(Mat image, gpc_polygon gpsPolygon): image(image), gpsPolygon(gpsPolygon) {
  
  scale = findScale(image, gpsPolygon);
  ang = findAngleGPS(gpsPolygon.contour->vertex[0].x,
              gpsPolygon.contour->vertex[0].y,
              gpsPolygon.contour->vertex[1].x,
              gpsPolygon.contour->vertex[1].y);
}

bool nearly(double a, double b, double epsilon = 0.001){
  if (fabs(a-b) > epsilon){
    cerr << a << " != " << b << endl; 
    return false;
  }
  return true;
}

vector<int> ImageWithGPS::gpsToPixels(double lon, double lat){
  vector<int> result;  
  
  int x =(int) (scale * ((lon * cos(ang)) - (lat * sin(ang))));
  int y =(int) (scale * ((lat * sin(ang)) - (lat * cos(ang))));
  result.push_back(x);
  result.push_back(y);
  return result;
}


Mat rotateImage(const Mat &source, double angle, Size size){
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



double findScale(Mat img, gpc_polygon gpsPoly){

	double lat1 = gpsPoly.contour->vertex[0].x;
	double lon1 = gpsPoly.contour->vertex[0].y; 
	double lat2 = gpsPoly.contour->vertex[1].x;
	double lon2 = gpsPoly.contour->vertex[1].y;
	double lat3 = gpsPoly.contour->vertex[3].x;
	double lon3 = gpsPoly.contour->vertex[3].y; 
	
	double distance_12 = distance(lat1,lon1,lat2,lon2);
	double distance_13 = distance(lat1,lon1,lat3,lon3);
    double largeSideGPS = max (distance_12, distance_13);
    double smallSideGPS = min (distance_12, distance_13);
    double largeSidePixels = max(img.rows,img.cols);
    double smallSidePixels= min(img.rows,img.cols);

    double largeScale = largeSideGPS / largeSidePixels;
    double smallScale = largeSideGPS / largeSidePixels;

    return (largeScale + smallScale) / 2.0;

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

class GPSFeaturesFinder: public FeaturesFinder {
  public:
	int img_idx; 
    vector<ImageWithGPS> images;
    GPSFeaturesFinder(vector<ImageWithGPS> images){
	  img_idx = -1;
      this->otherImages = images;
    }
    void find(const Mat &image, ImageFeatures &features){
      (*this)(image,features);
    }
    void operator ()(const Mat &image, ImageFeatures &features) {
      vector<Point2f> gpsData;
      vector<KeyPoint> all;
      ImageWithGPS data;
	  img_idx++;
      data = otherImages[img_idx];
	  
	  cout<<"ImageWithGPS Rows: "<<data.image.rows<<"\n"; 
	  cout<<"ImageWithGPS Cols: "<<data.image.cols<<"\n";
	  cout<<"Image Rows: "<<image.rows<<"\n"; 
	  cout<<"Image Cols: "<<image.cols<<"\n";

      for (unsigned int i = 0; i< otherImages.size(); i++){
		  if(data.image.data == otherImages.at(i).image.data) continue;

        gpc_polygon* intersection = new gpc_polygon();
		gpc_polygon_clip( GPC_INT, &data.gpsPolygon, &otherImages[i].gpsPolygon,intersection);
		vector<double> coord = getExtremes(data.gpsPolygon);

       float maxLon = (float) coord.back(); coord.pop_back();
	   float maxLat = (float) coord.back(); coord.pop_back();
	   float minLon = (float) coord.back(); coord.pop_back();
	   float minLat = (float) coord.back(); coord.pop_back();


        vector<int> ul = data.gpsToPixels(maxLon, minLat);
        vector<int> ur = data.gpsToPixels(maxLon, maxLat);
        vector<int> bl = data.gpsToPixels(minLon, minLat);
        vector<int> br = data.gpsToPixels(minLon, maxLat);

        Point2f ulPoint = Point2f((float)ul[0], (float)ul[1]);
        Point2f urPoint = Point2f((float)ur[0], (float)ur[1]);
        Point2f blPoint = Point2f((float)bl[0], (float)bl[1]);
        Point2f brPoint = Point2f((float)br[0], (float)br[1]);

        KeyPoint ulKeyPt = KeyPoint(ulPoint, 1);
        KeyPoint urKeyPt = KeyPoint(urPoint, 1);
        KeyPoint blKeyPt = KeyPoint(blPoint, 1);
        KeyPoint brKeyPt = KeyPoint(brPoint, 1);

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
		float* Mi = descriptors.ptr<float>(i);
		Mi[0] = gpsData[i].x;
		Mi[1] = gpsData[i].y;
        //descriptors.push_back(gpsData[i].x);
        //descriptors.push_back(gpsData[i].y);

      }

      features.img_idx = img_idx;
      features.img_size =  image.size();
      features.keypoints = all;
      features.descriptors = descriptors;
    }
private:
	vector<ImageWithGPS> otherImages;
};

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
// for simple testing, not include gpspolygon
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
// 
ImageWithGPS iterativeStitch(ImageWithGPS accumulatedImage, vector<ImageWithGPS> newImages) {
  Mat result;
  //Rect_<double> rect = accumulatedImage.rect;
  gpc_polygon poly = accumulatedImage.gpsPolygon;

  vector<Mat> newVec(newImages.size()+1);
  /*for(unsigned int i =0; i < newImages.size(); i++){
    if(newImages[i].rect.x < rect.x)
      rect.x = newImages[i].rect.x;
    if(newImages[i].rect.y < rect.y)
      rect.y = newImages[i].rect.y;
    if(newImages[i].rect.height+newImages[i].rect.y > rect.y+rect.height)
      rect.height = newImages[i].rect.height+newImages[i].rect.y-rect.y;
    if(newImages[i].rect.width+newImages[i].rect.x > rect.x+rect.width)
      rect.width = newImages[i].rect.width+newImages[i].rect.x-rect.x;
    newVec[i] = newImages[i].image;
  }*/
  Stitcher stitcher = Stitcher::createDefault(true);

  newVec.push_back(accumulatedImage.image);
  stitcher.stitch(newVec, result);
  return ImageWithGPS(result,poly);
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

}

