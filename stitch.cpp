
#include "StdAfx.h"
#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <vector>
#include "opencv2/stitching/stitcher.hpp"
#include "RotateRect.h"
#include "opencv2/stitching/detail/matchers.hpp"
#include "stitch.h"
#include "gpc.h"
#include "math.h"

using namespace std;
using namespace cv;
using namespace cv::detail;

ImageWithGPS::ImageWithGPS(){}
ImageWithGPS::ImageWithGPS(Mat image, gpc_polygon gpsPolygon){}

vector<int> ImageWithGPS::gpsToPixels(double lat, double lon){
  vector<int> result;   
  int x = scale * ((lon * cos(angle)) - (lat * sin(angle)));
  int y = scale * ((lat * sin(angle)) - (lat * cos(angle)));
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

void findMaxOrMin (double *arr){
   
   int i, j;
   double tmp;
   for (i = 1; i < 4; i++) {
     j = i;
     while (j > 0 && arr[j - 1] > arr[j]) {
     tmp = arr[j];
     arr[j] = arr[j - 1];
     arr[j - 1] = tmp;
     j--;
     }
   }
}

vector<double> getExtremes (gpc_vertex* vertices){
	double minLat = INT_MAX;
	double minLon = INT_MAX;
	double maxLat = INT_MIN;
	double maxLon = INT_MIN;
	vector<double> result;
	for(int i = 0; i < 4; i++){
		if (vertices[i].x < minLon ) minLon = vertices[i].x;
		if (vertices[i].x > maxLon ) maxLon = vertices[i].x;
		if (vertices[i].y < minLat ) minLat = vertices[i].y;
	    if (vertices[i].y > maxLat ) maxLat = vertices[i].y;

	}
	result.push_back(minLat);
	result.push_back(minLon);
	result.push_back(maxLat);
	result.push_back(maxLon);
	return result;
}


double scale_y(ImageWithGPS img){
	double scale;
	double x1 = img.gpsPolygon.contour->vertex[0].x;
	double y1 = img.gpsPolygon.contour->vertex[0].y; 
	double x2 = img.gpsPolygon.contour->vertex[1].x;
	double y2 = img.gpsPolygon.contour->vertex[1].y;
	double x3 = img.gpsPolygon.contour->vertex[3].x;
	double y3 = img.gpsPolygon.contour->vertex[3].y; 
	
	double distance_1 = distance(x1,y1,x2,y2);
	double distance_2 = distance(x1,y1,x3,y3);
	double x; double y;
	if(distance_1 >= distance_2){ distance_1 = x; distance_2 = y;} 
	else {distance_1 = y; distance_2 = x;}

	double img_x = img.image.cols;
	double img_y = img.image.rows;
    scale = img_y/y;
	return scale;
}

class GPSFeaturesFinder: public FeaturesFinder {
  public:
    void operator ()(const Mat &image, ImageFeatures &features) {
      vector<Point2f> gpsData;
      vector<KeyPoint> all;
      ImageWithGPS data;


      for (auto element : otherImages ){
        if ( element.image == image ) {
          data = element;
          break;
        }
      }

      for (unsigned int i = 0; i< otherimages.size(); i++){
        if(image == &otherimages[i]) continue;

        gpc_polygon* intersection;
        gpc_polygon_clip(&image.gpsPolygon, &otherimages[i].gpsPolygon,intersection);
		vector<double> coord = getExtremes(polygon.contour->vertex);

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
        descriptors.push_back(gpsData[i].x);
        descriptors.push_back(gpsData[i].y);
      }

      features.img_idx = img_idx;
      features.img_size =  image1.image.size();
      features.keypoints = all;
      features.descriptors = descriptors;
    }
};


double distance(double x1, double y1, double x2, double y2){
  return sqrt(pow(x2-x1,2)+pow(y2-y1,2));
}
double angle(double x1, double y1, double x2, double y2){
  double dy = y2-y1;
  double dx = x2-x1;
  return tan(dy/dx);
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

      vector<double> ul; ul.push_back(imageY*scale); ul.push_back(imageX*scale);
      vector<double> ur; ur.push_back(imageY*scale); ur.push_back((imageX+imageWidth)*scale);
      vector<double> br; br.push_back((imageY+imageHeight)*scale); br.push_back(imageX+imageWidth*scale);
      vector<double> bl; bl.push_back((imageY+imageHeight)*scale); bl.push_back(imageX*scale);
      vector<vector<double> > coords; coords.push_back(ul); coords.push_back(ur);coords.push_back(br); coords.push_back(bl); 
      resultImages[rows *j +i] = ImageWithGPS(result,coords);	  
    }
  }
  return resultImages;
}

ImageWithGPS iterativeStitch(ImageWithGPS accumulatedImage, vector<ImageWithGPS> newImages) {
  Mat result;
  Rect_<double> rect = accumulatedImage.rect;
  vector<Mat> newVec(newImages.size()+1);
  for(unsigned int i =0; i < newImages.size(); i++){
    if(newImages[i].rect.x < rect.x)
      rect.x = newImages[i].rect.x;
    if(newImages[i].rect.y < rect.y)
      rect.y = newImages[i].rect.y;
    if(newImages[i].rect.height+newImages[i].rect.y > rect.y+rect.height)
      rect.height = newImages[i].rect.height+newImages[i].rect.y-rect.y;
    if(newImages[i].rect.width+newImages[i].rect.x > rect.x+rect.width)
      rect.width = newImages[i].rect.width+newImages[i].rect.x-rect.x;
    newVec[i] = newImages[i].image;
  }
  Stitcher stitcher = Stitcher::createDefault(true);

  newVec.push_back(accumulatedImage.image);
  stitcher.stitch(newVec, result);
  return ImageWithGPS(result,rect);
}

int main(){

  ImageWithGPS accumulator, pano;
  vector<ImageWithGPS> images = getTestDataForImage(imread("image.jpg"),2,2,0.2,0.2,0.9);
  imwrite("a.jpg",images[0].image);
  imwrite("b.jpg",images[1].image);
  imwrite("c.jpg",images[2].image);
  imwrite("d.jpg",images[3].image);
  accumulator = images[3];
  images.pop_back();
  pano = iterativeStitch(accumulator,images);
  imwrite("result.jpg",pano.image);

}


