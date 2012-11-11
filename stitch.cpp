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


using namespace std;
using namespace cv;
using namespace cv::detail;

ImageWithGPS::ImageWithGPS(){}
ImageWithGPS::ImageWithGPS(Mat image, gpc_polygon gpsPolygon){}

vector<int> ImageWithGPS::gpsToPixels(double lat, double lon){
  vector<int> result;   
  /*
  int x = (int)((lon - rect.x)/ rect.width * image.cols); 
  int y = (int)((rect.y - lat)/ rect.height* image.rows);

  result.push_back(x);
  result.push_back(y);*/


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

vector<double> getExtremes (vertex* vertices){
	double minLat = INT_MAX;
	double minLon = INT_MAX;
	double maxLat = INT_MIN;
	double maxLon = INT_MIN;
	for(int i = 0; i < 4; i++){
		if (vertex[i].x < minLon ) minLon = vertex[i].x;
		if (vertex[i].x > maxLon ) maxLon = vertex[i].x;
		if (vertex[i].y < minLat ) minLat = vertex[i].y;
	    if (vertex[i].y > maxLat ) maxLat = vertex[i].y;

}
}
vector<double> finding (gpc_polygon polygon){
  vector<double> result;
  
  double x1 = polygon.contour[0].vertex[0].x;
  double x2 = polygon.contour[0].vertex[1].x;
  double x3 = polygon.contour[0].vertex[2].x;
  double x4 = polygon.contour[0].vertex[3].x;
  double y1 = polygon.contour[0].vertex[0].y;
  double y2 = polygon.contour[0].vertex[1].y;
  double y3 = polygon.contour[0].vertex[2].y;
  double y4 = polygon.contour[0].vertex[3].y;
  double arrayX [] = {x1,x2,x3,x4};   
  double arrayY [] = {y1,y2,y3,y4};
  findMaxOrMin(arrayX);
  findMaxOrMin(arrayY);
  double maxLon = arrayX[3]-arrayX[0];
  double minLat = arrayY[2]-arrayY[1];
  double minLon = arrayX[2]-arrayX[1];
  double maxLat = arrayY[3]-arrayY[0];
  result.push_back(minLon);
  result.push_back(minLat);
  result.push_back(maxLon); 
  result.push_back(maxLat);
  
  return result;
}

ImageFeatures findIntersectionFeatures(ImageWithGPS image1, vector<ImageWithGPS> otherimages, int img_idx) {

  ImageFeatures result;
  vector<Point2f> gpsData;
  vector<KeyPoint> all;
  gpc_op op = GPC_INT;
  gpc_polygon polygon = image1.gpsPolygon;
  gpc_polygon resultPoly;
  vector<double> coord = finding(image1.gpsPolygon);

  for (unsigned int i = 0; i< otherimages.size(); i++){
    if(&image1 == &otherimages[i]) continue;
    gpc_polygon* intersection;
    gpc_polygon_clip(op, &image1.gpsPolygon, &polygon, &resultPoly);

     /* 
	float maxLat = (float)
	float maxLon = (float) coord.pop_back();
	float minLat = (float) coord.pop_back();
	float minLon = (float) coord.pop_back();
	
    float maxLon = (float) (gpsPolygon.intersection.x + 2 * gpsPolygon.intersection.width / 3);
    float maxLat = (float) (gpsPolygon.intersection.y - gpsPolygon.intersection.height / 3);
    float minLat = (float) (gpsPolygon.intersection.y - 2 * gpsPolygon.intersection.height / 3);
	*/
    vector<int> ul = image1.gpsToPixels(maxLon, minLat);
    vector<int> ur = image1.gpsToPixels(maxLon, maxLat);
    vector<int> bl = image1.gpsToPixels(minLon, minLat);
    vector<int> br = image1.gpsToPixels(minLon, maxLat);

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
    //ImageFeatures imageFeature1; imageFeature1.img_idx = img_idx1;
    //ImageFeatures imageFeature2; imageFeature2.img_idx = img_idx2;
    //imageFeature1.img_size = image1.image.size();
    //imageFeature2.img_size = image2.image.size();
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
  result.img_idx = img_idx;
  result.img_size =  image1.image.size();
  result.keypoints = all;
  result.descriptors = descriptors;
  return result;
}

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


