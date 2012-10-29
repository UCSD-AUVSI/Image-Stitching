


int _tmain(int argc, _TCHAR* argv[])
{
	return 0;
}
#include "stdafx.h"
#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <vector>
#include "opencv2/stitching/stitcher.hpp"
#include "rotateRect.cpp"
#include "opencv2/stitching/detail/matchers.hpp"
#include "stitch.h"

using namespace std;
using namespace cv;
using namespace cv::detail;


	ImageWithGPS::ImageWithGPS(Mat imag, vector<vector<double>> corners){
		double width = distance(corners[0][0], corners[0][1], corners[1][0], corners[1][1] );
		double height = distance(corners[1][0], corners[1][1], corners[2][0],corners[2][1]);
		double ang = angle(corners[0][0], corners[0][1], corners[1][0], corners[1][1] );
        rect = getLargestRectangle(width,height,ang,0);
	    Size size = Size(width, height);
		this -> image = rotateImage(imag, ang,size);
	}

	vector<int> ImageWithGPS::gpsToPixels(double lat, double lon ){
      vector<int> result;   
	  int x = (lon - rect.x)/ rect.width * image.cols; 
	  int y = (rect.y - lat)/ rect.height* image.rows; 
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

ImageFeatures findIntersectionFeatures(ImageWithGPS image1, vector<ImageWithGPS> otherimages, int img_idx) {
  
  ImageFeatures result;
  vector<Point2f> gpsData;
  vector<KeyPoint> all;

  for (int i = 0; i< otherimages.size(); i++){
	  if(&image1 == &otherimages[i]) continue;
	  Rect_<double> intersection = image1.rect & otherimages[i].rect; 
  cout << "Intersection: " << intersection.x << "," << intersection.y << "," <<
       intersection.width << "," <<intersection.height <<endl;
  if (intersection.width == 0 || intersection.height == 0 ){
    cout <<"These images contain no intersection points";
  }

  double minLon = intersection.x + intersection.width / 3;
  double maxLon = intersection.x + 2 * intersection.width / 3;
  double maxLat = intersection.y - intersection.height / 3;
  double minLat = intersection.y - 2 * intersection.height / 3;
  vector<int> ul = image1.gpsToPixels(maxLon, minLat);
  vector<int> ur = image1.gpsToPixels(maxLon, maxLat);
  vector<int> bl = image1.gpsToPixels(minLon, minLat);
  vector<int> br = image1.gpsToPixels(minLon, maxLat);
  Point2f ulPoint = Point2f(ul[0], ul[1]);
  Point2f urPoint = Point2f(ur[0], ur[1]);
  Point2f blPoint = Point2f(bl[0], bl[1]);
  Point2f brPoint = Point2f(br[0], br[1]);
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
  for(int i =0; i < gpsData.size(); i++){
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
	  vector<vector<double>> coords; coords.push_back(ul); coords.push_back(ur);coords.push_back(br); coords.push_back(bl); 
	  resultImages[rows * columns +j] = ImageWithGPS(result,coords);	  
     }
  }
  return resultImages;
}

ImageWithGPS iterativeStitch(ImageWithGPS accumulatedImage, vector<ImageWithGPS> newImages) {
  ImageWithGPS result;
  Stitcher stitcher = Stitcher::createDefault(true);
  newImages.push_back(accumulatedImage);
  stitcher.stitch(newImages,result);
    return result;
}

int main(){
	
  Mat accumulator, pano;
  vector<ImageWithGPS> images = getTestDataForImage(imread("image.jpg"),2,2,0.2,0.2,0.9);
  imwrite("a.jpg",images[0].image);
  imwrite("b.jpg",images[1].image);
  imwrite("c.jpg",images[2].image);
  imwrite("d.jpg",images[3].image);
  accumulator = images[3].image;
  images.pop_back();
  pano = iterativeStitch(accumulator,images);
  imwrite("result.jpg",pano);
  
}

 
