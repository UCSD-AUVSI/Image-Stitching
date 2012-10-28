#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <vector>
#include "opencv2/stitching/stitcher.hpp"

#define IMAGES_PER_STITCH 4

using namespace std;
using namespace cv;
using namespace cv::detail;

class ImageWithGPS{
public:
  ImageWithGPS(Mat image, Rect_<double> rect): image(image), rect(rect) {}
  ImageWithGPS(Mat image, vector<vector<double> > corners): image(image) {
     
  }
  Mat image;
  Rect_<double> rect;
};

double distance(double x1, double y1, double x2, double y2){
  return sqrt(pow(x2-x1,2) + pow(y2-y1,2));
}

double angle(double x1, double y1, double x2, double y2){
  double dx = x2 - x1;
  double dy = y2 - y1;
  return tan(dy/dx);
}


vector<Mat> getTestDataForImage(Mat image,
    int rows,
    int columns,
    double horizontalOverlap,
    double verticalOverlap){
  vector<Mat> resultImages = vector<Mat>(rows * columns);
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
      resultImages[j * rows + i] = Mat(image,Range(imageY, imageY+imageHeight),Range(imageX,imageX +imageWidth));
    }
  }
  return resultImages;
}

Mat iterativeStitch(Mat accumulatedImage, vector<Mat> newImages) {
  Mat result;
  Stitcher stitcher = Stitcher::createDefault(false);
  newImages.push_back(accumulatedImage);
  stitcher.stitch(newImages,result);
  return result;
}

vector<ImageFeatures> findIntersectionFeatures(ImageWithGPS image1, ImageWithGPS image2) {
  vector<ImageFeatures> result;
  Rect_<double> intersection = image1.rect & image2.rect; 
  cout << "Intersection: " << intersection.x << "," << intersection.y << "," <<
       intersection.width << "," <<intersection.height <<endl;
  if (intersection.width == 0 || intersection.height == 0 ){
    cout <<"These images contain no intersection points";
  }

  double left = intersection.x + intersection.width / 3;
  double right = intersection.x + 2 * intersection.width / 3;
  double top = intersection.y + intersection.height / 3;
  double bottom = intersection.y + 2 * intersection.height / 3;
  
  return result;
}

int main(){

  /*Mat accumulator, pano;
  vector<Mat> images = getTestDataForImage(imread("image.jpg"),2,2,0.1,0.1);
  pano = images.back();
  images.pop_back();
  while ( images.size() > 0){
    vector<Mat> temp;
    for (int i = 0; i < IMAGES_PER_STITCH && images.size() > 0; i++){
      temp.push_back(images.back());
      images.pop_back();
    }
    pano = iterativeStitch(pano,temp);
  }
  imwrite("result.jpg",pano);*/
  Mat temp;
  Rect_<double> rect1(5,5,2,2);
  Rect_<double> rect2(0,0,2,2);
  ImageWithGPS image1 = ImageWithGPS(temp,rect1);
  ImageWithGPS image2 = ImageWithGPS(temp,rect2);
  findIntersectionFeatures(image1,image2);
}
