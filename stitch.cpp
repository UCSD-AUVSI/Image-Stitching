#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <vector>
#include "opencv2/stitching/stitcher.hpp"

using namespace std;
using namespace cv;

vector<Mat> getTestDataForImage(Mat image,
    int rows,
    int columns,
    int horizontalOverlap,
    int verticalOverlap){
  vector<Mat> resultImages = vector<Mat>(rows * columns);
  int normalWidth = image.cols / columns;
  int normalHeight = image.rows / rows;
  int overlapWidth = normalWidth * horizontalOverlap;
  int overlapHeight = normalHeight * verticalOverlap;
  int imageX, imageY, imageWidth, imageHeight;
  for (int j = 0; j < rows; j++){
    for (int i = 0; i < columns; i++){
      imageX = min(i * normalWidth - overlapWidth,0);
      if (i == 0){
        imageWidth = min(normalWidth + overlapWidth, image.cols);
      } else {
        imageWidth = min(normalWidth + 2 * overlapWidth,image.cols - imageX);
      }
      imageY = min(i * normalHeight - overlapHeight,0);
      if (i == 0){
        imageHeight = min(normalHeight + overlapHeight, image.rows);
      } else {
        imageHeight = min(normalHeight + 2 * overlapHeight ,image.rows - imageY);
      }
      resultImages[j * rows + i] = Mat(image,Range(imageY, imageY+imageHeight),Range(imageX,imageX +imageWidth));
    }
  }
  return resultImages;
}
int main(){

  Mat image1 = imread("image1.jpg");
  Mat image2 = imread("image.jpg");
  Mat image3 = imread("image3.jpg");
  Mat image4 = imread("image4.jpg");

  //Stitcher* stitcher = new Stitcher();
  Mat pano;
  vector<Mat> images = vector<Mat>(4);
  images[0] = image2;
  images[1] = image1;
  images[2] = image3;
  images[3] = image4;

  Stitcher stitcher = Stitcher::createDefault(true);
  stitcher.stitch(images,pano);
  imwrite("result.jpg",pano);
  system("open result.jpg");
}
