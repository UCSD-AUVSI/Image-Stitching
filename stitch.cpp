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
  Stitcher stitcher = Stitcher::createDefault(true);
  newImages.push_back(accumulatedImage);
  if ( stitcher.stitch(newImages,result) ){
    cout <<"Stitch successful";
    return result;
    }
  else
    return accumulatedImage;
}

int main(){

  Mat accumulator, pano;
  vector<Mat> images = getTestDataForImage(imread("image.jpg"),2,2,0.2,0.2);
  imwrite("a.jpg",images[0]);
  imwrite("b.jpg",images[1]);
  imwrite("c.jpg",images[2]);
  imwrite("d.jpg",images[3]);
  accumulator = images[3];
  images.pop_back();
  pano = iterativeStitch(accumulator,images);
  imwrite("result.jpg",pano);
}
