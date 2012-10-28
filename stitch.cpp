#include "Stdafx.h"


int _tmain(int argc, _TCHAR* argv[])
{
	return 0;
}

#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <vector>
#include "opencv2/stitching/stitcher.hpp"

using namespace std;
using namespace cv;

class ImageWithGPS{
	Mat image;
	Rect_<double> rect;
	vector<vector<double>> corners;
	ImageWithGPS(Mat image, vector<vector<double>> corners): image(image){
		double width = distance(corners[0][0], corners[0][1], corners[1][0], corners[1][1] );
		double height = distance(corners[1][0], corners[1][1], corners[2][0],corners[2][1]);
		double angle = angle(corners[0][0], corners[0][1], corners[1][0], corners[1][1] );
        rect = getLargesRectangle(width,height,angle,0);
	} 
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

double distance(double x1, double y1, double x2, double y2){
    return sqrt(pow(x2-x1,2)+pow(y2-y1,2));
}
double angle(double x1, double y1, double x2, double y2){
	double dy = y2-y1;
	double dx = x2-x1;
	return tan(dy/dx);
}

vector<Mat> getTestDataForImage(Mat image,
    int rows,
    int columns,
    double horizontalOverlap,
    double verticalOverlap,
	double scale){
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

Mat iterativeStitch(Mat accumulatedImage, vector<Mat> newImages) {
  Mat result;
  Stitcher stitcher = Stitcher::createDefault(true);
  newImages.push_back(accumulatedImage);
  stitcher.stitch(newImages,result);
    return result;
}

int main(){
	
  Mat accumulator, pano;
  vector<Mat> images = getTestDataForImage(imread("image.jpg"),2,2,0.2,0.2,0.9);
  imwrite("a.jpg",images[0]);
  imwrite("b.jpg",images[1]);
  imwrite("c.jpg",images[2]);
  imwrite("d.jpg",images[3]);
  accumulator = images[3];
  images.pop_back();
  pano = iterativeStitch(accumulator,images);
  imwrite("result.jpg",pano);
  
}

 
