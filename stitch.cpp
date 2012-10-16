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
	for (int j = 0; j < rows; j++){
		for (int i = 0; i < cols; i++){
			int imageX, imageY, imageWidth, imageHeight;
			if (i == 0){
				imageX = 0;
				imageWidth = image->cols / image * ((horizontalOverlap-1)/2+1);
			} else if (i == cols- 1){
				imageWidth = image->cols / image * ((horizontalOverlap-1)/2+1);
				imageX = image->cols - imageWidth;
			}
			
			if (i == 0){
				ImageY = 0;
				imageHeight = image->cols / image * ((horizontalOverlap-1)/2+1);
			} else if (i == cols- 1){
				imageHeight = image->cols / image * ((horizontalOverlap-1)/2+1);
				ImageY = image->cols - imageHeight;
			}
		}
	}
								}
int main(){
	
	Mat image1 = imread("image1.jpg");
	Mat image2 = imread("image2.jpg");
	Mat image3 = imread("image3.jpg");
	Mat image4 = imread("image4.jpg");

	//Stitcher* stitcher = new Stitcher();
	Mat pano;
	vector<Mat> images = vector<Mat>(4);
	images[0] = image1;
	images[1] = image2;
	images[2] = image3;
	images[3] = image4;

	Stitcher stitcher = Stitcher::createDefault(true);
	stitcher.stitch(images,pano);
	imwrite("result.jpg",pano);
	system("open result.jpg");
}
