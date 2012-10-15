#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <vector>
#include "opencv2/stitching/stitcher.hpp"

using namespace std;
using namespace cv;

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