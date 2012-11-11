#ifndef STITCH_H
#define STITCH_H
#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <vector>
#include "opencv2/stitching/stitcher.hpp"
#include "opencv2/stitching/detail/matchers.hpp"

using namespace std;
using namespace cv;
using namespace cv::detail;

class ImageWithGPS{
  public:
    Mat image;
    vector<vector<double> > corners;
    ImageWithGPS();
    ImageWithGPS(Mat image, vector<vector<double> > corners);
    vector<int> gpsToPixels(double lat, double lon );
  private:
    double scale;
};
Mat rotateImage(const Mat &source, double angle, Size size);
ImageFeatures findIntersectionFeatures(ImageWithGPS image1, 
    vector<ImageWithGPS> otherimages, int img_idx);


double distance(double x1, double y1, double x2, double y2);
double angle(double x1, double y1, double x2, double y2);
vector<ImageWithGPS> getTestDataForImage(Mat image,
    int rows,
    int columns,
    double horizontalOverlap,
    double verticalOverlap,
    double scale);
ImageWithGPS iterativeStitch(ImageWithGPS accumulatedImage, vector<ImageWithGPS> newImages);
int main();

#endif


