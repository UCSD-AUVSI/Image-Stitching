#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <vector>
#include <opencv2/stitching/stitcher.hpp>
#include <opencv2/stitching/detail/matchers.hpp>
#include "stitch.h"
#include "GPSFeaturesFinder.h"
#include "gpc.h"
#include "DataTypes.h"
#include "camera.h"
#include "util.h"
#include "test.h"

#ifdef __WIN32__
#include "gpc.c"
#endif

using namespace std;
using namespace cv;
using namespace cv::detail;

int main(){

  testGetExtremes();

  Mat pano;
  
  vector<ImageWithPlaneData> imagesWithData = getTestDataForImage(
    imread("image.jpg"),      // image
    2,                        // rows
    2,                        // columns
    0.4,                      // horizontal overlap,
    1.0,                      // pixels per meter
    32.0,                     // minimum latitude
    -117.0);                  // minimum longitude

  imwrite("a.jpg",imagesWithData[0].image);
  imwrite("b.jpg",imagesWithData[1].image);
  imwrite("c.jpg",imagesWithData[2].image);
  imwrite("d.jpg",imagesWithData[3].image);

  vector<Mat> images;
  images.push_back(imagesWithData[0].image);
  images.push_back(imagesWithData[1].image);
  images.push_back(imagesWithData[2].image);
  images.push_back(imagesWithData[3].image);

  Stitcher stitcher = stitcher.createDefault(true);
  stitcher.setPanoConfidenceThresh(0.4);
  stitcher.setFeaturesFinder(cv::Ptr<FeaturesFinder>(new GPSFeaturesFinder(imagesWithData)));
  stitcher.setFeaturesMatcher(new BestOf2NearestMatcher(true, 0.4f,1,1));

  stitcher.stitch(images,pano);
  imwrite("result.jpg",pano);
  cout <<"Done!"<<endl;
  getchar();
}
