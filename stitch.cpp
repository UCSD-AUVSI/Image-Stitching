#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <vector>
#include <opencv2/stitching/stitcher.hpp>
#include <opencv2/stitching/detail/matchers.hpp>
#include "MultiFeaturesFinder.h"
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



int main(int argc, char* argv[]){
  
  if ( argc > 2 ){
    Stitcher stitcher = stitcher.createDefault(true);
    vector<Mat> images;
    Mat first = imread(argv[1]);
    Mat pano;
    cv::resize(first,pano,Size(0,0),0.2,0.2);
    for(int i = 1; i < argc; i++){
      cout << "Adding: "<<argv[i] << endl;
      Mat image = imread(argv[i]);
      Mat resized;
      cv::resize(image,resized,Size(0,0),0.2,0.2);
      images.push_back(resized);
      images.push_back(pano);
      if (stitcher.stitch(images,pano) == Stitcher::OK){
        images = vector<Mat>();
        images.push_back(pano);
      }
    }
    imwrite("result.jpg",pano);
    cout <<"Done!"<<endl;
    getchar();
    return 0;
  }

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

  GPSFeaturesFinder* gpsFinder = new GPSFeaturesFinder(imagesWithData);
  SurfFeaturesFinder* surfFinder = new SurfFeaturesFinder();
  MultiFeaturesFinder* multiFinder = new MultiFeaturesFinder(surfFinder,gpsFinder);

  Stitcher stitcher = stitcher.createDefault(true);
  stitcher.setPanoConfidenceThresh(0.4);
  stitcher.setFeaturesFinder(multiFinder);
  stitcher.setFeaturesMatcher(new BestOf2NearestMatcher(true, 0.4f,1,1));

  stitcher.stitch(images,pano);
  imwrite("result.jpg",pano);
  cout <<"Done!"<<endl;
  getchar();
}
