#include <iostream>
#include <cstdlib>
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
    //// ITERATIVE
      int step = 0, imagesPerStep = 50;
      Stitcher stitcher = stitcher.createDefault(true);
      stitcher.setFeaturesFinder(new SurfFeaturesFinder(500));
      stitcher.setExposureCompensator(new NoExposureCompensator());
      stitcher.setFeaturesMatcher(new BestOf2NearestMatcher(false, 0.2f)); 
      vector<Mat> images,completed, toStitch;
      for (int i = 1; i < argc; i++){
        cout << "Adding: "<<argv[i] << endl;
        Mat image = imread(argv[i]);
        Mat resized;
        cv::resize(image,resized,Size(0,0),0.2,0.2);
        images.push_back(resized);
      }
      Mat pano;
      while(images.size() > 1){
        for(int i = 1; i < images.size(); i++){
          toStitch.push_back(images[i]);
          if (i % imagesPerStep == 0 || i == images.size()-1 ){
            if (stitcher.stitch(toStitch,pano) == Stitcher::OK){
              Mat temp;
              pano.copyTo(temp);
              completed.push_back(temp);
            } else {
              for (int i = 0; i < toStitch.size(); i++){
                completed.push_back(toStitch[i]);
              }
              cerr << "STITCHING FAILED!";
            }
            toStitch = vector<Mat>();
          }
        }
        cout <<"Completed step " << step++ << endl;
        for (int i = 0; i < completed.size(); i++){
          char index[10], _step[10];
          sprintf(index,"%d",i);
          sprintf(_step,"%d",step);
          imwrite("image" + string(index) +"_" + string(_step) + ".jpg", completed[i]);
        }
        images = completed;
        completed = vector<Mat>();
    }

    if ( images.size() == 1){
      imwrite("pano.jpg",images[0]);
    }
    // NON-Iterative
    /**
    Mat pano;
    vector<Mat> images;
    for (int i = 1; i < argc; i++){
      Mat image = imread(argv[i]);
      Mat resized;
      cv::resize(image,resized,Size(0,0),0.2,0.2);
      images.push_back(resized);
    }
    Stitcher stitcher = stitcher.createDefault(true);
    stitcher.stitch(images,pano);
    **/
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
    0.1,                      // pixels per meter
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
  stitcher.setFeaturesFinder(gpsFinder);
  stitcher.setFeaturesMatcher(new BestOf2NearestMatcher(true, 0.4f,1,1));

  stitcher.stitch(images,pano);
  imwrite("result.jpg",pano);
  cout <<"Done!"<<endl;
  getchar();
}
