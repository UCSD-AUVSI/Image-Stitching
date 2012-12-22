#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cv.h>
#include <highgui.h>
#include <vector>
#include <opencv2/stitching/stitcher.hpp>
#include <opencv2/stitching/detail/matchers.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
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

vector<ImageWithPlaneData> getImagesWithData(vector<string> imageFilenames, string dataFilename){
  ifstream imageData(dataFilename.c_str()); 
  char buffer[1024];
  map<string, ImageWithPlaneData> dataMap;
  vector<string> parts;
  while (true){
    imageData.getline(buffer,1024);
    if (!buffer)
      break;

    boost::split(parts, buffer, boost::is_any_of(" \r"));

    if ( parts.size() == 1){
      break;
    }

    string filename = "IMG_" + parts[0] + ".JPG";

    /**
     * For each of the parts below, the first character of the segment is 
     * removed. This character is the 'R', 'P', 'Y', or 'A' indicator
     */
    double planeRoll = boost::lexical_cast<int>(parts[1].substr(1)) / 100.0; 
    double planePitch = boost::lexical_cast<int>(parts[2].substr(1)) / 100.0;
    double planeYaw = boost::lexical_cast<int>(parts[3].substr(1)) / 100.0;
    double planeAlt = boost::lexical_cast<int>(parts[4].substr(1)) / 100.0;


    int latWholePart = boost::lexical_cast<int>(parts[9]);
    int latFractionPart = boost::lexical_cast<int>(parts[10]);
    double planeLat = (double)latWholePart + (double)latFractionPart / 1000000;

    int lonWholePart = boost::lexical_cast<int>(parts[11]);
    int lonFractionPart = boost::lexical_cast<int>(parts[12]);
    double planeLon = (double)lonWholePart + (double)lonFractionPart / 1000000;


    double gimbalRoll = 0;
    double gimbalPitch = 0;

    dataMap[filename] = ImageWithPlaneData(
        cv::Mat(),
        planeLat,
        planeLon,
        planeAlt,
        planeRoll,
        planePitch,
        planeYaw,
        gimbalRoll,
        gimbalPitch);
  }

  vector<ImageWithPlaneData> imagesWithData;
  for (int i = 0; i < imageFilenames.size(); i++){
    if (dataMap.count(imageFilenames[i])){
      ImageWithPlaneData& imageWithData = dataMap[imageFilenames[i]];
      imageWithData.image = cv::imread(imageFilenames[i]);
      imagesWithData.push_back(imageWithData);
    }
  }

  return imagesWithData;
}

int main(int argc, char* argv[]){

  if (argc < 2){
    cout << "Usage: " << argv[0] << " planeDataFile image1 [image2 ...]\n";
    return 0;
  }

  /**
   * The first argument passed to Image-Stitcher should be the file with the plane data
   */
  string planeData(argv[1]);

  /**
   * Get the filenames for the rest of the images
   */
  vector<string> imageFilenames;
  for (int i = 2; i < argc; i++){
    imageFilenames.push_back(argv[i]);
  }

  vector<ImageWithPlaneData> imagesWithData = getImagesWithData(imageFilenames,planeData);

  cout << "Attempting to stitch " << imagesWithData.size() << " images.\n";

  int step = 0, imagesPerStep = 10;
  Stitcher stitcher = stitcher.createDefault(true);
  stitcher.setFeaturesFinder(new SurfFeaturesFinder(1000));
  stitcher.setExposureCompensator(new NoExposureCompensator());
  stitcher.setFeaturesMatcher(new BestOf2NearestMatcher(false, 0.2f)); 
  vector<Mat> images,completed, toStitch;
  for(int i = 0; i < imagesWithData.size(); i++){
    images[i] = imagesWithData[i].image;
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

