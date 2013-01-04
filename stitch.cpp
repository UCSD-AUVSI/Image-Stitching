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
#include "GPSStitcher.h"
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

void plotPositions(vector<CameraParams> cameras, string filename){
  ofstream outfile(filename);
  for ( auto camera : cameras ) {
    outfile << camera.t.at<float>(0,0) << " " << camera.t.at<float>(1,0) << " "
            << camera.t.at<float>(2,0) << endl;
  }
}

vector<ImageWithPlaneData> getImagesWithData(vector<string> imageFilenames, string dataFilename){

  cout << "Reading image data file...\n";

  ifstream imageData(dataFilename.c_str()); 
  char buffer[1024];
  map<string, ImageWithPlaneData> dataMap;
  vector<string> parts;

  while (true){
    imageData.getline(buffer,1024);
    if (!buffer)
      break;

    boost::split(parts, buffer, boost::is_any_of(" \r"));

    if ( parts.size() < 12){
      break;
    }

    string filename = "IMG_" + parts[0] + ".JPG";

    /**
     * For each of the parts below, the first character of the segment is 
     * removed. This character is the 'R', 'P', 'Y', or 'A' indicator
     */
    double planeRoll = boost::lexical_cast<int>(parts[1].substr(1)) / 1000.0; 
    double planePitch = boost::lexical_cast<int>(parts[2].substr(1)) / 1000.0;
    double planeYaw = boost::lexical_cast<int>(parts[3].substr(1)) / 1000.0;
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

  cout << dataMap.size() << " entries loaded.\n";

  vector<ImageWithPlaneData> imagesWithData(imageFilenames.size());
  for (int i = 0; i < imageFilenames.size(); i++){
    vector<string> parts;
    boost::split(parts, imageFilenames[i], boost::is_any_of("/"));
    string filename = parts.back();
    if (dataMap.count(filename)){
      ImageWithPlaneData& imageWithData = dataMap[filename];
      cout << "Loading " << filename << "...";
      imageWithData.image = cv::imread(imageFilenames[i]);
      cout << "Done\n";
      imagesWithData[i] = imageWithData;
    } else {
      cout << "WARNING: Data for " << filename << "does not exist.\n";
    }
  }

  return imagesWithData;
}

void parseArguments(int argc, char* argv[], GPSStitcherArgs& arguments, vector<string>& imageFilenames){
  for (int i = 2; i < argc; i++){
    string token(argv[i]);
    if ( token == "--reg_resol"){
      arguments.registrationResolution = boost::lexical_cast<double>(argv[i++]);
    } else if ( token == "--seam_est_resol"){
      arguments.seamEstimationResolution = boost::lexical_cast<double>(argv[i++]);
    } else if ( token == "--compose_resol"){
      arguments.compositingResolution = boost::lexical_cast<double>(argv[i++]);
    } else if ( token == "--conf_thresh"){
      arguments.confidenceThreshold = boost::lexical_cast<double>(argv[i++]);
    } else if ( token == "--wave-correct"){
      arguments.doWaveCorrect = true;
    } else if ( token == "--no-wave-correct"){
      arguments.doWaveCorrect = false;
    } else if ( token == "--no-features"){
      arguments.useFeatures = false;
    } else if ( token == "--no-seam-finder"){
      arguments.seamFinder = new detail::NoSeamFinder();
    } else if ( token == "--no-exposure-compensator"){
      arguments.exposureCompensator = new detail::NoExposureCompensator();
    } else if ( token == "--bundle-adjuster-ray"){
      arguments.bundleAdjuster == new detail::BundleAdjusterRay();
    } else {
      imageFilenames.push_back(token);
    }
  }
}

int main(int argc, char* argv[]){

  if (argc < 2){
    cout << "Usage: " << argv[0] << " planeDataFile image1 [image2 ...]\n";
    return 0;
  }

  /**
   * The first argument passed to Image-Stitcher should be the file with the plane data
   */
  string planeDataFilename(argv[1]);

  GPSStitcherArgs gpsArgs;
  vector<string> imageFilenames;


  /**
   * Get the command-line parameters
   */
  parseArguments(argc,argv,gpsArgs,imageFilenames);

  int numImages = imageFilenames.size();

  cout << "Preparing to stitch " << numImages << " images.\n";

  vector<ImageWithPlaneData> imagesWithData = getImagesWithData(imageFilenames,planeDataFilename);
  cout << "Images Loaded\n";

  double minLat = DBL_MAX;
  double minLon = DBL_MAX;
  for (auto imageWithData: imagesWithData) {
    minLat = min(minLat,imageWithData.latitude);
    minLon = min(minLon,imageWithData.longitude);
  }

  vector<CameraParams> cameras(numImages);

  for (int i = 0; i < numImages; i++ ){
    cameras[i] = imagesWithData[i].getCameraParams(minLat,minLon);
  }

  plotPositions(cameras,"positions");

  vector<Mat> images(numImages);
  for(int i = 0; i < numImages; i++){
    images[i] = imagesWithData[i].image;
  }

  Mat pano;
  GPSStitcher stitcher = GPSStitcher(gpsArgs);

  cout << "Beginning Stitch...\n";
  stitcher.stitch(images,pano,cameras,gpsArgs.useFeatures);
  cout << "Stitch Completed\n";

  cout << "Saving result.jpg\n";
  imwrite("result.jpg",pano);
  cout << "File saved.\n";
  
  return 0;
}

