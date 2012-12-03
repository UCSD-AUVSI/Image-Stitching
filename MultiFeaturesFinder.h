#ifndef MULTI_FEATURES_FINDER_H
#define MULTI_FEATURES_FINDER_H

#include <cv.h>
#include <vector>
#include <opencv2/stitching/stitcher.hpp>
#include <opencv2/stitching/detail/matchers.hpp>
#include "GPSFeaturesFinder.h"

using namespace cv;
using namespace cv::detail;

class MultiFeaturesFinder : public FeaturesFinder {
  public:
    MultiFeaturesFinder(SurfFeaturesFinder* surfFinder, GPSFeaturesFinder* gpsFinder):
      surfFinder(surfFinder), gpsFinder(gpsFinder) {}

    /**
     * Finds the ImageFeatures. Calls operator()
     */
    void find(const Mat &image, ImageFeatures &features){
      (*this)(image,features);
    }

    /**
     * Finds the ImageFeatures. This function is called for every image that
     * gets stitched. It finds the features using both of the FeaturesFinders and
     * combines them 
     */
    void operator ()(const Mat &image, ImageFeatures &features){
      (*surfFinder)(image,features);
      (*gpsFinder)(image,features);
    }
  private:
    FeaturesFinder *surfFinder, *gpsFinder;
};

#endif
