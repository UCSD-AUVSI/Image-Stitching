#include "AdjacentFeaturesMatcher.h"
#include <iostream>
using namespace std;
using namespace cv::detail;

AdjacentFeaturesMatcher::AdjacentFeaturesMatcher(){
  cout << "AdjacentFeaturesMatcher initialized...\n";
}

void AdjacentFeaturesMatcher::match(const ImageFeatures &features1,
                                    const ImageFeatures &features2,
                                    MatchesInfo &matches_info) {
  if (abs(features1.img_idx - features2.img_idx) == 1){
    twoNearestMatcher(features1,features2,matches_info);
  } else {
    cout << "Not matching features because they are not adjacent.\n";
  }
}
