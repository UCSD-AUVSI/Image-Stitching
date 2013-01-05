#ifndef ADJACENT_FEATURES_MATCHER_H
#define ADJACENT_FEATURES_MATCHER_H
#include <cv.h>
#include <opencv2/stitching/detail/matchers.hpp>
#include <opencv2/stitching/detail/exposure_compensate.hpp>

class AdjacentFeaturesMatcher: public cv::detail::FeaturesMatcher {
  public:
    AdjacentFeaturesMatcher();
  protected:  
    cv::detail::BestOf2NearestMatcher twoNearestMatcher;
    void match(const cv::detail::ImageFeatures &features1,
              const cv::detail::ImageFeatures &features2,
              cv::detail::MatchesInfo &matches_info);
};

#endif
