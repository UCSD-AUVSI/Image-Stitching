#ifndef GPS_STITCHER_H
#define GPS_STITCHER_H

#include <cv.h>
#include <opencv2/stitching/stitcher.hpp>
#include "Datatypes.h"

class GPSStitcher {
public:
  enum { ORIG_RESOL = -1 };
  bool stitch(   cv::InputArray images,
                 cv::OutputArray pano,
                 std::vector<cv::detail::CameraParams> cameras,
                 bool useFeatures);
  bool prepareAndMatchImages(bool match);
  GPSStitcher(GPSStitcherArgs args);
  bool composePanorama(cv::InputArray images, cv::OutputArray pano, bool bundleAdjust);

  double registr_resol_;
  double seam_est_resol_;
  double compose_resol_;
  double conf_thresh_;
  cv::Ptr<cv::detail::FeaturesFinder> features_finder_;
  cv::Ptr<cv::detail::FeaturesMatcher> features_matcher_;
  cv::Mat matching_mask_;
  cv::Ptr<cv::detail::BundleAdjusterBase> bundle_adjuster_;
  bool do_wave_correct_;
  cv::detail::WaveCorrectKind wave_correct_kind_;
  cv::Ptr<cv::WarperCreator> warper_;
  cv::Ptr<cv::detail::ExposureCompensator> exposure_comp_;
  cv::Ptr<cv::detail::SeamFinder> seam_finder_;
  cv::Ptr<cv::detail::Blender> blender_;

  std::vector<cv::Mat> imgs_;
  std::vector<std::vector<cv::Rect> > rois_;
  std::vector<cv::Size> full_img_sizes_;
  std::vector<cv::detail::ImageFeatures> features_;
  std::vector<cv::detail::MatchesInfo> pairwise_matches_;
  std::vector<cv::Mat> seam_est_imgs_;
  std::vector<int> indices_;
  std::vector<cv::detail::CameraParams> cameras_;
  double work_scale_;
  double seam_scale_;
  double seam_work_aspect_;
  double warped_image_scale_;
};

#endif
