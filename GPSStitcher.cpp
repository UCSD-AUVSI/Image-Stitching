#include <vector>
#include <opencv2/stitching/stitcher.hpp>
#include <opencv2/stitching/warpers.hpp>
#include "GPSStitcher.h"

using namespace cv;
using namespace cv::detail;
using namespace std;

bool GPSStitcher::stitch( InputArray images,
                                      OutputArray pano,
                                      vector<CameraParams> cameras,
                                      bool useFeatures){
  /**
   * Use these cameras
   */
  this->cameras_ = cameras;

  images.getMatVector(imgs_);

  cout <<"Matching images...\n";
  
  if (!prepareAndMatchImages(useFeatures)){
    return false;
  }

  cout <<"Images matched successfully.\n";

  /**
   * Compose Panorama
   */
  return composePanorama(vector<Mat>(), pano);
}

bool GPSStitcher::composePanorama(InputArray images, OutputArray pano){
  
  /**
   * Bundle Adjustment
   */
  LOGLN("Performing Bundle Adjustment");
  // (*bundle_adjuster_)(features_, pairwise_matches_, cameras_);

  /**
   * Focal calculations
   */
  LOGLN("Focal / Scale Calculations");
  vector<double> focals;
  for (size_t i = 0; i < cameras_.size(); ++i)
  {
    focals.push_back(cameras_[i].focal);
  }

  sort(focals.begin(), focals.end());
  if (focals.size() % 2 == 1)
    warped_image_scale_ = static_cast<float>(focals[focals.size() / 2]);
  else
    warped_image_scale_ = static_cast<float>(focals[focals.size() / 2 - 1] + focals[focals.size() / 2]) * 0.5f;

  LOGLN("Warping images... ");

  vector<Mat> imgs;
  images.getMatVector(imgs);
  Mat &pano_ = pano.getMatRef();

#if ENABLE_LOG
  int64 t = getTickCount();
#endif

  vector<Point> corners(imgs_.size());
  vector<Mat> masks_warped(imgs_.size());
  vector<Mat> images_warped(imgs_.size());
  vector<Size> sizes(imgs_.size());
  vector<Mat> masks(imgs_.size());

  // Prepare image masks
  for (size_t i = 0; i < imgs_.size(); ++i)
  {
    masks[i].create(seam_est_imgs_[i].size(), CV_8U);
    masks[i].setTo(Scalar::all(255));
  }

  // Warp images and their masks
  Ptr<detail::PlaneWarper> w(new detail::PlaneWarper(float(warped_image_scale_ * seam_work_aspect_)));
  for (size_t i = 0; i < imgs_.size(); ++i)
  {
    Mat_<float> K;
    cameras_[i].K().convertTo(K, CV_32F);
    K(0,0) *= (float)seam_work_aspect_;
    K(0,2) *= (float)seam_work_aspect_;
    K(1,1) *= (float)seam_work_aspect_;
    K(1,2) *= (float)seam_work_aspect_;

    corners[i] = w->warp(seam_est_imgs_[i], K, cameras_[i].R, cameras_[i].t, INTER_NEAREST, BORDER_REFLECT, images_warped[i]);
    sizes[i] = images_warped[i].size();

    w->warp(masks[i], K, cameras_[i].R, cameras_[i].t, INTER_NEAREST, BORDER_CONSTANT, masks_warped[i]);
  }

  vector<Mat> images_warped_f(imgs_.size());
  for (size_t i = 0; i < imgs_.size(); ++i)
    images_warped[i].convertTo(images_warped_f[i], CV_32F);

  LOGLN("Warping images, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec");

  // Find seams
  exposure_comp_->feed(corners, images_warped, masks_warped);
  seam_finder_->find(images_warped_f, corners, masks_warped);

  // Release unused memory
  seam_est_imgs_.clear();
  images_warped.clear();
  images_warped_f.clear();
  masks.clear();

  LOGLN("Compositing...");
#if ENABLE_LOG
  t = getTickCount();
#endif

  Mat img_warped, img_warped_s;
  Mat dilated_mask, seam_mask, mask, mask_warped;

  //double compose_seam_aspect = 1;
  double compose_work_aspect = 1;
  bool is_blender_prepared = false;

  double compose_scale = 1;
  bool is_compose_scale_set = false;

  Mat full_img, img;
  for (size_t img_idx = 0; img_idx < imgs_.size(); ++img_idx)
  {
    LOGLN("Compositing image #" << img_idx + 1);

    // Read image and resize it if necessary
    full_img = imgs_[img_idx];
    if (!is_compose_scale_set)
    {
      if (compose_resol_ > 0)
        compose_scale = min(1.0, sqrt(compose_resol_ * 1e6 / full_img.size().area()));
      is_compose_scale_set = true;

      // Compute relative scales
      //compose_seam_aspect = compose_scale / seam_scale_;
      compose_work_aspect = compose_scale / work_scale_;

      // Update warped image scale
      warped_image_scale_ *= static_cast<float>(compose_work_aspect);
      w = warper_->create((float)warped_image_scale_);

      // Update corners and sizes
      for (size_t i = 0; i < imgs_.size(); ++i)
      {
        // Update intrinsics
        cameras_[i].focal *= compose_work_aspect;
        cameras_[i].ppx *= compose_work_aspect;
        cameras_[i].ppy *= compose_work_aspect;

        // Update corner and size
        Size sz = full_img_sizes_[i];
        if (std::abs(compose_scale - 1) > 1e-1)
        {
          sz.width = cvRound(full_img_sizes_[i].width * compose_scale);
          sz.height = cvRound(full_img_sizes_[i].height * compose_scale);
        }

        Mat K;
        cameras_[i].K().convertTo(K, CV_32F);
        Rect roi = w->warpRoi(sz, K, cameras_[i].R,cameras_[i].t);
        corners[i] = roi.tl();
        sizes[i] = roi.size();
      }
    }
    if (std::abs(compose_scale - 1) > 1e-1)
      resize(full_img, img, Size(), compose_scale, compose_scale);
    else
      img = full_img;
    full_img.release();
    Size img_size = img.size();

    Mat K;
    cameras_[img_idx].K().convertTo(K, CV_32F);

    // Warp the current image
    w->warp(img, K, cameras_[img_idx].R, cameras_[img_idx].t, INTER_LINEAR, BORDER_REFLECT, img_warped);

    // Warp the current image mask
    mask.create(img_size, CV_8U);
    mask.setTo(Scalar::all(255));
    w->warp(mask, K, cameras_[img_idx].R, cameras_[img_idx].t, INTER_NEAREST, BORDER_CONSTANT, mask_warped);

    // Compensate exposure
    exposure_comp_->apply((int)img_idx, corners[img_idx], img_warped, mask_warped);

    img_warped.convertTo(img_warped_s, CV_16S);
    img_warped.release();
    img.release();
    mask.release();

    // Make sure seam mask has proper size
    dilate(masks_warped[img_idx], dilated_mask, Mat());
    resize(dilated_mask, seam_mask, mask_warped.size());

    mask_warped = seam_mask & mask_warped;

    if (!is_blender_prepared)
    {
      blender_->prepare(corners, sizes);
      is_blender_prepared = true;
    }

    // Blend the current image
    blender_->feed(img_warped_s, mask_warped, corners[img_idx]);
  }

  Mat result, result_mask;
  blender_->blend(result, result_mask);

  LOGLN("Compositing, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec");

  // Preliminary result is in CV_16SC3 format, but all values are in [0,255] range,
  // so convert it to avoid user confusing
  result.convertTo(pano_, CV_8U);

  return true;

}

bool GPSStitcher::prepareAndMatchImages(bool match)
{
    if ((int)imgs_.size() < 2)
    {
        LOGLN("Need more images");
        return false;
    }

    work_scale_ = 1;
    seam_work_aspect_ = 1;
    seam_scale_ = 1;
    bool is_work_scale_set = false;
    bool is_seam_scale_set = false;
    Mat full_img, img;
    features_.resize(imgs_.size());
    seam_est_imgs_.resize(imgs_.size());
    full_img_sizes_.resize(imgs_.size());

    LOGLN("Finding features...");
#if ENABLE_LOG
    int64 t = getTickCount();
#endif

    for (size_t i = 0; i < imgs_.size(); ++i)
    {
        full_img = imgs_[i];
        full_img_sizes_[i] = full_img.size();

        if (registr_resol_ < 0)
        {
            img = full_img;
            work_scale_ = 1;
            is_work_scale_set = true;
        }
        else
        {
            if (!is_work_scale_set)
            {
                work_scale_ = min(1.0, sqrt(registr_resol_ * 1e6 / full_img.size().area()));
                is_work_scale_set = true;
            }
            resize(full_img, img, Size(), work_scale_, work_scale_);
        }
        if (!is_seam_scale_set)
        {
            seam_scale_ = min(1.0, sqrt(seam_est_resol_ * 1e6 / full_img.size().area()));
            seam_work_aspect_ = seam_scale_ / work_scale_;
            is_seam_scale_set = true;
        }

        if (match){
            (*features_finder_)(img, features_[i]);
            features_[i].img_idx = (int)i;
            LOGLN("Features in image #" << i+1 << ": " << features_[i].keypoints.size());
        }

        resize(full_img, img, Size(), seam_scale_, seam_scale_);
        seam_est_imgs_[i] = img.clone();
    }

    // Do it to save memory
    features_finder_->collectGarbage();
    full_img.release();
    img.release();

    LOGLN("Finding features, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec");

    LOG("Pairwise matching");
#if ENABLE_LOG
    t = getTickCount();
#endif
    if (match){
      (*features_matcher_)(features_, pairwise_matches_, matching_mask_);
    }
    features_matcher_->collectGarbage();
    LOGLN("Pairwise matching, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec");

    if ((int)imgs_.size() < 2)
    {
        LOGLN("Need more images");
        return false;
    }

    return true;
}

GPSStitcher::GPSStitcher() {

  // Registration Resolution
  registr_resol_ = 1.0;
  
  // Seam Estimation Resolution
  seam_est_resol_ = 1.0;

  // Compositing Resolution
  compose_resol_ = ORIG_RESOL;

  // Pano Confidence Threshold
  conf_thresh_ = 0.4;
  
  // Do wave correction
  do_wave_correct_ = false;

  // Set features matcher
  features_matcher_ = new detail::BestOf2NearestMatcher(false,0.2f);

  // Set features finder
  features_finder_ = new detail::SurfFeaturesFinder(1000);

  // Set warper
  warper_ = new cv::PlaneWarper();

  // Set seam finder
  seam_finder_ = new detail::NoSeamFinder();
  // this->setSeamFinder(new detail::GraphCutSeamFinder(detail::GraphCutSeamFinderBase::COST_COLOR));

  // Exposure Compensator
  exposure_comp_ = new detail::NoExposureCompensator();

  // Blender
  blender_ = new detail::MultiBandBlender(false);

  // Bundle Adjuster 
  bundle_adjuster_ = new detail::BundleAdjusterReproj();
  //this->setBundleAdjuster(new detail::BundleAdjusterRay());
}

