#include <vector>
#include <opencv2/stitching/stitcher.hpp>
#include <opencv2/stitching/warpers.hpp>
#include "GPSStitcher.h"

using namespace cv;
using namespace cv::detail;
using namespace std;

Stitcher::Status GPSStitcher::gpsStitch(InputArray images,
                                        OutputArray pano,
                                        vector<CameraParams> cameras){
  this->cameras_ = cameras;
  /**
   * Do focal calculations
   */
  vector<double> focals;
  LOGLN("About to calculate focals");
  for (size_t i = 0; i < cameras_.size(); ++i)
  {
    focals.push_back(cameras_[i].focal);
  }

  sort(focals.begin(), focals.end());
  if (focals.size() % 2 == 1)
    warped_image_scale_ = static_cast<float>(focals[focals.size() / 2]);
  else
    warped_image_scale_ = static_cast<float>(focals[focals.size() / 2 - 1] + focals[focals.size() / 2]) * 0.5f;

  /**
   * Wave Corrections
   */
  /**
  if (do_wave_correct_)
  {
    vector<Mat> rmats;
    for (size_t i = 0; i < cameras_.size(); ++i)
      rmats.push_back(cameras_[i].R);
    detail::waveCorrect(rmats, wave_correct_kind_);
    for (size_t i = 0; i < cameras_.size(); ++i)
      cameras_[i].R = rmats[i];
  }
  **/


  images.getMatVector(imgs_);
  rois_ = vector<vector<Rect> >();
  Status status;
  cout <<"Matching images...\n";
  if ((status = matchImages()) != OK)
    return status;
  cout <<"Images matched successfully.\n";

  /**
   * Compose Panorama
   */
  return gpsComposePanorama(vector<Mat>(), pano);
}

Stitcher::Status GPSStitcher::gpsComposePanorama(InputArray images, OutputArray pano){
  LOGLN("Warping images (auxiliary)... ");

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
    LOGLN("Compositing image #" << indices_[img_idx] + 1);

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

  return OK;

}

GPSStitcher::GPSStitcher(): Stitcher() {
  this->setRegistrationResol(0.6);
  this->setSeamEstimationResol(0.1);
  this->setCompositingResol(ORIG_RESOL);
  this->setPanoConfidenceThresh(0.4);
  this->setWaveCorrection(false);
  this->setFeaturesMatcher(new detail::BestOf2NearestMatcher(false,0.2f));
  this->setFeaturesFinder(new detail::SurfFeaturesFinder(1000));
  this->setWarper(new cv::PlaneWarper());
  this->setSeamFinder(new detail::GraphCutSeamFinder(detail::GraphCutSeamFinderBase::COST_COLOR));
  this->setExposureCompensator(new detail::NoExposureCompensator());
  this->setBlender(new detail::MultiBandBlender(false));
}
