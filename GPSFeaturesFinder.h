#ifndef GPS_FEATURES_FINDER_H
#define GPS_FEATURES_FINDER_H
#include <cv.h>
#include <vector>
#include "opencv2/stitching/stitcher.hpp"
#include "opencv2/stitching/detail/matchers.hpp"

using namespace cv;
using namespace cv::detail;
using std::vector;

class ImageWithGPS;

class GPSFeaturesFinder : public FeaturesFinder {
  public:
    /**
     * Constructor: Provide all of the images with GPS data for the images that need to 
     * be stitched.
     */
    GPSFeaturesFinder(vector<ImageWithGPS> imagesWithGPS): image_index(-1),
                                                    imagesWithGPS(imagesWithGPS) {}

    /**
     * Finds the ImageFeatures. Calls operator()
     */
    void find(const Mat &image, ImageFeatures &features){
      (*this)(image,features);
    }

    /**
     * Finds the ImageFeatures. This function is called for every image that
     * gets stitched. It tries to find coresponing GPS points between this image and 
     * every other image. For every point in this image that matches a neighboring image
     * a keypoint is added to `features` and a row in the descriptors column is created
     */
    void operator ()(const Mat &image, ImageFeatures &features);

  private:
    /**
     * The index of the image that is currently being examined. This element in
     * imagesWithGPS at the corresponding index has the associated
     * full-resolution image and GPS data
     */
    int image_index; 

    /**
     * This vector holds the images that are being stitched together, along with their
     * corresponding GPS data. This vector is used to find GPS features of the images
     */
    vector<ImageWithGPS> imagesWithGPS; 
};

#endif
