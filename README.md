Image-Stitching
===============

  Our image stitcher will use GPS information combined with visual attributes to
generate a large map from a series of images. Stitching could be done in realtime or
after a flight once all of the images have been generated. The stitcher will also be 
able to handle images that do not have GPS data associated with them, but the result
may not be stitched correctly.

**Architecture**

***Image Stitching Outline***

1. Load the images, along with their associated plane telemetry data into the 
   stitcher, creating a vector<ImageWithPlaneData>
2. Create a new cv::Stitcher object
3. Create a new GPSFeatureFinder. The featurefinder will contain the vector of 
   images with plane data. It will use match images that are sent to the stitcher
   with the plane data associated with them. It is important to note that the 
   stitcher must be loaded with all of the images that will be stitched together,
   or it will not be able to work correctly.
4. Make the GPSFeatureFinder the feature-finder for our stitcher.
5. Stitch the images

***GPSFeatureFinder Outline***

Each image to be stitched is passed to the stitcher individually. The images must
be passed in the order that they are in the vector<ImageWithPlaneData>, so that the
FeatureFinder can match the images. Note that each image passed in is a low-resolution
version of the original image.

For each image passed in

  - Associate the image with the the ImageWithPlaneData in the vector
  - Determine how much the image has been scaled down, store the ratio in variable `scale`
  - For each other image in the vector<ImageWithPlaneData>:
   - If this image and the other image intersect in physical space:
     - Find the pixel locations of four points in the intersection area.
     - Multiply each pixel position by `scale` to determine the pixel in the original image
     - Perform GeoReferening on each pixel position to determine the GPS coordinates of each pixel
      - Multiply each GPS coordinate by 1000 and convert to an integer to determine the
      descriptor for each pixel.
      - Add the pixel locations in the low-resolution image to the keypoints, and use the modified
      GPS coordinates as the descriptor
  - Return the keypoints and descriptors for the image

