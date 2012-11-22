#ifndef STITCH_H
#define STITCH_H
#include <cv.h>

class ImageWithPlaneData;

/**
 * Converts a distance in meters to a distance in GPS degrees. This function is not very
 * accurate
 */
double metersToGPS(double meters);

/**
 * Degrees to radians
 */
double toRadians(double degrees);

/**
 * Radians to degrees
 */
double toDegrees(double radians);

void testGetExtremes();

std::vector<ImageWithPlaneData> getTestDataForImage(cv::Mat image, int rows, int columns,
                                              double horizontalOverlap, double pixelsPerMeter,
                                              double minLat, double minLon);

int main();
#endif


