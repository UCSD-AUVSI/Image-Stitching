#ifndef UTIL_H
#define UTIL_H

#include <cv.h>
#include <vector>
struct ImageWithPlaneData;

std::vector<ImageWithPlaneData> getTestDataForImage(cv::Mat image,
    int rows,
    int columns,
    double horizontalOverlap,
    double pixelsPerMeter,
    double minLat,
    double minLon);

void printKeyPoint(cv::KeyPoint keypoint);

double toRadians(double degrees);

double toDegrees(double radians);

#endif
