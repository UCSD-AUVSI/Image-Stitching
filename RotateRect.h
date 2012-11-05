#ifndef ROTATERECT_H
#define ROTATERECT_H

#include <highgui.h>
#include <cv.h>

#define PI 3.14159

using namespace cv;
double toRadians(double degrees);
Rect_<double> getLargestRectangle(double imageWidth, double imageHeight, double rotateAngleDeg, int type);


#endif
