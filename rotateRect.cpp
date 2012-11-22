#include <highgui.h>
#include <cv.h>


#define PI 3.14159

using namespace cv;
/**
 * Return a largest Rectangle that will fit in a rotated image
 * @param imgWidth Width of image
 * @param imgHeight Height of Image
 * @param rotAngDeg Rotation angle in degrees
 * @param type 0 = Largest Area, 1 = Smallest Area, 2 = Widest, 3 = Tallest
 * @return
 */


double toRadians(double degrees){
  return PI * (degrees / 180);
}
Rect_<double> getLargestRectangle(double imageWidth, double imageHeight, double rotateAngleDeg, int type) {
  Rect_<double> rect;
  double imgWidth = imageWidth;
  double imgHeight = imageHeight;

  if (rotateAngleDeg == 0 || rotateAngleDeg == 180) {
    // Angle is 0. No change needed
    return Rect_<double>(0,0,imageWidth,imageHeight);
  }

  if (rotateAngleDeg == 90) {
    // Angle is 90. Width and height swapped
    return Rect_<double>(0,0,imageHeight,imageWidth);
  }

  if (rotateAngleDeg > 90) {
    // Angle > 90 therefore angle = 90 - ("+rotateAngleDeg+" - 90) = "+(90 - (rotateAngleDeg - 90))
    rotateAngleDeg = 90 - (rotateAngleDeg - 90);
  }

  double rotateAngle = toRadians(rotateAngleDeg);
  double sinRotAng = sin(rotateAngle);
  double cosRotAng = cos(rotateAngle);
  double tanRotAng = tan(rotateAngle);
  // Point 1 of rotated rectangle
  double x1 = sinRotAng * imgHeight;
  double y1 = 0;
  // Point 2 of rotated rectangle
  double x2 = cosRotAng * imgWidth + x1;
  double y2 = sinRotAng * imgWidth;
  // Point 3 of rotated rectangle
  double x3 = x2 - x1;
  double y3 = y2 + cosRotAng * imgHeight;
  // Point 4 of rotated rectangle
  double x4 = 0;
  double y4 = y3 - y2;
  // MidPoint of rotated image
  double midx = x2 / 2;
  double midy = y3 / 2;

  // Angle for new rectangle (based on image width and height)
  double imgAngle = atan(imgHeight / imgWidth);
  double imgRotAngle = atan(imgWidth / imgHeight);
  double tanImgAng = tan(imgAngle);
  double tanImgRotAng = tan(imgRotAngle);
  // X Point for new rectangle on bottom line
  double ibx1 = midy / tanImgAng + midx;
  double ibx2 = midy * tanImgAng + midx;

  // First intersecting lines
  // y = ax + b  ,  y = cx + d  ==>  x = (d - b) / (a - c)
  double a = y2 / x3;
  double b = tanRotAng * -x1;
  double c = -imgHeight / imgWidth;
  double d = tanImgAng * ibx1;

  // Intersecting point 1
  double ix1 = (d - b) / (a - c);
  double iy1 = a * ix1 + b;

  // Second intersecting lines
  c = -imgWidth / imgHeight;
  d = tanImgRotAng * ibx2;

  // Intersecting point 2
  double ix2 = (d - b) / (a - c);
  double iy2 = a * ix2 + b;

  // Work out smallest rectangle
  double radx1 = abs(midx - ix1);
  double rady1 = abs(midy - iy1);
  double radx2 = abs(midx - ix2);
  double rady2 = abs(midy - iy2);
  // Work out area of rectangles
  double area1 = radx1 * rady1;
  double area2 = radx2 * rady2;
  // Rectangle (x,y,width,height)
  Rect_<double> rect1(midx-radx1,midy-rady1,radx1*2,rady1*2);

  // Rectangle (x,y,width,height)
  Rect_<double> rect2(midx-radx2,midy-rady2,radx2*2,rady2*2);

  switch (type) {
    case 0: rect = (area1 > area2 ? rect1 : rect2); break;
    case 1: rect = (area1 < area2 ? rect1 : rect2); break;
    case 2: rect = (radx1 > radx2 ? rect1 : rect2); break;
    case 3: rect = (rady1 > rady2 ? rect1 : rect2); break;
  }

  return rect;
}
