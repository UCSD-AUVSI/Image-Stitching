ImageWithGPS::ImageWithGPS(){}

ImageWithGPS::ImageWithGPS(Mat image, gpc_polygon gpsPolygon): image(image), gpsPolygon(gpsPolygon) {

  scale = findScale(image, gpsPolygon);
  ang = findAngleGPS(gpsPolygon.contour->vertex[0].x,
      gpsPolygon.contour->vertex[0].y,
      gpsPolygon.contour->vertex[1].x,
      gpsPolygon.contour->vertex[1].y);
}

Pixel ImageWithGPS::gpsToPixels(double lon, double lat){
  vector<int> result;  
  double angle = toRadians(ang);
  int x =(int) (((lon * cos(angle)) - (lat * sin(angle)))) / scale;
  int y =(int) (((lat * sin(angle)) - (lon * cos(angle)))) / scale;
  result.push_back(x);
  result.push_back(y);
  return result;
}

/**
 * Finds the scale (GPS Degrees / Pixels) of an image with a corresponding polygon
 */
double findScale(Mat img, gpc_polygon gpsPoly){

  double lat1 = gpsPoly.contour->vertex[0].x;
  double lon1 = gpsPoly.contour->vertex[0].y; 
  double lat2 = gpsPoly.contour->vertex[1].x;
  double lon2 = gpsPoly.contour->vertex[1].y;
  double lat3 = gpsPoly.contour->vertex[3].x;
  double lon3 = gpsPoly.contour->vertex[3].y; 

  double distance_12 = distance(lat1,lon1,lat2,lon2);
  double distance_13 = distance(lat1,lon1,lat3,lon3);
  double largeSideGPS = max (distance_12, distance_13);
  double smallSideGPS = min (distance_12, distance_13);
  double largeSidePixels = max(img.rows,img.cols);
  double smallSidePixels= min(img.rows,img.cols);

  double largeScale = largeSideGPS / largeSidePixels;
  double smallScale = smallSideGPS / smallSidePixels;

  return (largeScale + smallScale) / 2.0;

}
