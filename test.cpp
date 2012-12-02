#include "test.h"
#include "gpc.h"
#include "DataTypes.h"
#include <iostream>

using namespace std;

void testGetExtremes(){
  cerr << "Testing getExtremes...";
  gpc_vertex bottomLeft; bottomLeft.x = 32; bottomLeft.y = -116;
  gpc_vertex bottomRight; bottomRight.x = 32;bottomRight.y = -116;
  gpc_vertex topRight; topRight.x = 33; topRight.y = -116;
  gpc_vertex topLeft; topLeft.x = 33; topLeft.y = -117;
  gpc_vertex vertices[] = {topLeft,topRight,bottomRight,bottomLeft};
  gpc_vertex_list* list = new gpc_vertex_list();
  list->num_vertices = 4;
  list->vertex = vertices;
  gpc_polygon polygon;
  polygon.num_contours = 1;
  polygon.hole=0;
  polygon.contour=list;
  GPSExtremes extremes = GPSExtremes(&polygon);
  assert(extremes.minLat == 32);    // Min Lat
  assert(extremes.minLon == -117);  // Min Lon
  assert(extremes.maxLat== 33);     // Max Lat
  assert(extremes.maxLon == -116);  // Max Lon
  cerr <<"Complete\n";
}
