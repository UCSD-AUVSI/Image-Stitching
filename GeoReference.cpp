#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#pragma warning(push)
#pragma warning(disable:4996 4267)
#include <cv.h>
#pragma warning(pop)
#include "GeoReference.h"

#define GIMBAL_YAW		0.0
#define PI_TO_RAD M_PI / 180.0

using namespace std;
using namespace Vision;

/*
 * Determines if two numbers are very close to each other
 */
inline bool veryNearlyEqual(double one, double two){
	return (fabs(one-two) < 0.000001);
}

bool Vision::approxEqual(double one, double two)
{
	if (fabs(one - two) > 0.001)
		return false;
	else 
		return true;
}

/**
	Converts a number in meters to the equivalent number of GPS degrees
*/
double GeoReference::metersToGPS(double meters){
	return meters / 111122;
}

/**
	Converts a number of GPS degrees to the equivalent number of meters
*/
double GeoReference::GPStoMeters(double gps){
	return gps * 111122;
}

double cosd(double input)
{
	return cos(input*M_PI/180.0);
}

double sind(double input)
{
	return sin(input*M_PI/180.0);
}

double atand(double input)
{
	return atan(input)*180.0/M_PI;
}

/**
  intersects the plane given by the points {p1, p2, p3}
  with the line given by the points {p4, p5}
  This function uses the math given at 
  http://mathworld.wolfram.com/Line-PlaneIntersection.html
  Written by Tim Wheeler
**/
cv::Mat GeoReference::intersectLinePlane(cv::Mat p1, cv::Mat p2, cv::Mat p3, cv::Mat p4, cv::Mat p5){
  // Extract for convenience
   double x1 = p1.ptr<double>(0)[1];
   double x2 = p2.ptr<double>(0)[1];
   double x3 = p3.ptr<double>(0)[1];
   double x4 = p4.ptr<double>(0)[1];
   double x5 = p5.ptr<double>(0)[1];
   double y1 = p1.ptr<double>(0)[2];
   double y2 = p2.ptr<double>(0)[2];
   double y3 = p3.ptr<double>(0)[2];
   double y4 = p4.ptr<double>(0)[2];
   double y5 = p5.ptr<double>(0)[2];
   double z1 = p1.ptr<double>(0)[3];
   double z2 = p2.ptr<double>(0)[3];
   double z3 = p3.ptr<double>(0)[3];
   double z4 = p4.ptr<double>(0)[3];
   double z5 = p5.ptr<double>(0)[3];

   double a[4][4] = { {1 ,1 ,1 ,1 },
                      {x1,x2,x3,x4},
                      {y1,y2,y3,y4},
                      {z1,z2,z3,z4},
                    };
   double b[4][4] = { {1 ,1 ,1 ,0 },
                      {x1,x2,x3,x5 - x4},
                      {y1,y2,y3,y5 - y4},
                      {z1,z2,z3,z5 - z4},
                    };

  cv::Mat A = cv::Mat(4, 4, CV_64F, a).inv();
  cv::Mat B = cv::Mat(4, 4, CV_64F, b).inv();

  double t = - cv::determinant(A) / cv::determinant(B);
  double x = x4 + (x5 - x4)*t;
  double y = y4 + (y5 - y4)*t;
  double z = z4 + (z5 - z4)*t;
  return cv::Mat_<double>(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1;
}

double GeoReference::scalarProjection(cv::Mat A, cv::Mat B){
  cv::Mat bNorm;
  cv::normalize(B,bNorm);
  return A.dot(bNorm);
}

double GeoReference::distanceBetweenGPS(double lat1, double lon1, double lat2, double lon2)
{
	double radius = 6378000; // radius of earth!
	double deltaLat = lat2 - lat1;
	double deltaLon = lon2 - lon1;
	double a = sind(deltaLat/2)*sind(deltaLat*2) + cosd(lat1)*cosd(lat2)*sind(deltaLon/2)*sind(deltaLon*2);
	double c = 2*atan2(sqrt(a), sqrt((1-a)));
	double d = radius*c;

	return d;
}
cv::Mat GeoReference::Quaternion(double theta, double X, double Y, double Z)
{
	double mag = sqrt(X*X + Y*Y + Z*Z);
	double x = X/mag;
	double y = Y/mag;
	double z = Z/mag;

	double quatarr[4] = {cos(theta/2.0), x * sin(theta/2.0),  y * sin(theta/2.0),  z * sin(theta/2.0) };
	cv::Mat Quat(1, 4, CV_64FC1, quatarr );

	double magnitude = 0.0;
	for (int i = 0; i < 4; i++) 
		magnitude += Quat.at<double>(0,i)*Quat.at<double>(0,i);

	magnitude = sqrt(magnitude);

	for (int i = 0; i < 4; i++) 
		Quat.at<double>(0,i) = Quat.at<double>(0,i)/magnitude;

	return Quat.t();


}

cv::Mat GeoReference::Quaternion_Transform(cv::Mat Orig_Vector, cv::Mat Quat)
{
	cv::Mat Q = Quat;
	double o = Q.at<double>(0,0);
	double x = Q.at<double>(1,0);
	double y = Q.at<double>(2,0);
	double z = Q.at<double>(3,0);

	double transferarr[9] = {x*x + o*o - y*y - z*z, 2.0*(x*y + z*o), 2.0*(x*z-y*o),
							2.0*(x*y-z*o), y*y + o*o - x*x - z*z, 2.0*(y*z + x*o),
							2.0*(x*z+y*o), 2.0*(y*z-x*o), z*z + o*o - x*x - y*y};
	cv::Mat Transfer = cv::Mat(3, 3, CV_64FC1, transferarr).t();

	return Transfer*Orig_Vector;

}

cv::Mat GeoReference::ECEF_to_NED(cv::Mat ECEF, double Latitude, double Longitude)
{
	double sinlat = sin(Latitude);
	double coslat = cos(Latitude);
	double sinlon = sin(Longitude);
	double coslon = cos(Longitude);

	double transferarr[9] = {-sinlat*coslon, -sinlat*sinlon, coslat,
							-sinlon,		coslon,			 0.0,
							-coslat*coslon, -coslat*sinlon, -sinlat};
	cv::Mat Transfer = cv::Mat(3, 3, CV_64FC1, transferarr);

	return Transfer*ECEF;

}

cv::Mat GeoReference::NED_to_ECEF(cv::Mat NED, double Latitude, double Longitude)
{
	double sinlat = sin(Latitude);
	double coslat = cos(Latitude);
	double sinlon = sin(Longitude);
	double coslon = cos(Longitude);

	double transferarr[9] = {-sinlat*coslon, -sinlon, -coslat*coslon,
							-sinlat*sinlon, coslon, -coslat*sinlon,
							coslat,			0.0,	-sinlat};
	cv::Mat Transfer = cv::Mat(3, 3, CV_64FC1, transferarr);

	return Transfer*NED;
}

cv::Mat GeoReference::ECEF_to_GEO(cv::Mat ECEF, double flatness, double eccentricity, double semi_major_axis)
{


	double X = ECEF.at<double>(0,0);
	double Y = ECEF.at<double>(1,0);
	double Z = ECEF.at<double>(2,0);

	double f = flatness;
	double e = eccentricity;
	double e2 = e*e;
	double a = semi_major_axis;

	double h = 0.0;
	double N = a;
	double p = sqrt(X*X + Y*Y);
	double lon = atan(Y/X);
	double lat = 0.0;
	double delta = 1;
	double oldlat = 0;

	for (int i = 0; i < 15 && delta > 0.00000000001; i++)
	{
		double sin_lat = Z / (N*(1.0 - e2) + h);
		lat = atan((Z + e2*N*sin_lat) / p);
		N = a / sqrt(1.0 - e2*sin(lat)*sin(lat));
		h = p / cos(lat) - N;

		delta = abs(lat-oldlat); oldlat = lat;
		//PRINT("lat:"+lat+" N:"+N+" h:"+h+" delta:"+delta);
	}


	cv::Mat New_Vector(3, 1, CV_64FC1 );
	New_Vector.at<double>(0,0) = lat;
	New_Vector.at<double>(1,0) = lon;
	New_Vector.at<double>(2,0) = N+h;
	
	
	return New_Vector;
}

cv::Mat GeoReference::EulerAngles_Plane(cv::Mat Orig_Vector, double Roll, double Pitch, double Yaw)
{
	//PRINT("Euler Angle called with:");
	//PRINT(matToString(Orig_Vector));
	//PRINT("Roll: " +Roll);
	//PRINT("Pitch: " +Pitch);
	//PRINT("Yaw: " +Yaw);

	double R = Roll;
	double P = Pitch;
	double Y = Yaw;

	double rollarr[9] = {1,0,0,
						0, cos(R), sin(R),
						0, -sin(R), cos(R)};
	cv::Mat Roll_Transform(3, 3, CV_64FC1, rollarr );

	double pitcharr[9] = {cos(P), 0, -sin(P),
						0, 1, 0,
						sin(P), 0, cos(P)};
	cv::Mat Pitch_Transform(3, 3, CV_64FC1, pitcharr );

	double yawarr[9] = {cos(Y), sin(Y), 0,
						-sin(Y), cos(Y), 0,
						0, 0, 1};
	cv::Mat Yaw_Transform(3, 3, CV_64FC1, yawarr );


	cv::Mat Transfer_t(3,3,CV_64FC1);
	cv::Mat Transfer = Roll_Transform * Pitch_Transform * Yaw_Transform;
	cv::transpose(Transfer, Transfer_t);

	//PRINT(matToString((Transfer*Orig_Vector)));
	return Transfer_t*Orig_Vector;
}

// GPS in degrees
// plane and gimbal orientation in radians
// target_x/y in pixels, where 0,0 is center
bool GeoReference::forwardGeoreferencing(double plane_latitude,
                                         double plane_longitude,
                                         double plane_altitude,
                                         double plane_roll,
                                         double plane_pitch,
                                         double plane_heading, 
                                         double gimbal_roll, 
                                         double gimbal_pitch, 
                                         double gimbal_yaw, 
				                         double target_x,
                                         double target_y,
                                         double x_pixels,
                                         double y_pixels,
                                         double zoom,
                                         double & Target_Latitude,
                                         double & Target_Longitude, 
                                         double & Target_Height)
{
	/////////////////////
	// Funcs used by forwardGeoreferencing:
	//	EulerAngles_Plane(), Mat Quaternion(double, double, double, double), Mat Quaternion_Transform(Mat, Mat), 
	//	Mat NED_to_ECEF(Mat, double, double), Mat ECEF_to_GEO(Mat, double, double, double)
	/////////////////////
	
	// bL = 0,0. tR = w,h.
	

	double x_fov = 46.0 * PI_TO_RAD;
	double y_fov = 34.0 * PI_TO_RAD;
	double a = 6378137;
	double b = 6356752.3142;
	double ground_altitude = 0.0;


	double pixel_x = target_x;// + x_pixels/2.0;
	double pixel_y = target_y;// + y_pixels/2.0;
	double zoom_factor = zoom;

	
	plane_latitude = plane_latitude * PI_TO_RAD;
	plane_longitude = plane_longitude * PI_TO_RAD;

	


	//double cam_pt_vec[3] = {0,0,1};
	//cv::Mat Camera_Point_Vector = cv::Mat(3, 1, CV_64FC1, cam_pt_vec);
	//double cam_up_vec[3] = {1,0,0};
	//cv::Mat Camera_Up_Vector = cv::Mat(3, 1, CV_64FC1, cam_up_vec);

	//double ground_altitude = 0;


	/////////////// Part A /////////////////////
	double N_vec_arr[3] = {1,0,0};
	double E_vec_arr[3] = {0,1,0};
	double D_vec_arr[3] = {0,0,1};
	cv::Mat Plane_N_vector = cv::Mat(3, 1, CV_64FC1, N_vec_arr);
	cv::Mat Plane_E_vector = cv::Mat(3, 1, CV_64FC1, E_vec_arr);
	cv::Mat Plane_D_vector = cv::Mat(3, 1, CV_64FC1, D_vec_arr);

	cv::Mat Camera_Point_Vector = EulerAngles_Plane(Plane_D_vector, plane_roll, plane_pitch, plane_heading);
	cv::Mat Camera_Up_Vector = EulerAngles_Plane(Plane_N_vector, plane_roll, plane_pitch, plane_heading);



	/////////////// Part B /////////////////////
	cv::Mat Q_gimbal_roll = Quaternion(gimbal_roll, Camera_Up_Vector.at<double>(0,0),  Camera_Up_Vector.at<double>(1,0),  Camera_Up_Vector.at<double>(2,0));
	Camera_Point_Vector = Quaternion_Transform(Camera_Point_Vector, Q_gimbal_roll);
	Camera_Up_Vector = Quaternion_Transform(Camera_Up_Vector, Q_gimbal_roll);

	cv::Mat axis = Camera_Point_Vector.cross(Camera_Up_Vector); 
	cv::Mat Q_gimbal_pitch = Quaternion(gimbal_pitch, axis.at<double>(0,0), axis.at<double>(1,0), axis.at<double>(2,0));
	Camera_Point_Vector = Quaternion_Transform(Camera_Point_Vector, Q_gimbal_pitch);
	Camera_Up_Vector = Quaternion_Transform(Camera_Up_Vector, Q_gimbal_pitch);

	/////////////// Part C /////////////////////
	double fovarr[3] = {x_fov, y_fov, 1};
	cv::Mat FOV(3, 1, CV_64FC1, fovarr );
	double scalearr[9] = {1/zoom_factor, 0, 0, 0, 1/zoom_factor, 0, 0, 0, 1};
	cv::Mat Scale(3, 3, CV_64FC1, scalearr );

	cv::Mat FOV_zoom_accounted = Scale*FOV;

	/////////////// Part D /////////////////////

	cv::Mat c_p = Camera_Point_Vector;
	cv::Mat c_u = Camera_Up_Vector;
	cv::Mat c_s = c_p.cross(c_u);


	double max_w = tan(FOV_zoom_accounted.at<double>(0,0)/2)*2;
	double max_h = tan(FOV_zoom_accounted.at<double>(1,0)/2)*2;

	double w = max_w * (0.5 - pixel_x/(x_pixels - 1));
	double h = max_h * (pixel_y/(y_pixels - 1) - 0.5);

	cv::Mat c_f = c_p - (w * c_s) + (h * c_u);
	cv::Mat Pixel_Point_Vector = c_f;


	/////////////// Part E  /////////////////////
	double f = a/(a-b);
	double e=sqrt((1/f)*(2-(1/f)));
	double N=a/(sqrt(1-e*e*sin(plane_latitude)*sin(plane_latitude)));

	double Plane_XYZ_arr[3] = {0,0,0};
	Plane_XYZ_arr[0] = (N+plane_altitude)*cos(plane_latitude)*cos(plane_longitude);
	Plane_XYZ_arr[1] = (N+plane_altitude)*cos(plane_latitude)*sin(plane_longitude);
	Plane_XYZ_arr[2] = (N*(1-e*e)+plane_altitude)*sin(plane_latitude);
	cv::Mat Plane_XYZ(3, 1, CV_64FC1, Plane_XYZ_arr );

	/////////////// Part F  /////////////////////
	
	double temparr[3] = {0, 0, plane_altitude + ground_altitude};
	cv::Mat temp(3, 1, CV_64FC1, temparr );

	double Ground_XYZ_arr[3] = {0,0,0}; // TODO: fix "plane_latitude"
	Ground_XYZ_arr[0] = (N)*cos(plane_latitude)*cos(plane_longitude);
	Ground_XYZ_arr[1] = (N)*cos(plane_latitude)*sin(plane_longitude);
	Ground_XYZ_arr[2] = (N*(1-e*e))*sin(plane_latitude);
	cv::Mat Ground_XYZ(3, 1, CV_64FC1, Ground_XYZ_arr );

	cv::Mat Pixel_XYZ = NED_to_ECEF(Pixel_Point_Vector, plane_latitude, plane_longitude);
	
	double dist = Plane_XYZ.dot(Ground_XYZ - Plane_XYZ) / Pixel_XYZ.dot(Plane_XYZ);

	cv::Mat Target_XYZ = Plane_XYZ + Pixel_XYZ*dist;
	
	cv::Mat Target_GEO = ECEF_to_GEO(Target_XYZ, f, e, a);
	
	Target_Latitude = Target_GEO.at<double>(0,0) * 180.0 / M_PI;
	Target_Longitude = Target_GEO.at<double>(1,0) * 180.0 / M_PI - 180.0;
	Target_Height = Target_GEO.at<double>(2,0) - N;
	
	/////////////// Error Checking /////////////////////
	bool success = true;
	if (abs(Target_Latitude) > 180.0 || approxEqual(Target_Latitude, 0.0) || approxEqual(abs(Target_Latitude), 180.0))
	{
		Target_Latitude = 1000.0;
		success = false;
	}

	if (abs(Target_Longitude) > 180.0 || approxEqual(Target_Longitude, 0.0) || approxEqual(abs(Target_Longitude), 180.0))
	{
		Target_Longitude = 1000.0;
		success = false;
	}

	if (Pixel_Point_Vector.at<double>(2,0) < 0.0)
	{
		Target_Latitude = 1000.0;
		Target_Longitude = 1000.0;
		success = false;
	}

	return success;
}

cv::Mat GeoReference::EulerAngles(bool transpose, cv::Mat Orig_Vector, double Roll, double Pitch, double Yaw)
{
	double transarr[9] = {1,1,1,
							1,1,1,  
							1,1,1};
	cv::Mat Transfer = cv::Mat(3, 3, CV_64FC1, transarr).inv();

	return Transfer;


}

void GeoReference::pixelGeoreference(double plane_latitude_deg,
                                     double plane_longitude_deg,
                                     double plane_altitude,
                                     double plane_roll,
                                     double plane_pitch,
                                     double plane_heading,
                                     double gimbal_roll,
                                     double gimbal_pitch,
                                     double target_latitude_deg,
                                     double target_longitude_deg,
                                     double horizontal_fov,
                                     double vertical_fov,
                                     double x_pixels,
                                     double y_pixels,
                                     double& pixel_x,
                                     double& pixel_y){
  double plane_latitude = plane_latitude_deg * M_PI / 180.0;
  double plane_longitude = plane_longitude_deg * M_PI / 180.0;
  double target_latitude = target_latitude_deg * M_PI / 180.0;
  double target_longitude = target_longitude_deg * M_PI / 180.0;
  double x_fov = horizontal_fov * M_PI / 180.0;
  double y_fov = vertical_fov * M_PI / 180.0;
  double a = 6378137.0;
  double b = 6356752.3142;

/* -------------------------- Part A ----------------------------- */

  double N_vec_arr[3] = {1,0,0};
  double E_vec_arr[3] = {0,1,0};
  double D_vec_arr[3] = {0,0,1};
  cv::Mat plane_N_vector = cv::Mat(3, 1, CV_64FC1, N_vec_arr);
  cv::Mat plane_E_vector = cv::Mat(3, 1, CV_64FC1, E_vec_arr);
  cv::Mat plane_D_vector = cv::Mat(3, 1, CV_64FC1, D_vec_arr);

  cv::Mat Q_plane_roll = Quaternion(plane_roll, 1, 0, 0);
  plane_E_vector = Quaternion_Transform(plane_E_vector, Q_plane_roll);
  plane_D_vector = Quaternion_Transform(plane_D_vector, Q_plane_roll);

  cv::Mat Q_plane_pitch = Quaternion(plane_pitch, 0, 1, 0);
  plane_D_vector = Quaternion_Transform(plane_D_vector, Q_plane_pitch);
  plane_N_vector = Quaternion_Transform(plane_N_vector, Q_plane_pitch);
  plane_E_vector = Quaternion_Transform(plane_E_vector, Q_plane_pitch);

  cv::Mat Q_plane_yaw = Quaternion(plane_heading, 0, 0, 1);
  plane_D_vector = Quaternion_Transform(plane_D_vector, Q_plane_yaw);
  plane_N_vector = Quaternion_Transform(plane_N_vector, Q_plane_yaw);
  plane_E_vector = Quaternion_Transform(plane_E_vector, Q_plane_yaw);

  cv::Mat Camera_Point_Vector = plane_D_vector;
  cv::Mat Camera_Up_Vector = plane_N_vector;

/* -------------------------- Part B ----------------------------- */

  // set up quaternion and rotate about camera_up_vector, which points forward down the plane
  cv::Mat Q_gimbal_roll = Quaternion(gimbal_roll,
                                     Camera_Up_Vector.ptr<double>(0)[0],
                                     Camera_Up_Vector.ptr<double>(0)[1],
                                     Camera_Up_Vector.ptr<double>(0)[2]); 

  // rotate through gimbal roll
  Camera_Point_Vector = Quaternion_Transform(Camera_Point_Vector, Q_gimbal_roll);

  // rotate through gimbal roll
  Camera_Up_Vector = Quaternion_Transform(Camera_Up_Vector, Q_gimbal_roll);  

  cv::Mat axis = Camera_Point_Vector.cross(Camera_Up_Vector); // set up axis to rotate about

  // build the gimbal pitch quaternion, rotation about camera up
  double* _axis = axis.ptr<double>(0);
  cv::Mat Q_gimbal_pitch = Quaternion(gimbal_pitch, _axis[0], _axis[1], _axis[2]);

  // rotate through gimbal pitch  %%HERE IS ERRORRRRRRRRRR!!!!
  Camera_Point_Vector = Quaternion_Transform(Camera_Point_Vector, Q_gimbal_pitch); 

  // rotate through gimbal pitch
  Camera_Up_Vector = Quaternion_Transform(Camera_Up_Vector, Q_gimbal_pitch);  

/* -------------------------- Part D ----------------------------- */

  double f=a/(a-b); // Earth flatness
  double e=sqrt((1/f)*(2-(1/f))); // Earth eccenricity

  /* Length of Normal to the ellipsoid, extending from the surface to the intersection with 
     the z-axis in ECEF */
  double N= a / ( sqrt ( 1 -( e * e ) * sin(plane_latitude) * sin(plane_latitude))); 

  double Plane_XYZ_arr[3] = {0,0,0};
  Plane_XYZ_arr[0] = (N+plane_altitude)*cos(plane_latitude)*cos(plane_longitude);
  Plane_XYZ_arr[1] = (N+plane_altitude)*cos(plane_latitude)*sin(plane_longitude);
  Plane_XYZ_arr[2] = (N*(1-e*e)+plane_altitude)*sin(plane_latitude);
  cv::Mat Plane_XYZ(3, 1, CV_64FC1, Plane_XYZ_arr );

/* -------------------------- Part E ----------------------------- */

  double target_altitude = 0;
  double Ground_XYZ_arr[3] = {0,0,0};
  Ground_XYZ_arr[0] = (N+target_altitude)*cos(target_latitude)*cos(target_longitude);
  Ground_XYZ_arr[1] = (N+target_altitude)*cos(target_latitude)*sin(target_longitude);
  Ground_XYZ_arr[2] = (N*(1-e*e)+target_altitude)*sin(target_latitude);
  cv::Mat Ground_XYZ(3, 1, CV_64FC1, Ground_XYZ_arr );

  // points from plane's location to the target location in ECEF
  cv::Mat Ground_Point_Vector = Ground_XYZ - Plane_XYZ; 
  Ground_Point_Vector = Ground_Point_Vector/norm(Ground_Point_Vector); // unit vector

/* -------------------------- Part F ----------------------------- */

  cv::Mat CPV_XYZ = NED_to_ECEF(Camera_Point_Vector, plane_latitude, plane_longitude);
  cv::Mat CUV_XYZ = NED_to_ECEF(Camera_Up_Vector, plane_latitude, plane_longitude);

  cv::Mat Camera_Plane_Origin = Plane_XYZ + CPV_XYZ;

  // set up the input for plane-line intersection
  cv::Mat P1 = Camera_Plane_Origin;
  cv::Mat P2 = Camera_Plane_Origin + CUV_XYZ;
  cv::Mat P3 = Camera_Plane_Origin + CPV_XYZ.cross(CUV_XYZ);
  cv::Mat P4 = Plane_XYZ;
  cv::Mat P5 = Plane_XYZ + Ground_Point_Vector;

  // obtain the intersection point
  cv::Mat E = GeoReference::intersectLinePlane(P1,P2,P3,P4,P5); 

  cv::Mat PE = E - Plane_XYZ;
  cv::Mat U = CUV_XYZ;
  cv::Mat Z = Camera_Point_Vector.cross(Camera_Up_Vector);

  // project PE onto U to get the y-component vector
  double y_component = scalarProjection(PE, U);
  double x_component = scalarProjection(PE, Z);

  // obtain the pixel locations
  double tot_y = 2*tan(y_fov/2); // x distance in the viewing box
  double tot_x = 2*tan(x_fov/2); // y distance in the viewing box
  pixel_y = (1+2*y_component/tot_y)*y_pixels/2;
  pixel_x = (1+2*x_component/tot_x)*x_pixels/2;
}

void GeoReference::reverseGeoreference(double plane_latitude,
                                       double plane_longitude,
                                       double plane_altitude,
                                       double plane_roll,
                                       double plane_pitch,
                                       double plane_heading,
				                       double Target_Latitude,
                                       double Target_Longitude,
                                       double Target_Height,
                                       double & gimbal_roll,
                                       double & gimbal_pitch)
{
	typedef cv::Vec<double, 1> VT;

	// Step 1: Moving Plane and Target LAt/Lon into ECEF
	double Plane_yaw = plane_heading;
	double G_lat = 32.0;
	double G_long = -117.0;

	double a = 6378137.0;
	double b = 6356752.3142;
	double f = a/(a-b);
	double e = sqrt((1/f)*(2.0-(1.0/f)));
	double N_P = a/(sqrt(1.0-(e*e)*sind(plane_latitude)*sind(plane_latitude)));
	double N_T = a/(sqrt(1.0-(e*e)*sind(Target_Latitude)*sind(Target_Latitude)));

	double X_P = (N_P+plane_altitude)*cosd(plane_latitude)*cosd(plane_longitude);
	double Y_P = (N_P+plane_altitude)*cosd(plane_latitude)*sind(plane_longitude);
	double Z_P = (N_P*(1-e*e)+plane_altitude)*sind(plane_latitude);
	
	double X_T = (N_T+Target_Height)*cosd(Target_Latitude)*cosd(Target_Longitude);
	double Y_T = (N_T+Target_Height)*cosd(Target_Latitude)*sind(Target_Longitude);
	double Z_T = (N_T*(1-e*e)+Target_Height)*sind(Target_Latitude);

	// Step 2: Form Positions, find vector that connects the two points 
	double xyzparr[3] = {X_P, Y_P, Z_P};
	cv::Mat XYZP(3, 1, CV_64FC1, xyzparr );
	
	double xyztarr[3] = {X_T, Y_T, Z_T};
	cv::Mat XYZT(3, 1, CV_64FC1, xyzparr );

	double transGarr[9] = {	-sind(G_lat)*cosd(G_long),		-sind(G_lat)*sind(G_long),	cosd(G_lat), 
								-sind(G_long),					cosd(G_long),				0.0, 
								-cosd(G_lat)*cosd(G_long),		-cosd(G_lat)*sind(G_long),	-sind(G_lat)	};
	cv::Mat transG(3, 3, CV_64FC1, transGarr );

	cv::Mat NEDP = transG*XYZP;
	cv::Mat NEDT = transG*XYZT;

	cv::Mat NED_TRUE = NEDT-NEDP;


	// Step 3:  find NED plane

	cv::Mat NED_PLANE = EulerAngles(true, NED_TRUE, plane_roll, plane_pitch, Plane_yaw);
	
	double up = NED_PLANE.at<VT>(0, 0)[0];
	double vp = NED_PLANE.at<VT>(1, 0)[0];
	double wp = NED_PLANE.at<VT>(2, 0)[0];

	// Step 4: Find Gimbal Angles

	gimbal_roll = -atand(vp/wp);
	gimbal_pitch = -atand((-up)/(sind(gimbal_roll)*vp+cosd(gimbal_roll)*wp));

}

/*Reference ellipsoids derived from Peter H. Dana's website- 
http://www.utexas.edu/depts/grg/gcraft/notes/datum/elist.html
Department of Geography, University of Texas at Austin
Internet: pdana@mail.utexas.edu
3/22/95

Source
Defense Mapping Agency. 1987b. DMA Technical Report: Supplement to Department of Defense World Geodetic System
1984 Technical Report. Part I and II. Washington, DC: Defense Mapping Agency
*/

cv::Mat EulerAngles(bool transpose, cv::Mat Orig_Vector, float Roll, float Pitch, float Yaw)
{

	double transarr[9] = {1,1,1,
						  1,1,1,  
						  1,1,1};
	cv::Mat Transfer = cv::Mat(3, 3, CV_64FC1, transarr).inv();

	return Transfer;

	
}


