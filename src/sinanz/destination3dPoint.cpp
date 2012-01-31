/*
 * destination3dPoint.cpp
 *
 *  Created on: Dec 5, 2011
 *      Author: root
 */

#include "destination3dPoint.h"

destination3dPoint::destination3dPoint(HoopPosition hp, PxSHMImageClient* cl, const mavlink_message_t* message, cv::Mat inverseIntrinsicMat) {
	hoop = hp;
	img = hp.disparityImageWithPlane;
	client = cl;
	msg = message;
	inverseK = inverseIntrinsicMat;
	getNormalVector();
	get2ndPoint();
	endPoint = globalPoint(pixelCoordinatesWithDepth);

}

destination3dPoint::~destination3dPoint() {}

/**
 * A function that will calculate the normal to the fitted plane.
 */
void destination3dPoint:: getNormalVector(){

	// 3-Points Selection.
	int size = hoop.depthValuesOfHoop.size();
	cv::Vec3f pt1; pt1 = hoop.depthValuesOfHoop[0];
	cv::Vec3f pt2; pt2 = hoop.depthValuesOfHoop[size/3];
	cv::Vec3f pt3; pt3 = hoop.depthValuesOfHoop[2*size/3];

	cv::Point3f pt1_3d = globalPoint(cv::Point3f(pt1));
	cv::Point3f pt2_3d = globalPoint(cv::Point3f(pt2));
	cv::Point3f pt3_3d = globalPoint(cv::Point3f(pt3));

	// Vectors of the 3 points and Vector product of two vectors.
	cv::Vec3f a; a[0] = pt3_3d.x - pt1_3d.x; a[1] = pt3_3d.y - pt1_3d.y; a[2] = pt3_3d.z - pt1_3d.z;
	cv::Vec3f b; b[0] = pt2_3d.x - pt1_3d.x; b[1] = pt2_3d.y - pt1_3d.y; b[2] = pt2_3d.z - pt1_3d.z;

	normalVector[0] = a[1] * b[2] - a[2] * b[1];
	normalVector[1] = a[2] * b[0] - a[0] * b[2];
	normalVector[2] = a[0] * b[1] - a[1] * b[0];

	normalVector = normalVector *
			(-1 / sqrt(
			normalVector[0] * normalVector[0] +
			normalVector[1] * normalVector[1] +
			normalVector[2] * normalVector[2]));
}

void destination3dPoint::get2ndPoint(){

	cv::Point hoopCentroid = hoop.ellipse.center;

	double x = hoopCentroid.x;
	double y = hoopCentroid.y;
	double z;
	printf("number of z values: %d\n", hoop.depthValuesOfHoop.size());
	for(int i = 0; i < hoop.depthValuesOfHoop.size(); i++){
	  z = z + hoop.depthValuesOfHoop[i][2];
	}
	z = z / hoop.depthValuesOfHoop.size();
	
//	double z = hoop.depthValuesOfHoop[0][2];

	pixelCoordinatesWithDepth = cv::Point3f(x, y, z);
}

cv::Point3f destination3dPoint::globalPoint(cv::Point3f point){

	float roll, pitch, yaw;
	float x, y, z;

	client->getRollPitchYaw(msg, roll, pitch, yaw);
	client->getGroundTruth(msg, x, y, z);

	float ca = cos(yaw);
	float sa = sin(yaw);
	float cb = cos(pitch);
	float sb = sin(pitch);
	float cg = cos(roll);
	float sg = sin(roll);

  float H1r[4][4] = {
    {ca * cb,  ca * sb * sg - sa * cg,  ca * sb * cg + sa * sg,  0},
    {sa * cb,  sa * sb * sg + ca * cg,  sa * sb * cg - ca * sg,  0},
    {-sb, 	   cb * sg, 				        cb * cg, 				         0},
    {0, 	     0, 					            0, 					             1}
	};

  float H1t[4][4] = {
    {1, 0, 0, x * 1000},
    {0, 1, 0, y * 1000},
    {0, 0, 1, z * 1000},
    {0, 0, 0, 1	      }
  };

	float H2t[4][4] = {
    {0, 0, 1, 0},
    {1, 0, 0, 0},
    {0, 1, 0, 0},
    {0, 0, 0, 1}
	};

  cv::Mat H1R(4, 4, CV_32FC1, H1r);
  cv::Mat H1T(4, 4, CV_32FC1, H1t);
  cv::Mat H2 (4, 4, CV_32FC1, H2t);

  cv::Mat H = H1T * (H1R * H2);

	cv::Mat intrinsic = inverseK.inv();
	float focus = intrinsic.at<float>(0,0);
	cv::Vec3f ooo;

	ooo[0] = (point.x - img.cols / 2.) * point.z / focus;
	ooo[1] = (point.y - img.rows / 2.) * point.z / focus;
	ooo[2] = point.z;

	cv::Vec4f cameraPoint = cv::Vec4f(
			ooo[0] * 1000.0f,
			ooo[1] * 1000.0f,
			ooo[2] * 1000.0f,
			1);

	cv::Mat X = H * cv::Mat(cameraPoint);

	cv::Vec3f fP = cv::Vec<float, 3>(X.at<float>(0, 0) / X.at<float>(3, 0),
			X.at<float>(1, 0) / X.at<float>(3, 0),
			X.at<float>(2, 0) / X.at<float>(3, 0));

	return cv::Point3f(fP)*0.001;
}
