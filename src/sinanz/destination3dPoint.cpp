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
	globalPoint();
}

destination3dPoint::~destination3dPoint() {}

/**
 * A function that will calculate the normal to the fitted plane.
 */
void destination3dPoint:: getNormalVector(){
	cv::Vec3f plane = hoop.plane;

	double A = plane[0];
	double B = plane[1];
	double C = plane[2];

	double x_center = hoop.ellipse.center.x;
	double y_center = hoop.ellipse.center.y;

	// 3-Points Selection.
	cv::Vec3f pt1; pt1 = hoop.selectedDepthPoints[0];
	cv::Vec3f pt2; pt2 = hoop.selectedDepthPoints[1];
	cv::Vec3f pt3; pt3 = hoop.selectedDepthPoints[2];

	// Vectors of the 3 points and Vector product of two vectors.
	cv::Vec3f a; a[0] = pt3[0] - pt1[0]; a[1] = pt3[1] - pt1[1]; a[2] = pt3[2] - pt1[2];
	cv::Vec3f b; b[0] = pt2[0] - pt1[0]; b[1] = pt2[1] - pt1[1]; b[2] = pt2[2] - pt1[2];


	normalVector[0] = a[1] * b[2] - a[2] * b[1];
	normalVector[1] = a[2] * b[0] - a[0] * b[2];
	normalVector[2] = a[0] * b[1] - a[1] * b[0];
}

void destination3dPoint::get2ndPoint(){

	cv::Point hoopCentroid = hoop.ellipse.center;

	double x = hoopCentroid.x;
	double y = hoopCentroid.y;
	double z = hoop.depthValuesOfHoop[0][2];

	pixelCoordinatesWithDepth = cv::Point3f(x, y, z);
	/*
	 *
	 for(int i = 0; i < hoop.depthValuesOfHoop.size(); i++){
		std::cout<<hoop.depthValuesOfHoop[i][2]<<"|";
		if(hoop.depthValuesOfHoop[i][2] == 0){
			std::cout<<"ZERO"<<"\n";
		}
	}
	std::cout<<"\n\npixelCoordinates"<<cv::Mat(pixelCoordinatesWithDepth)<<"\n\n";
	*/
}

void destination3dPoint::globalPoint(){

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

	float H1t[4][4] = {
			{ca * cb,  ca * sb * sg - sa * cg,  ca * sb * cg + sa * sg,  x * 1000},
			{sa * cb,  sa * sb * sg + ca * cg,  sa * sb * cg - ca * sg,  y * 1000},
			{-sb, 	   cb * sg, 				        cb * cg, 				         z * 1000},
			{0, 	     0, 					            0, 					             1	     }
	};


	float H2t[4][4] = {
			{0, 1, 0, 0},
			{0, 0, 1, 0},
			{1, 0, 0, 0},
			{0, 0, 0, 1}
	};

	cv::Mat H1(4, 4, CV_32FC1, H1t);
	cv::Mat H2(4, 4, CV_32FC1, H2t);

	cv::Mat H = H1 * H2.inv();

	cv::Mat intrinsic = inverseK.inv();
	float focus = intrinsic.at<float>(0,0);
	cv::Vec3f ooo;

	ooo[0] = (pixelCoordinatesWithDepth.x - img.cols / 2.) * pixelCoordinatesWithDepth.z / focus;
	ooo[1] = (pixelCoordinatesWithDepth.y - img.rows / 2.) * pixelCoordinatesWithDepth.z / focus;
	ooo[2] = pixelCoordinatesWithDepth.z;

	cv::Vec4f cameraPoint = cv::Vec4f(
			ooo[0] * 1000.0f,
			ooo[1] * 1000.0f,
			ooo[2] * 1000.0f,
			1);

	cv::Mat X = H * cv::Mat(cameraPoint);

	cv::Vec3f fP = cv::Vec<float, 3>(X.at<float>(0, 0) / X.at<float>(3, 0),
			X.at<float>(1, 0) / X.at<float>(3, 0),
			X.at<float>(2, 0) / X.at<float>(3, 0));

	endPoint = cv::Point3f(fP)*0.001;
	//std::cout<<"pixelCoordinates + Depth: "<<cv::Mat(pixelCoordinatesWithDepth)<<std::endl;
	//std::cout<<"Camera Point: "<<cv::Mat(ooo)<<std::endl;
	//std::cout<<"World Point: "<<cv::Mat(endPoint)<<"\n";
}
