/*
 * destination3dPoint.cpp
 *
 *  Created on: Dec 5, 2011
 *      Author: root
 */

#include "destination3dPoint.h"

destination3dPoint::destination3dPoint(HoopPosition hp, PxSHMImageClient* cl, const mavlink_message_t* message) {
	hoop = hp;
	img = hp.disparityImageWithPlane;
	client = cl;
	msg = message;
	getNormalVector();
	getStartPoint();
	get2ndPoint();
	plotNormalVector();
	constructRotationMatrix();
	getEndPoint();
	getTrajectoryVector();
	//printAll();
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
	//z = x*A + y*B + C;
	cv::Vec3f pt1; pt1[0] = x_center; pt1[1] = y_center; pt1[2] = pt1[0]*A + pt1[1]*B + C;
	cv::Vec3f pt2; pt2[0] = x_center + (rand() % 20 + 5); pt2[1] = y_center + (rand() % 20 + 5); pt2[2] = pt2[0]*A + pt2[1]*B + C;
	cv::Vec3f pt3; pt3[0] = x_center + (rand() % 20 + 5); pt3[1] = y_center + (rand() % 20 + 5); pt3[2] = pt3[0]*A + pt3[1]*B + C;

	// Vectors of the 3 points and Vector product of two vectors.
	cv::Vec3f a; a[0] = pt3[0] - pt1[0]; a[1] = pt3[1] - pt1[1]; a[2] = pt3[2] - pt1[2];
	cv::Vec3f b; b[0] = pt2[0] - pt1[0]; b[1] = pt2[1] - pt1[1]; b[2] = pt2[2] - pt1[2];


	normalVector[0] = a[1] * b[2] - a[2] * b[1];
	normalVector[1] = a[2] * b[0] - a[0] * b[2];
	normalVector[2] = a[0] * b[1] - a[1] * b[0];

	//std::cout<<"a0: "<<a[0]<<"a1: "<<a[1]<<"a2: "<<a[2]<<"\n";
	//std::cout<<"b0: "<<b[0]<<"b1: "<<b[1]<<"b2: "<<b[2]<<"\n";
	//std::cout<<"normal: 0:"<<normalVector[0]<<" 1: "<<normalVector[1]<<" 2: "<<normalVector[2]<<"\n";
}

void destination3dPoint::get2ndPoint(){
	int lengthOfLine = 50;
	cv::Vec3f plane = hoop.plane;

	double A = plane[0];
	double B = plane[1];
	double C = plane[2];
	cv::Point hoopCentroid = hoop.ellipse.center;

	double new_x = hoopCentroid.x - normalVector[0] * lengthOfLine;
	double new_y = hoopCentroid.y - normalVector[1] * lengthOfLine;
	double new_z = new_x*A + new_y*B + C;

	secondPoint = cv::Point3f(new_x, new_y, new_z);
}


/**
 * A function that will plot the normal vector on the image that contains the fitted plane.
 */
void destination3dPoint::plotNormalVector(){

	cv::Point hoopCentroid = hoop.ellipse.center;
	cv::Scalar color = cv::Scalar(0, 0, 255);

	cv::line(img, hoopCentroid, cv::Point((int) secondPoint.x, (int) secondPoint.y), color, 3, 1, 0);
}

void destination3dPoint::constructRotationMatrix(){

	// Inspired by : http://planning.cs.uiuc.edu/node102.html
	// yaw = alpha, pitch = beta, roll = gamma;
	float roll, pitch, yaw;
	CvMat *rotMat  = cvCreateMat(3, 3, CV_32FC1);
	client->getRollPitchYaw(msg, roll, pitch, yaw);

	printf("RollPitchYaw: Roll: %f, Pitch: %f, Yaw: %f\n\n", roll, pitch, yaw);

	cvmSet(rotMat, 0, 0, cos(yaw) * cos(pitch));
	cvmSet(rotMat, 0, 1, cos(yaw) * sin(pitch) * sin(roll) - sin(yaw) * cos(roll));
	cvmSet(rotMat, 0, 2, cos(yaw) * sin(pitch) * cos(roll) + sin(yaw) * sin(roll));

	cvmSet(rotMat, 1, 0, sin(yaw) * cos(pitch));
	cvmSet(rotMat, 1, 1, sin(yaw) * sin(pitch) * sin(roll) + cos(yaw) * cos(roll));
	cvmSet(rotMat, 1, 2, sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll));

	cvmSet(rotMat, 2, 0, -sin(pitch));
	cvmSet(rotMat, 2, 1, cos(pitch) * sin(roll));
	cvmSet(rotMat, 2, 2, cos(pitch) * cos(roll));

	rotationMatrix = rotMat;
}

void destination3dPoint::getStartPoint(){

	float ground_x, ground_y, ground_z;
	client->getGroundTruth(msg, ground_x, ground_y, ground_z);

	startPoint = cv::Point3f(ground_x, ground_y, ground_z);
}

void destination3dPoint::getEndPoint(){

	float x = secondPoint.x;
	float y = secondPoint.y;
	float z = secondPoint.z;

	endPoint.x = cvmGet(rotationMatrix, 0, 0) * x + cvmGet(rotationMatrix, 0, 1) * y + cvmGet(rotationMatrix, 0, 2) * z;
	endPoint.y = cvmGet(rotationMatrix, 1, 0) * x + cvmGet(rotationMatrix, 1, 1) * y + cvmGet(rotationMatrix, 1, 2) * z;;
	endPoint.z = cvmGet(rotationMatrix, 2, 0) * x + cvmGet(rotationMatrix, 2, 1) * y + cvmGet(rotationMatrix, 2, 2) * z;;
}

void destination3dPoint::getTrajectoryVector(){

	trajectoryVector[0] = endPoint.x - startPoint.x;
	trajectoryVector[1] = endPoint.y - startPoint.y;
	trajectoryVector[2] = endPoint.z - startPoint.z;
}

void destination3dPoint::printAll(){

	// Print Rotation Matrix:
	std::cout<<" [\n";
	for(int i = 0; i < 3; i++){
		std::cout<<"[ ";
		for(int j = 0; j < 3; j++){
			std::cout<<". "<<i<<" . "<<j<<" . : "<<cvmGet(rotationMatrix, i, j);
		}
		std::cout<<" ]\n";
	}
	std::cout<<" ]\n";

	// Print the second Point:
	printf("SecondPoint: x: %f. y: %f. z: %f. \n", secondPoint.x, secondPoint.y, secondPoint.z);

	// Print the end Point:
	printf("EndPoint: x: %f. y: %f. z: %f. \n", endPoint.x, endPoint.y, endPoint.z);

	// Print the sart Point:
	printf("StartPoint: x: %f. y: %f. z: %f. \n", startPoint.x, startPoint.y, startPoint.z);
}
