/*
 * destination3dPoint.cpp
 *
 *  Created on: Dec 5, 2011
 *      Author: root
 */

#include "destination3dPoint.h"

destination3dPoint::destination3dPoint(HoopPosition hp) {
	hoop = hp;
	img = hp.disparityImageWithPlane;
	getNormalVector();
	plotNormalVector();
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

	std::cout<<"a0: "<<a[0]<<"a1: "<<a[1]<<"a2: "<<a[2]<<"\n";
	std::cout<<"b0: "<<b[0]<<"b1: "<<b[1]<<"b2: "<<b[2]<<"\n";

	normalVector[0] = a[1] * b[2] - a[2] * b[1];
	normalVector[1] = a[2] * b[0] - a[0] * b[2];
	normalVector[2] = a[0] * b[1] - a[1] * b[0];

	std::cout<<"normal: 0:"<<normalVector[0]<<" 1: "<<normalVector[1]<<" 2: "<<normalVector[2]<<"\n";


	//http://paulbourke.net/geometry/planeeq/
	//A = y1 (z2 - z3) + y2 (z3 - z1) + y3 (z1 - z2)
	//B = z1 (x2 - x3) + z2 (x3 - x1) + z3 (x1 - x2)
	//C = x1 (y2 - y3) + x2 (y3 - y1) + x3 (y1 - y2)
	//- D = x1 (y2 z3 - y3 z2) + x2 (y3 z1 - y1 z3) + x3 (y1 z2 - y2 z1)

	//double x1 = pt1[0]; double x2 = pt2[0]; double x3 = pt3[0];
	//double y1 = pt1[1]; double y2 = pt2[1]; double y3 = pt3[1];
	//double z1 = pt1[2]; double z2 = pt2[2]; double z3 = pt3[2];
	//double D;
	//std::cout<<"Plane: A: "<<A<<". B: "<<B<<". C: "<<C<<"\n";

	//A = y1 * (z2 - z3) + y2 * (z3 - z1) + y3 * (z1 - z2);
	//B = z1 * (x2 - x3) + z2 * (x3 - x1) + z3 * (x1 - x2);
	//C = x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2);
	//D = -1 * ( x1 * (y2 * z3 - y3 * z2) + x2 * (y3 * z1 - y1 * z3) + x3 * (y1 * z2 - y2 * z1) );
	//std::cout<<"calc: A: "<<A<<". B: "<<B<<". C: "<<C<<". D: "<<D<<"\n";

	//normalVector[0] = A / sqrt((A*A + B*B + C*C));
	//normalVector[1] = B / sqrt((A*A + B*B + C*C));
	//normalVector[2] = C / sqrt((A*A + B*B + C*C));

	//normalVector[0] = A;
	//normalVector[1] = B;
	//normalVector[2] = C;
}

/**
 * A function that will plot the normal vector on the image that contains the fitted plane.
 */
void destination3dPoint::plotNormalVector(){

	cv::Point pt1 = hoop.ellipse.center;
	double new_x = pt1.x - hoop.plane[0] * 5000;
	double new_y = pt1.y - hoop.plane[1] * 5000;
	cv::Point pt2 = cv::Point((int) new_x, (int) new_y);

	cv::Scalar color = cv::Scalar(0, 0, 255);
	cv::line(img, pt1, pt2, color, 3, 1, 0);
}
