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

	getStartPoint();
	get2ndPoint();
	//plotNormalVector();
	constructInverseP();
	//getEndPoint();
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
}

void destination3dPoint::get2ndPoint(){
	//int lengthOfLine = 50;
	cv::Vec3f plane = hoop.plane;

	double A = plane[0];
	double B = plane[1];
	double C = plane[2];
	cv::Point hoopCentroid = hoop.ellipse.center;

	//double new_x = hoopCentroid.x - normalVector[0] * lengthOfLine;
	//double new_y = hoopCentroid.y - normalVector[1] * lengthOfLine;
	//double new_z = new_x*A + new_y*B + C;

	//secondPoint = cv::Point3f(new_x, new_y, new_z);

	double x = hoopCentroid.x;
	double y = hoopCentroid.y;
	double z = hoop.selectedDepthPoints[0][2];
	//printf("\nz: %f\n", z);
	//double z = x*A + y*B + C;
}


/**
 * A function that will plot the normal vector on the image that contains the fitted plane.
 */
void destination3dPoint::plotNormalVector(){

	cv::Point hoopCentroid = hoop.ellipse.center;
	cv::Scalar color = cv::Scalar(0, 0, 255);

	cv::line(img, hoopCentroid, cv::Point((int) secondPoint.x, (int) secondPoint.y), color, 3, 1, 0);
}

void destination3dPoint::constructInverseP(){

	float roll,pitch,yaw;
	float x,y,z;

	client->getRollPitchYaw(msg,roll,pitch,yaw);
	client->getGroundTruth(msg,x,y,z);

	//x = 0; y = 0; z = -1;
	//roll = pitch = yaw = 0;

	float ca = cos(yaw);
	float sa = sin(yaw);
	float cb = cos(pitch);
	float sb = sin(pitch);
	float cg = cos(roll);
	float sg = sin(roll);

	float rotMat[4][4] =
	{{ca*cb,    ca*sb*sg-sa*cg,      ca*sb*cg+sa*sg,    	   x*1000},
			{sa*cb,    sa*sb*sg+ca*cg,      sa*sb*cg-ca*sg,    y*1000},
			{-sb,          cb*sg,               cb*cg,         z*1000},
			{0,              0,                   0,           1}};

	float rotMat2[4][4] =
	{{0,1,0,0},
			{0,0,1,0},
			{1,0,0,0},
			{0,0,0,1}
	};

	cv::Mat rot(4,4,CV_32FC1,rotMat);
	cv::Mat rot2(4,4,CV_32FC1,rotMat2);

	inverseP = rot*rot2.inv();
}

void destination3dPoint::getStartPoint(){

	float ground_x, ground_y, ground_z;
	client->getGroundTruth(msg, ground_x, ground_y, ground_z);

	startPoint = cv::Point3f(ground_x, ground_y, ground_z);
}

void destination3dPoint::getEndPoint(){

	cv::Mat intrinsic = inverseK.inv();
	float focus = intrinsic.at<float>(0, 0);

	//secondPoint.x = 752/2;
	//secondPoint.y = 480/2;
	//secondPoint.z = 1.0f;

	cv::Vec4f cameraPoint;
	cameraPoint[0] = 1000.0f * (secondPoint.x - hoop.depthImage.cols / 2.0f) * secondPoint.z / focus;
	cameraPoint[1] = 1000.0f * (secondPoint.y - hoop.depthImage.rows / 2.0f) * secondPoint.z / focus;
	cameraPoint[2] = 1000.0f * secondPoint.z;
	cameraPoint[3] = 1;

//  std::cout << "\n cameraPoint: " << cv::Mat(cameraPoint) << std::endl;

	cv::Mat X = inverseP * cv::Mat(cameraPoint);

		endPoint = cv::Point3f(X.at<float>(0,0)/X.at<float>(3,0),
				X.at<float>(1,0)/X.at<float>(3,0),
			X.at<float>(2,0)/X.at<float>(3,0));
			
		endPoint*=0.001;
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
	
	ooo[0] = (secondPoint.x - img.cols / 2.) * secondPoint.z / focus;
	ooo[1] = (secondPoint.y - img.rows / 2.) * secondPoint.z / focus;
	ooo[2] = secondPoint.z;

  std::cout<<"secondPoint: "<<cv::Mat(secondPoint)<<std::endl;
  std::cout<<"ooo: "<<cv::Mat(ooo)<<std::endl;


	cv::Vec4f cameraPoint = cv::Vec4f(ooo[0] * 1000.0f,
	    			                        ooo[1] * 1000.0f,
			                              ooo[2] * 1000.0f,
  			                            1);

	cv::Mat X = H * cv::Mat(cameraPoint);

	cv::Vec3f fP = cv::Vec<float, 3>(X.at<float>(0, 0) / X.at<float>(3, 0),
	                         X.at<float>(1, 0) / X.at<float>(3, 0),
        			             X.at<float>(2, 0) / X.at<float>(3, 0));

	endPoint = cv::Point3f(fP)*0.001;
	std::cout<<"endPoint: "<<cv::Mat(endPoint)<<"\nfP: "<<cv::Mat(fP)<<"\n\n";
}
