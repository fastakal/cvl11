/*
 * HoopPosition.cpp
 *
 *  Created on: Nov 26, 2011
 *      Author: root
 */

#include "HoopPosition.h"
HoopPosition::HoopPosition(){}

HoopPosition::HoopPosition(cv::Mat depthImg, cv::Mat dispImage, cv::RotatedRect el, int numberOfPointsForDepth) {
	colormap_jet =
	{
			{0.0f,0.0f,0.53125f},
			{0.0f,0.0f,0.5625f},
			{0.0f,0.0f,0.59375f},
			{0.0f,0.0f,0.625f},
			{0.0f,0.0f,0.65625f},
			{0.0f,0.0f,0.6875f},
			{0.0f,0.0f,0.71875f},
			{0.0f,0.0f,0.75f},
			{0.0f,0.0f,0.78125f},
			{0.0f,0.0f,0.8125f},
			{0.0f,0.0f,0.84375f},
			{0.0f,0.0f,0.875f},
			{0.0f,0.0f,0.90625f},
			{0.0f,0.0f,0.9375f},
			{0.0f,0.0f,0.96875f},
			{0.0f,0.0f,1.0f},
			{0.0f,0.03125f,1.0f},
			{0.0f,0.0625f,1.0f},
			{0.0f,0.09375f,1.0f},
			{0.0f,0.125f,1.0f},
			{0.0f,0.15625f,1.0f},
			{0.0f,0.1875f,1.0f},
			{0.0f,0.21875f,1.0f},
			{0.0f,0.25f,1.0f},
			{0.0f,0.28125f,1.0f},
			{0.0f,0.3125f,1.0f},
			{0.0f,0.34375f,1.0f},
			{0.0f,0.375f,1.0f},
			{0.0f,0.40625f,1.0f},
			{0.0f,0.4375f,1.0f},
			{0.0f,0.46875f,1.0f},
			{0.0f,0.5f,1.0f},
			{0.0f,0.53125f,1.0f},
			{0.0f,0.5625f,1.0f},
			{0.0f,0.59375f,1.0f},
			{0.0f,0.625f,1.0f},
			{0.0f,0.65625f,1.0f},
			{0.0f,0.6875f,1.0f},
			{0.0f,0.71875f,1.0f},
			{0.0f,0.75f,1.0f},
			{0.0f,0.78125f,1.0f},
			{0.0f,0.8125f,1.0f},
			{0.0f,0.84375f,1.0f},
			{0.0f,0.875f,1.0f},
			{0.0f,0.90625f,1.0f},
			{0.0f,0.9375f,1.0f},
			{0.0f,0.96875f,1.0f},
			{0.0f,1.0f,1.0f},
			{0.03125f,1.0f,0.96875f},
			{0.0625f,1.0f,0.9375f},
			{0.09375f,1.0f,0.90625f},
			{0.125f,1.0f,0.875f},
			{0.15625f,1.0f,0.84375f},
			{0.1875f,1.0f,0.8125f},
			{0.21875f,1.0f,0.78125f},
			{0.25f,1.0f,0.75f},
			{0.28125f,1.0f,0.71875f},
			{0.3125f,1.0f,0.6875f},
			{0.34375f,1.0f,0.65625f},
			{0.375f,1.0f,0.625f},
			{0.40625f,1.0f,0.59375f},
			{0.4375f,1.0f,0.5625f},
			{0.46875f,1.0f,0.53125f},
			{0.5f,1.0f,0.5f},
			{0.53125f,1.0f,0.46875f},
			{0.5625f,1.0f,0.4375f},
			{0.59375f,1.0f,0.40625f},
			{0.625f,1.0f,0.375f},
			{0.65625f,1.0f,0.34375f},
			{0.6875f,1.0f,0.3125f},
			{0.71875f,1.0f,0.28125f},
			{0.75f,1.0f,0.25f},
			{0.78125f,1.0f,0.21875f},
			{0.8125f,1.0f,0.1875f},
			{0.84375f,1.0f,0.15625f},
			{0.875f,1.0f,0.125f},
			{0.90625f,1.0f,0.09375f},
			{0.9375f,1.0f,0.0625f},
			{0.96875f,1.0f,0.03125f},
			{1.0f,1.0f,0.0f},
			{1.0f,0.96875f,0.0f},
			{1.0f,0.9375f,0.0f},
			{1.0f,0.90625f,0.0f},
			{1.0f,0.875f,0.0f},
			{1.0f,0.84375f,0.0f},
			{1.0f,0.8125f,0.0f},
			{1.0f,0.78125f,0.0f},
			{1.0f,0.75f,0.0f},
			{1.0f,0.71875f,0.0f},
			{1.0f,0.6875f,0.0f},
			{1.0f,0.65625f,0.0f},
			{1.0f,0.625f,0.0f},
			{1.0f,0.59375f,0.0f},
			{1.0f,0.5625f,0.0f},
			{1.0f,0.53125f,0.0f},
			{1.0f,0.5f,0.0f},
			{1.0f,0.46875f,0.0f},
			{1.0f,0.4375f,0.0f},
			{1.0f,0.40625f,0.0f},
			{1.0f,0.375f,0.0f},
			{1.0f,0.34375f,0.0f},
			{1.0f,0.3125f,0.0f},
			{1.0f,0.28125f,0.0f},
			{1.0f,0.25f,0.0f},
			{1.0f,0.21875f,0.0f},
			{1.0f,0.1875f,0.0f},
			{1.0f,0.15625f,0.0f},
			{1.0f,0.125f,0.0f},
			{1.0f,0.09375f,0.0f},
			{1.0f,0.0625f,0.0f},
			{1.0f,0.03125f,0.0f},
			{1.0f,0.0f,0.0f},
			{0.96875f,0.0f,0.0f},
			{0.9375f,0.0f,0.0f},
			{0.90625f,0.0f,0.0f},
			{0.875f,0.0f,0.0f},
			{0.84375f,0.0f,0.0f},
			{0.8125f,0.0f,0.0f},
			{0.78125f,0.0f,0.0f},
			{0.75f,0.0f,0.0f},
			{0.71875f,0.0f,0.0f},
			{0.6875f,0.0f,0.0f},
			{0.65625f,0.0f,0.0f},
			{0.625f,0.0f,0.0f},
			{0.59375f,0.0f,0.0f},
			{0.5625f,0.0f,0.0f},
			{0.53125f,0.0f,0.0f},
			{0.5f,0.0f,0.0f}
	};

	depthImage = depthImg;
	disparityImage = dispImage;
	ellipse = el;
	depthValuesOfHoop = findPointsValues(numberOfPointsForDepth);
	selectedDepthPoints = findPointsValues(3);
	plotDepthValues(disparityImage);
	plotDepthValues(depthImage);
	disparityImageWithPlane = disparityImage;

	// If less than 4 points of depth are valid, then instead of having an error fitting the plane, we put the disparity image the same as input.
	if( depthValuesOfHoop.size() < 4){
		disparityImageWithPlane = disparityImage;
		std::cout<<"\n\n\n WARNING: less than 5 valid points in the ellipse. Validitiy: Depth values\n\n\n";
		std::cout<<depthValuesOfHoop.size()<<"\n";
	}
	else {
		fitPlane();
		plotThe4Points();

		//addFittedPlaneToImage(disparityImage);
	}
}

HoopPosition::~HoopPosition() {
	// TODO Auto-generated destructor stub
}

/**
 * A function that will take the ellipse, find the depth values of points on its perimeter and save them to a vector of points.
 *
 */
cv::vector<cv::Vec3f> HoopPosition::findPointsValues(int numberOfPoints){
	float h = ellipse.center.x;
	float k = ellipse.center.y;
	float ellipseAngle = ellipse.angle * 3.14 / 180.0;

	float a = ellipse.size.width / 2;
	float b = ellipse.size.height / 2;
	float x, y, z;
	cv::Scalar color;


	cv::vector<cv::Vec3f> depthValues;
	depthValues.resize(numberOfPoints);
	cv::Vec3f currentDepthValue;
	int validDepthValuesCounter = 0;

	for(int i = 0; i < numberOfPoints; i++){
		float angle = 2*3.14*(double(i)/double(numberOfPoints));
		x = h + a*cos(angle)*cos(ellipseAngle) - b*sin(angle)*sin(ellipseAngle);
		y = k + a*cos(angle)*sin(ellipseAngle) + b*sin(angle)*cos(ellipseAngle);
		currentDepthValue = get3DPoint(x,y);
		z = currentDepthValue[2];

		if(z > 0){
			depthValues[i] = currentDepthValue;
			validDepthValuesCounter++;
		}
	}

	depthValues.resize(validDepthValuesCounter);
	return depthValues;
}

/**
 * A function that plots small circles in the position of the depth values, with a color that corresponds to the depth value.
 */
void HoopPosition::plotDepthValues(cv::Mat& dispImage){
	int size = depthValuesOfHoop.size();
	cv::Vec3f pointIn3D;
	int x, y, z;
	cv::Scalar color;

	for(int i = 0; i < size; i++){
		pointIn3D = depthValuesOfHoop[i];
		x = pointIn3D[0];
		y = pointIn3D[1];
		z = pointIn3D[2];

		// If it's the depth image, then create a grayscale "color", otherwise, check the corresponding colormap.
		if( dispImage.channels() == 1){
			color = cv::Scalar(z, z, z);
		}
		else{
			int idx = fmin(z, 10.0f) / 10.0f * 127.0f;
			idx = 127 - idx;
			color = cv::Scalar(colormap_jet[idx][2] * 255.0f, colormap_jet[idx][1] * 255.0f, colormap_jet[idx][0] * 255.0f);
		}

		cv::circle(dispImage, cv::Point(x,y), 3, color, 1, 1);
	}
}

/**
 * A function that takes a position (x and y) and returns the value in that neighbourhood.
 */
cv::Vec3f HoopPosition::get3DPoint(int x,int y){
	cv::Vec3f pointIn3D;
	float z;
	pointIn3D[0] = x;
	pointIn3D[1] = y;
	float* depth = depthImage.ptr<float>(y);
	z = depth[x];
	pointIn3D[2] = z;

	return pointIn3D;
}

void HoopPosition::fitPlane(){

	cv::Vec3f a3DPoint;
	int noOfPoints = depthValuesOfHoop.size();

	//1. prepare the matrices for the data.
	CvMat *res  = cvCreateMat(3, 1, CV_32FC1);
	CvMat *matX = cvCreateMat(noOfPoints, 3, CV_32FC1);
	CvMat *matZ = cvCreateMat(noOfPoints, 1, CV_32FC1);

	//2. input the data into the matrices.
	for(int idx = 0; idx < noOfPoints; idx++){
		a3DPoint = depthValuesOfHoop[idx];
		cvmSet(matX, idx, 0, a3DPoint[0]);
		cvmSet(matX, idx, 1, a3DPoint[1]);
		cvmSet(matX, idx, 2, 1);
		cvmSet(matZ, idx, 0, a3DPoint[2]);
	}

	//3. solve the equation
	cvSolve(matX, matZ, res, CV_SVD);
	float A = cvmGet(res, 0, 0);
	float B = cvmGet(res, 1, 0);
	float C = cvmGet(res, 2, 0);

	plane[0] = A;
	plane[1] = B;
	plane[2] = C;
}

void HoopPosition::plotThe4Points(){

	float h = ellipse.center.x;
	float k = ellipse.center.y;
	float ellipseAngle = ellipse.angle * 3.14 / 180.0;

	float a = ellipse.size.width/2.0;
	float b = ellipse.size.height/2.0;
	float x, y, z;
	double A = plane[0];
	double B = plane[1];
	double C = plane[2];
	cv::Scalar color;

	for( int i = 0; i < 4; i++){
		float angle = 2*3.14*(double(i)/4.0);
		x = h + a*cos(angle)*cos(ellipseAngle) - b*sin(angle)*sin(ellipseAngle);
		y = k + a*cos(angle)*sin(ellipseAngle) + b*sin(angle)*cos(ellipseAngle);
		z = x*A + y*B + C;

		// Convert to colormap.
		int idx = fmin(z, 10.0f) / 10.0f * 127.0f;
		idx = 127 - idx;

		color = cv::Scalar(colormap_jet[idx][2] * 255.0f, colormap_jet[idx][1] * 255.0f, colormap_jet[idx][0] * 255.0f);

		cv::circle(disparityImage, cv::Point(x,y), 4, color, 5, 1);
	}
}

void HoopPosition::addFittedPlaneToImage(cv::Mat& dispOrDepthImage){

	double planeOrientationAngle = ellipse.angle;
	cv::Mat planeImage = cv::Mat::zeros(dispOrDepthImage.size(), dispOrDepthImage.type());
	dispOrDepthImage.copyTo(planeImage);

	planeImage= fillPlaneImage(planeImage);
	planeImage = rotatePlaneImage(planeImage, planeOrientationAngle);
	cv::addWeighted(dispOrDepthImage, 1, planeImage, 1, 0, planeImage);

	disparityImageWithPlane = planeImage;
}

cv::Mat HoopPosition::rotatePlaneImage(cv::Mat& planeImage, double angle){
	cv::Mat dst;
	cv::Point2f imageCenter(ellipse.center.x, ellipse.center.y);
	cv::Mat rot_mat = cv::getRotationMatrix2D(imageCenter, -angle, 1.0);
	warpAffine(planeImage, dst, rot_mat, planeImage.size());

	return dst;
}

cv::Mat HoopPosition::fillPlaneImage(cv::Mat& planeImage){
	cv::Mat dst = cv::Mat::zeros(planeImage.size(), planeImage.type());
	int x = ellipse.size.width / 1.5;
	int y = ellipse.size.height / 1.5;
	int centerX = ellipse.center.x;
	int centerY = ellipse.center.y;

	int startPointX = centerX - x;
	int startPointY = centerY - y;
	int endPointX = centerX + x;
	int endPointY = centerY + y;

	double A = plane[0];
	double B = plane[1];
	double C = plane[2];

	for( int i = startPointY; i < endPointY; i++){
		for( int j = dst.channels() * startPointX; j < dst.channels() * endPointX; j++){
			// Check for boundaries before changing the color.
			if(i>0 && j>0 && i < dst.rows && j < dst.cols*dst.channels()){
				// Outside, do nothing.
			}
			float z = (A*((double) j)/dst.channels() + B*i + C)*4;
			if(dst.channels() == 1){
				dst.at<float>(i,j) = z;
			}
			else{
				int idx = fmin(z, 10.0f) / 10.0f * 127.0f;
				idx = 127 - idx;

				dst.at<unsigned char>(i,j) = colormap_jet[idx][2] * 255.0f;
				dst.at<unsigned char>(i,j+1) = colormap_jet[idx][1] * 255.0f;
				dst.at<unsigned char>(i,j+2) = colormap_jet[idx][0] * 255.0f;
			}
		}
	}

	return dst;
}
