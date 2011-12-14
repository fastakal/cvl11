/*
 * CodeContainer.cpp
 *
 *  Created on: Nov 15, 2011
 *      Author: root
 */

#include "CodeContainer.h"
#include "MyEllipses.h"

CodeContainer::CodeContainer(cv::Mat img, cv::Mat imgDepthColor, cv::Mat imgDepth, cv::Mat inverseK,
		PxSHMImageClient* client,
		const mavlink_message_t* message,
		int g_minimumLengthOfAcceptedContours,
		int g_maximumLengthOfAcceptedContours,
		int g_CannyThreshold,
		int g_contourApproxOrder,
		int g_dilate,
		int g_line_size_for_plotting,
		int g_distance_between_lines,
		bool plotOption,
		double g_angle_threshold_ratio) {

	depthImage = imgDepth;
	numberOfPointsForDepthOfHoop = 100;
	inverseIntrinsicMat = inverseK;

	cv::Mat temp = cv::Mat::zeros(img.size(), CV_8UC3);
	cv::Mat input_image = cv::Mat::zeros(img.size(), CV_8UC3);;
	Mat filteredLinesImage = Mat::zeros(img.size(), CV_8UC3);
	cv::Mat eImagePlusEllipses = Mat::zeros(img.size(), CV_8UC3);
	vector<vector<Vec4f>> lines;
	vector<vector<Vec4f> > lLines;

	img.copyTo(input_image);

	// create a segmented image out of the input image.
	segmentedImage myImage = segmentedImage(input_image, g_CannyThreshold, 2*g_CannyThreshold);

	// 2.2 regroup them in contours.
	// 2.3 approximate contours by a polynomial (curves/lines).
	ContoursAndLines contours = ContoursAndLines(myImage.getEdgeImage(),
			g_minimumLengthOfAcceptedContours,
			g_maximumLengthOfAcceptedContours,
			g_contourApproxOrder,
			g_line_size_for_plotting
	);

	// 2.4 approximate contours by lines of fixed size.

	lines = contours.getLines();

	// 2.5 Link and connect Lines (algorithm explained in the article).
	LinkedLines linkedlines = LinkedLines(lines,
			g_distance_between_lines,
			g_angle_threshold_ratio,
			g_line_size_for_plotting,
			filteredLinesImage);

	lLines = linkedlines.getLinkedLines();
	filteredLinesImage = linkedlines.filteredLinesImage;

	// 2.4 regroup contours into potential ellipses and choose the "best" one.
	filteredLinesImage.copyTo(temp);
	MyEllipses ml = MyEllipses(lLines, temp, myImage.getEdgeImage());

	filteredLinesImage = ml.filteredEllipsesImage;
	eImagePlusEllipses = ml.edgeImagePlusEllipses;
	finalEdgeImage = eImagePlusEllipses;

	// Plot the hoop on the disparity image.
	cv::RotatedRect hoop = ml.chosenEllipse;

	// Going to 3D hoop + plane fitting..
	if( hoop.center.x != 0 && hoop.center.y != 0){
		HoopPosition hp = HoopPosition(depthImage, imgDepthColor, hoop, numberOfPointsForDepthOfHoop);
		imgDepthColor = hp.disparityImageWithPlane;

		destination3dPoint pointIn3D = destination3dPoint(hp, client, message, inverseIntrinsicMat);
		cv::imshow("imgDepthColor+hoop+plane", pointIn3D.img);
		endPoint = pointIn3D.endPoint;
	}
	else {
		endPoint.x = 0; endPoint.y = 0; endPoint.z = 0;
	}



	if (plotOption){
		cv::Mat cnt_img = cv::Mat::zeros(img.size(), CV_8UC3);
		cnt_img = contours.getContoursImage();
		cv::Mat linesImage = cv::Mat::zeros(img.size(), CV_8UC3);
		linesImage = contours.linesImage;

		plotPipeLineImages(myImage.getEdgeImage(), cnt_img, linesImage, filteredLinesImage);
	}

}

CodeContainer::~CodeContainer() {}

double CodeContainer::getTimeNow(){
	struct 	timeval tp;
	double sec, usec, time;

	gettimeofday(&tp, NULL);
	sec = static_cast<double>( tp.tv_sec);
	usec = static_cast<double>( tp.tv_usec) / 1E6;
	time = sec + usec;

	return time;
}

void CodeContainer::plotPipeLineImages(cv::Mat firstImage,cv::Mat secondImage,cv::Mat thirdImage,cv::Mat fourthImage){

	int newSizeX = secondImage.cols/2 + 40;
	int newSizeY = secondImage.rows/2;
	int marginBetweenWindows = 20;

	cv::namedWindow("Thresholded Image",0);
	cv::namedWindow( "contours",		0);
	cv::namedWindow("lines",			0);
	cv::namedWindow("filtered lines",	0);

	cv::imshow("Thresholded Image", firstImage);
	cv::imshow("contours", 			secondImage);
	cv::imshow("lines", 			thirdImage);
	cv::imshow("filtered lines", 	fourthImage);

	cvResizeWindow("Thresholded Image", newSizeX, newSizeY);
	cvResizeWindow("contours", 			newSizeX, newSizeY);
	cvResizeWindow("lines", 			newSizeX, newSizeY);
	cvResizeWindow("filtered lines", 	newSizeX, newSizeY);
	cvResizeWindow("eImagePlusEllipses", newSizeX, newSizeY);

	cvMoveWindow("Thresholded Image", 	marginBetweenWindows, newSizeY + marginBetweenWindows);
	cvMoveWindow("contours", 			newSizeX + 2*marginBetweenWindows, newSizeY + marginBetweenWindows);
	cvMoveWindow("lines", 				marginBetweenWindows, 2*(newSizeY + marginBetweenWindows));
	cvMoveWindow("filtered lines", 		newSizeX + 2*marginBetweenWindows, 2*(newSizeY + marginBetweenWindows));
}
