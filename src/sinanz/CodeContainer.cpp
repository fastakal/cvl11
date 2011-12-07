/*
 * CodeContainer.cpp
 *
 *  Created on: Nov 15, 2011
 *      Author: root
 */

#include "CodeContainer.h"
#include "MyEllipses.h"

CodeContainer::CodeContainer(cv::Mat img, cv::Mat imgDepthColor, cv::Mat imgDepth,
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

	cv::Mat temp = cv::Mat::zeros(img.size(), CV_8UC3);
	cv::Mat cnt_img = cv::Mat::zeros(img.size(), CV_8UC3);
	cv::Mat input_image = cv::Mat::zeros(img.size(), CV_8UC3);;
	vector<vector<Vec4f>> lines;
	cv::Mat linesImage = cv::Mat::zeros(img.size(), CV_8UC3);
	Mat filteredLinesImage = Mat::zeros(img.size(), CV_8UC3);
	cv::Mat eImagePlusEllipses = Mat::zeros(img.size(), CV_8UC3);

	// Create a font to be used...
	img.copyTo(input_image);

	// create a segmented image out of the input image.
	segmentedImage myImage = segmentedImage(input_image, g_CannyThreshold, 2*g_CannyThreshold, g_dilate);

	// 2.2 regroup them in contours.
	// 2.3 approximate contours by a polynomial (curves/lines).
	ContoursAndLines contours = ContoursAndLines(myImage.getEdgeImage(),
			g_minimumLengthOfAcceptedContours,
			g_maximumLengthOfAcceptedContours,
			g_contourApproxOrder,
			g_line_size_for_plotting
	);

	cnt_img = contours.getContoursImage();

	// 2.4 approximate contours by lines of fixed size.

	lines = contours.getLines();
	linesImage = contours.linesImage;

	// 2.5 Link and connect Lines (algorithm explained in the article).
	LinkedLines linkedlines = LinkedLines(lines);
	vector<vector<Vec4f> > rawLines = linkedlines.getRawLines();
	linkedlines.connectRawLines(rawLines, g_distance_between_lines);

	vector<vector<Vec4f> > connectedLines = linkedlines.getConnectedLines();
	linkedlines.linkConnectedLines(connectedLines, g_angle_threshold_ratio);
	vector<vector<Vec4f> > lLines = linkedlines.getLinkedLines();
	linkedlines.plotLines(lLines, g_line_size_for_plotting, filteredLinesImage);

	// 2.4 regroup contours into potential ellipses and choose the "best" one.

	filteredLinesImage.copyTo(temp);
	MyEllipses ml = MyEllipses(lLines, temp, myImage.getEdgeImage());
	//filteredLinesImage = ml.ellipsesImage;
	filteredLinesImage = ml.filteredEllipsesImage;
	eImagePlusEllipses = ml.edgeImagePlusEllipses;
	finalEdgeImage = eImagePlusEllipses;

	// Plot the hoop on the disparity image.
	cv::RotatedRect hoop = ml.chosenEllipse;
	cv::Scalar color = cv::Scalar( 255,0,0 );
	//cv::ellipse(imgDepthColor, hoop, color, 2, 8);

	// Going to 3D hoop + plane fitting..
	if( hoop.center.x != 0 && hoop.center.y != 0){
		HoopPosition hp = HoopPosition(depthImage, imgDepthColor, hoop, numberOfPointsForDepthOfHoop);
		imgDepthColor = hp.disparityImageWithPlane;
		//imgDepthColor = hp.depthImage;
		//cv::imshow("imgDepthColor+hoop+plane", imgDepthColor);

		// Here, the plane is fitted, so we'll instantiate a class
		// of destination3dPoint to estimate the destination point in 3D for the trajectory.

		destination3dPoint pointIn3D = destination3dPoint(hp);
		cv::imshow("imgDepthColor+hoop+plane", pointIn3D.img);
	}

	if (plotOption){
		int newSizeX = cnt_img.cols/2 + 40;
		int newSizeY = cnt_img.rows/2;
		int marginBetweenWindows = 20;

		cv::namedWindow("Thresholded Image",0);
		cv::namedWindow( "contours",		0);
		cv::namedWindow("lines",			0);
		cv::namedWindow("filtered lines",	0);



		cv::imshow("Thresholded Image", myImage.getEdgeImage());
		cv::imshow("contours", 			cnt_img);
		cv::imshow("lines", 			linesImage);
		cv::imshow("filtered lines", 	filteredLinesImage);


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
}

CodeContainer::~CodeContainer() {}
