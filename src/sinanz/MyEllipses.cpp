/*
 * MyEllipses.cpp
 *
 *  Created on: Nov 18, 2011
 *      Author: root
 */

#include "MyEllipses.h"

MyEllipses::MyEllipses(cv::vector<cv::vector<cv::Vec4f>> lLines, cv::Mat eImage) {

	linkedLines = lLines;
	edgeImage = eImage;
	ExtractPointsFromLines();
	fitEllipses();
	eliminateEllipses();
	rejectEllipsesFromEdgeImage();
}

MyEllipses::~MyEllipses() {
}

/**
 * This function simply takes one point out of each line. Thus returning a vector of vector of points. This is needed
 * for fitting the ellipses as we need points not lines.
 */
void MyEllipses::ExtractPointsFromLines(){
	int size = linkedLines.size();

	cv::vector<cv::vector<cv::Point> > points;
	points.resize(size);
	cv::vector<cv::Vec4f> currentSetOfLines;

	int sizeOfLinesInSet;
	cv::Vec4f currentLine;
	cv::Point aPoint;

	for( int i = 0; i < size; i++){
		cv::vector<cv::Point> currentSetOfPoints;
		currentSetOfLines = linkedLines[i];
		sizeOfLinesInSet = currentSetOfLines.size();
		currentSetOfPoints.resize(sizeOfLinesInSet);

		for( int j = 0; j < sizeOfLinesInSet; j++){
			currentLine = currentSetOfLines[j];
			aPoint.x = currentLine[2];
			aPoint.y = currentLine[3];
			currentSetOfPoints[j] = aPoint;
		}
		points[i] = currentSetOfPoints;
	}
	linkedPoints = points;
}

/**
 * This function fits ellipses to each set of points (contour).
 */
void MyEllipses::fitEllipses(){
	int size = linkedPoints.size();
	cv::vector<cv::Point> currentContour;
	cv::vector<cv::RotatedRect> minEllipse(size);
	cv::Scalar color = cv::Scalar( 255,0,0 );
	int validEllipsesCounter = 0;

	for( int i = 0; i < size; i++){
		currentContour = linkedPoints[i];
		if( currentContour.size()>5){
			minEllipse[validEllipsesCounter] = cv::fitEllipse(cv::Mat(currentContour));
			//cv::ellipse( ellipsesImage, minEllipse[i], color, 2, 8 );
			validEllipsesCounter++;
		}
	}
	minEllipse.resize(validEllipsesCounter);
	fittedEllipses = minEllipse;
}

/**
 * A function that takes the fittedEllipses variable, and filters them according to the following criterias:
 * A potential hoop ellipse should have a ratio between axes that is bigger than 0.5.
 *
 */
void MyEllipses::eliminateEllipses(){
	int noOfEllipses = fittedEllipses.size();
	cv::RotatedRect currentEllipse;
	cv::vector<cv::RotatedRect> filterEllipses;
	filterEllipses.resize(noOfEllipses);
	int validEllipsesCounter = 0;
	cv::Scalar color = cv::Scalar( 255,0,0 );
	cv::vector<cv::Point> currentPoints;
	cv::vector<cv::vector<cv::Point> > points;
	points.resize(noOfEllipses);

	for( int i = 0; i < noOfEllipses; i++){
		currentEllipse = fittedEllipses[i];
		currentPoints = linkedPoints[i];

		// If the current ellipse is validated + error measure is less than the threshold, then accept it.
		if(validateEllipse(currentEllipse) &&
				errorMeasureEllipse(currentEllipse, currentPoints)){
			points[validEllipsesCounter] = currentPoints;
			filterEllipses[validEllipsesCounter] = currentEllipse;
			//cv::ellipse( filteredEllipsesImage, currentEllipse, color, 2, 8 );
			validEllipsesCounter++;
		}
	}
	points.resize(validEllipsesCounter);
	filterEllipses.resize(validEllipsesCounter);
	filteredPoints = points;
	filteredEllipses = filterEllipses;
}

/**
 * A function that validates an ellipse according to the criteria in eliminateEllipses(), used by eliminateEllilpses().
 */
bool MyEllipses::validateEllipse(cv::RotatedRect anEllipse){
	float h = anEllipse.size.height;
	float w = anEllipse.size.width;
	float relativeRatio = h/w;

	bool relativeAxesCriteria = relativeRatio<4;
	bool sizeCriteria = anEllipse.size.area()> 10000;

	return  sizeCriteria & relativeAxesCriteria;
}

/**
 * Another function to validate ellipses. Based on having an error measure of
 * points of contours with respect to their respective position
 * on the fitted ellipse.
 */
bool MyEllipses::errorMeasureEllipse(cv::RotatedRect anEllipse, cv::vector<cv::Point> setOfPoints){

	/* Equation of an ellipse
 	 (x-h)^2/a^2 + (y-k)^2/b^2 = 1
	where,
	(h,k) are the coordinates of the center of the ellipse
	a is the length of the semi-major or minor axis
	b is the length of the semi-major or minor axis
	 */
	float h = anEllipse.center.x;
	float k = anEllipse.center.y;

	float a = anEllipse.size.width / 2;
	float b = anEllipse.size.height / 2;
	float diff = 0;

	int size = setOfPoints.size();

	for( int i = 0; i < size; i++){
		float xx = (float) (setOfPoints[i].x) - h;
		float yy = (float) (setOfPoints[i].y) - k;

		float common = a*b/(float) (sqrt((double) ((b*xx)*(b*xx) + (a*yy)*(a*yy))));
		float xIntersection = xx*common;
		float yIntersection = yy*common;

		//distance = sqrt((xx-x)2 + (yy-y)2)
		float currentDiff = (float) sqrt((xx-xIntersection)*(xx-xIntersection) + (yy-yIntersection)*(yy-yIntersection));
		diff = diff + currentDiff;
	}
	diff = diff/size;
	return diff < 100;
}

void MyEllipses::plotEllipsesOnEdgeImage(){
	cv::Mat tempImage = cv::Mat::zeros(edgeImage.size(), CV_8UC3);
	edgeImage.copyTo(tempImage);
	int size = filteredEllipses.size();
	cv::Scalar color = cv::Scalar( 255,0,0 );

	for(int i = 0; i < size; i++){
		cv::ellipse( tempImage, filteredEllipses[i], color, 2, 8 );
	}
	edgeImagePlusEllipses = tempImage;
}

/**
 * A function that will take the edge image and the ellipses, then will calculate
 * how many edge points lie on the ellipses in respect to the whole tested points.
 */
void MyEllipses::rejectEllipsesFromEdgeImage(){
	/* Equation of an ellipse
 	 (x-h)^2/a^2 + (y-k)^2/b^2 = 1
	where,
	(h,k) are the coordinates of the center of the ellipse
	a is the length of the semi-major or minor axis
	b is the length of the semi-major or minor axis
	 */
	int size = filteredEllipses.size();
	int numberOfPoints = 40;
	float x, y;
	cv::Scalar color = cv::Scalar(255,0,0);
	cv::vector<cv::RotatedRect> filtEl; filtEl.resize(size);
	cv::vector<double> qualityOfEllipses; qualityOfEllipses.resize(size);
	double ratioOfCorrectPoints;
	double acceptableRatioOfCorrectPoints = 0.6;
	int correctEllipsesCounter = 0;

	for( int i = 0; i < size ; i++){

		cv::RotatedRect anEllipse = filteredEllipses[i];
		float h = anEllipse.center.x;
		float k = anEllipse.center.y;
		float ellipseAngle = anEllipse.angle * 3.14 / 180.0;

		float a = anEllipse.size.width / 2;
		float b = anEllipse.size.height / 2;


		int correctPointsCounter = 0;
		for( int j = 0; j<numberOfPoints; j++){
			float angle = 2*3.14*(double(j)/double(numberOfPoints));
			x = h + a*cos(angle)*cos(ellipseAngle) - b*sin(angle)*sin(ellipseAngle);
			y = k + a*cos(angle)*sin(ellipseAngle) + b*sin(angle)*cos(ellipseAngle);

			correctPointsCounter  = correctPointsCounter + getPointsValue((int) x, (int) y);
		}

		ratioOfCorrectPoints = (double) correctPointsCounter / (double) numberOfPoints;
		if( ratioOfCorrectPoints > acceptableRatioOfCorrectPoints){
			filtEl[correctEllipsesCounter] = anEllipse;
			qualityOfEllipses[correctEllipsesCounter] = ratioOfCorrectPoints;
			correctEllipsesCounter++;
		}
	}
	filtEl.resize(correctEllipsesCounter);
	qualityOfEllipses.resize(correctEllipsesCounter);

	filteredEllipses = filtEl;
	if(filtEl.size())
		chosenEllipse = getBestEllipse(filtEl, qualityOfEllipses);
}

/**
 * A function that takes a position (x and y) and returns 1 if the value in that neighbourhood is 255, 0 otherwise.
 */
int MyEllipses::getPointsValue(int x,int y){
	int patchSize = 3;
	int value = 0;

	if( !( (x + patchSize) < edgeImage.cols &&
			(y + patchSize) < edgeImage.rows &&
			(x - patchSize) > 0 &&
			(y - patchSize) > 0))
		return 0;


	for(int i = -patchSize; i < patchSize; i++){
		for( int j = -patchSize; j < patchSize; j++){
			value = value + (int) edgeImage.at<uchar>(y + i,x + j);
		}
	}

	if (value>0)
		value = 1;
	return value;
}

/**
 * A function that takes a list of ellipses with their respective quality and returns the best one.
 */
cv::RotatedRect MyEllipses::getBestEllipse(cv::vector<cv::RotatedRect> ellipses, cv::vector<double> qualityOfEllipses){
	int size = ellipses.size(); std::cout<<size;
	double maxQuality = 0;
	int index = 0;

	for(int i = 0; i < size; i++){
		if( qualityOfEllipses[i]>maxQuality ){
			maxQuality = qualityOfEllipses[i];
			index = i;
		}
	}
	return ellipses[index];
}

void MyEllipses::plot(cv::Mat lImage){

	lImage.copyTo(ellipsesImage);
	cv::Scalar color = cv::Scalar( 255,0,0 );
	int size = fittedEllipses.size();

	for( int i = 0; i < size; i++){
		cv::ellipse( ellipsesImage, fittedEllipses[i], color, 2, 8 );
	}

	edgeImagePlusEllipses = edgeImage;
	cv::ellipse( edgeImagePlusEllipses, chosenEllipse, color, 2, 8 );
}
