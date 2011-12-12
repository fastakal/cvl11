/*
 * CodeContainer.h
 *
 *  Created on: Nov 15, 2011
 *      Author: root
 */

#ifndef CODECONTAINER_H_
#define CODECONTAINER_H_

#include <cstdio>
#include <unistd.h>
#include <glib.h>
#include "mavconn.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <signal.h>
// Include files and declare global variables.
// Including header file
#include "MyEllipses.h"
#include "segmentedImage.h"
#include "ContoursAndLines.h"
#include "LinkedLines.h"
#include "HoopPosition.h"
#include "destination3dPoint.h"

#include <interface/shared_mem/PxSHMImageClient.h>

class CodeContainer {

public:
	cv::Mat depthImage;
	cv::Mat finalEdgeImage;
	int numberOfPointsForDepthOfHoop;

	CodeContainer(cv::Mat, cv::Mat,cv::Mat, PxSHMImageClient*, const mavlink_message_t*, int, int, int, int, int, int, int, bool, double);
	virtual ~CodeContainer();
	void createControlPanel();
	double getTimeNow();
	void plotPipeLineImages(cv::Mat firstImage,cv::Mat secondImage,cv::Mat thirdImage,cv::Mat fourthImage);
};

#endif /* CODECONTAINER_H_ */
