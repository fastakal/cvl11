/*
 * destination3dPoint.h
 *
 *  Created on: Dec 5, 2011
 *      Author: root
 */

#ifndef DESTINATION3DPOINT_H_
#define DESTINATION3DPOINT_H_
#include "HoopPosition.h"

/**
 * For a plane given by the equation ax + by + cz + d = 0, the vector (a,b,c) is a normal.
 * According to our model: Ax + By + C - z = 0 ==> the normal is (A, B, -1);
 *
 */
class destination3dPoint {
public:
	HoopPosition hoop;
	cv::Vec3f normalVector;
	cv::Mat img;

	destination3dPoint(HoopPosition hoop);
	virtual ~destination3dPoint();

	/**
	 * A function that will calculate the normal to the fitted plane.
	 */
	void getNormalVector();

	/**
	 * A function that will plot the normal vector on the image that contains the fitted plane.
	 */
	void plotNormalVector();
};

#endif /* DESTINATION3DPOINT_H_ */
