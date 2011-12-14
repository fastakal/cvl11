/*
 * destination3dPoint.h
 *
 *  Created on: Dec 5, 2011
 *      Author: root
 */

#ifndef DESTINATION3DPOINT_H_
#define DESTINATION3DPOINT_H_
#include "HoopPosition.h"
#include <interface/shared_mem/PxSHMImageClient.h>

/**
 *
 */
class destination3dPoint {
public:
	HoopPosition hoop;
	cv::Vec3f normalVector;
	cv::Mat img;
	cv::Point3f startPoint, endPoint;
	PxSHMImageClient* client;
	const mavlink_message_t* msg;
	CvMat *rotationMatrix;
	cv::Vec3f translationVector;
	cv::Point3f secondPoint;
	cv::Vec3f trajectoryVector;

	destination3dPoint(HoopPosition hoop, PxSHMImageClient* cl, const mavlink_message_t* message);
	virtual ~destination3dPoint();

	/**
	 * A function that will calculate the normal to the fitted plane.
	 */
	void getNormalVector();

	/**
	 * A function that will plot the normal vector on the image that contains the fitted plane.
	 */
	void plotNormalVector();

	/**
	 * A function that constructs the 3D rotation matrix using the roll, pitch and yaw from the client.
	 */
	void constructRotationMatrix();

	void constructTranslationVector();

	/**
	 * A function that will get the second point (the point that's half a meter in front of the hoop and perpedicular.
	 */
	void get2ndPoint();

	/**
	 * A function that will get the satrting point (the world's coordinates of the helicopter).
	 */
	void getStartPoint();

	/**
	 * A function that will get the destination point (the world's coordinates of the shifted centroid of the hoop).
	 * This will simply multiply the rotation matrix by the image coordinates of the hoop in 3D.
	 */
	void getEndPoint();

	/**
	 * A function that will calculate the trajectory vector of the helicopter. Basically, it's a straight line from the startPoint to the endPoint.
	 */
	void getTrajectoryVector();

	void printAll();
};

#endif /* DESTINATION3DPOINT_H_ */
