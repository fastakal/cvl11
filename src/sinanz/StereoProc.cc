#include "StereoProc.h"

#include <fstream>
#include <opencv2/imgproc/imgproc.hpp>

bool
StereoProc::init(const std::string& configFile)
{
	// load the stereo calibration file
    std::ifstream fileCalib(configFile.c_str(), std::ifstream::in);

    if (!fileCalib)
    {
    	fprintf(stderr, "# ERROR: Unable to open file %s.\n", configFile.c_str());
        return false;
    }

    // read the filename of the left camera calibration file
    std::string filename;
    std::getline(fileCalib, filename);
    //printf("%s", filename.c_str());
    std::ifstream fileCalibL(filename.c_str(), std::ifstream::in);
    if (!fileCalibL)
    {
    	fprintf(stderr, "# ERROR: Unable to open file %s.\n", filename.c_str());
        return false;
    }

    // read the filename of the right camera calibration file
    std::getline(fileCalib, filename);
    std::ifstream fileCalibR(filename.c_str(), std::ifstream::in);
    if (!fileCalibR)
    {
    	fprintf(stderr, "# ERROR: Unable to open file %s.\n", filename.c_str());
        return false;
    }

    // *** process left camera calibration file ***

    // ignore the header line
	std::string fileHeader;
	std::getline(fileCalibL, fileHeader);

	cv::Mat intrinsicMatL = cv::Mat::eye(3, 3, CV_64F);
	cv::Mat distortionMatL = cv::Mat::zeros(1, 5, CV_64F);

	cv::Size frameSize;
	// Process the values
	fileCalibL >> frameSize.width; fileCalibL >> frameSize.height; // get image size
	fileCalibL >> intrinsicMatL.at<double>(0,2); fileCalibL >> intrinsicMatL.at<double>(1,2); // get camera center
	fileCalibL >> intrinsicMatL.at<double>(0,0); fileCalibL >> intrinsicMatL.at<double>(1,1); // get focal length
	for (int i = 0; i < 5; ++i)
	{
		fileCalibL >> distortionMatL.at<double>(0, i); // get distortion parameters
	}

	// Close the calibration file
	fileCalibL.close();

    // *** process right camera calibration file ***

    // ignore the header line
	std::getline(fileCalibR, fileHeader);

	cv::Mat intrinsicMatR = cv::Mat::eye(3, 3, CV_64F);
	cv::Mat distortionMatR = cv::Mat::zeros(1, 5, CV_64F);

	// Process the values
	fileCalibR >> frameSize.width; fileCalibR >> frameSize.height; // get image size
	fileCalibR >> intrinsicMatR.at<double>(0,2); fileCalibR >> intrinsicMatR.at<double>(1,2); // get camera center
	fileCalibR >> intrinsicMatR.at<double>(0,0); fileCalibR >> intrinsicMatR.at<double>(1,1); // get focal length
	for (int i = 0; i < 5; ++i)
	{
		fileCalibR >> distortionMatR.at<double>(0, i); // get distortion parameters
	}

	// Close the calibration file
	fileCalibR.close();

	// *** process stereo calibration file ***

	cv::Mat rotationVec(3, 1, CV_64F);
	cv::Mat translationVec(3, 1, CV_64F);

    // Process the values
	for (int i = 0; i < 3; ++i)
	{
		fileCalib >> rotationVec.at<double>(i,0); // rotation
	}
	for (int i = 0; i < 3; ++i)
	{
		fileCalib >> translationVec.at<double>(i,0); // translation
	}

    // Close the calibration file
    fileCalib.close();

    // now create the undistortion and rectification maps
    cv::Mat rotationMat;
    cv::Rodrigues(rotationVec, rotationMat);

    cv::Mat R1, R2, P1, P2, Q;
    cv::Rect rectROILeft;

    cv::stereoRectify(intrinsicMatL, distortionMatL,
    		intrinsicMatR, distortionMatR,
    		frameSize, rotationMat, translationVec,
    		R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, -1,
    		cv::Size(0,0), &rectROILeft);

	cv::initUndistortRectifyMap(intrinsicMatL, distortionMatR, R1, P1,
								frameSize, CV_32FC1,
								mUndistortMapXLeft, mUndistortMapYLeft);
	cv::initUndistortRectifyMap(intrinsicMatR, distortionMatR, R2, P2,
								frameSize, CV_32FC1,
								mUndistortMapXRight, mUndistortMapYRight);

	mFocalLength = Q.at<double>(2,3);

	// convert baseline from mm to m
	mBaseline = - 0.001 / Q.at<double>(3,2);

	double cameraCenterX = - Q.at<double>(0,3);
	double cameraCenterY = - Q.at<double>(1,3);

	mIntrinsicMat = cv::Mat::eye(3, 3, CV_32F);
	mIntrinsicMat.at<float>(0,0) = mFocalLength;
	mIntrinsicMat.at<float>(0,2) = cameraCenterX;
	mIntrinsicMat.at<float>(1,1) = mFocalLength;
	mIntrinsicMat.at<float>(1,2) = cameraCenterY;

	//mStereoBM.init(0, 0, 15);

	// set stereo matching parameters
	mStereoBM.state->preFilterSize = 21;
	mStereoBM.state->preFilterCap = 31;
	mStereoBM.state->SADWindowSize = 21;
	mStereoBM.state->textureThreshold = 5;
	mStereoBM.state->uniquenessRatio = 15;
	mStereoBM.state->speckleWindowSize = 21;
	mStereoBM.state->speckleRange = 1;
	mStereoBM.state->minDisparity = ceilf(sqrtf(mBaseline * mFocalLength * 0.5 / 0.4));

	return true;
}

bool
StereoProc::getImageInfo(cv::Mat& intrinsicMat) const
{
	mIntrinsicMat.copyTo(intrinsicMat);

	return true;
}

bool
StereoProc::process(cv::Mat& img1, cv::Mat& img2,
					cv::Mat& imgRectified, cv::Mat& imgDepth)
{
	cv::remap(img1, mImgRectifiedLeft,
			  mUndistortMapXLeft, mUndistortMapYLeft,
			  cv::INTER_LINEAR);
	cv::remap(img2, mImgRectifiedRight,
			  mUndistortMapXRight, mUndistortMapYRight,
			  cv::INTER_LINEAR);
	mStereoBM(mImgRectifiedLeft, mImgRectifiedRight, mImgDisparity, CV_32F);
	mImgRectifiedLeft.copyTo(imgRectified);
	float minDisparity = mStereoBM.state->minDisparity;

	imgDepth = cv::Mat::zeros(mImgRectifiedLeft.size(), CV_32F);

	// project disparity to depth
	for (int32_t r = 0; r < mImgDisparity.rows; ++r)
	{
		const float* disparity = mImgDisparity.ptr<float>(r);
		float* depth = imgDepth.ptr<float>(r);

		for (int32_t c = 0; c < mImgDisparity.cols; ++c)
		{
			if (disparity[c] > minDisparity)
			{
				depth[c] = mBaseline * mFocalLength / disparity[c];
			}
		}
	}
	return true;
}

