/*=====================================================================
This file is inspired from the project done by Lorenz.
=====================================================================*/

#include <cstdio>
#include <unistd.h>
#include <glib.h>
#include "mavconn.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <signal.h>
#include <interface/shared_mem/PxSHMImageClient.h>

// Latency Benchmarking
#include <sys/time.h>
#include <time.h>

// Timer for benchmarking & a counter vector for stats.
struct timeval tv;

int sysid = 42;
int compid = 112;
bool verbose = false;
bool debug = 0;

static GString* configFile = g_string_new("conf/abc.cfg");

std::string fileBaseName("frame");
std::string fileExt(".png");

bool quit = false;

// Include files and declare global variables in "globals.h".
// Includes
#include "StereoProc.h"
#include "CodeContainer.h"
#include "globals.h"

float roll, pitch, yaw;
bool initialize = true;
//////// Functions and Structures

struct united {
	PxSHMImageClient *imageClient;
	lcm_t *lcm;
};

/**
 * A function that creates a window with trackbars controlling parameters to test...
 */
void createControlPanel(){
	namedWindow("Control Panel");
	createTrackbar("Threshold", "Control Panel", 				&g_CannyThreshold, 255, 0);
	createTrackbar("MinLengthContours", "Control Panel", 		&g_minimumLengthOfAcceptedContours, 500, 0);
	createTrackbar("MaxLengthContours", "Control Panel", 		&g_maximumLengthOfAcceptedContours, 100000, 0);
	createTrackbar("contourApproxOrder", "Control Panel", 		&g_contourApproxOrder, 10, 0);
	createTrackbar("dilate", "Control Panel", 					&g_dilate, 1, 0);
	createTrackbar("Line-Size for plotting", "Control Panel", 	&g_line_size_for_plotting, 100, 0);
	createTrackbar("Distance between lines", "Control Panel", 	&g_distance_between_lines, 10000, 0);
	createTrackbar("Angle Threshold", "Control Panel", 			&g_angle_threshold_ratio, 10, 0);
}

void
colorDepthImage(cv::Mat& imgDepth, cv::Mat& imgColoredDepth)
{
	imgColoredDepth = cv::Mat::zeros(imgDepth.size(), CV_8UC3);

	for (int i = 0; i < imgColoredDepth.rows; ++i)
	{
		const float* depth = imgDepth.ptr<float>(i);
		unsigned char* pixel = imgColoredDepth.ptr<unsigned char>(i);
		for (int j = 0; j < imgColoredDepth.cols; ++j)
		{
			if (depth[j] != 0)
			{
				int idx = fmin(depth[j], 10.0f) / 10.0f * 127.0f;
				idx = 127 - idx;

				pixel[0] = colormap_jet[idx][2] * 255.0f;
				pixel[1] = colormap_jet[idx][1] * 255.0f;
				pixel[2] = colormap_jet[idx][0] * 255.0f;
			}

			pixel += 3;
		}
	}
}

void plotInputImages(cv::Mat imgL, cv::Mat imgR, cv::Mat imgDepthColor){

	int newSizeX = imgL.cols/2;
	int newSizeY = imgL.rows/2;
	int marginBetweenWindows = 20;

	cv::namedWindow("Left Image",	0);
	cv::namedWindow("Right Image",	0);
	cv::namedWindow("Depth map",	0);

	cv::imshow("Left Image", 	imgL);
	cv::imshow("Right Image", 	imgR);
	cv::imshow("Depth map",		imgDepthColor);

	cvResizeWindow("Left Image", 	newSizeX, newSizeY);
	cvResizeWindow("Right Image", 	newSizeX, newSizeY);
	cvResizeWindow("Depth map",		newSizeX, newSizeY);

	cvMoveWindow("Left Image", 		marginBetweenWindows, 0);
	cvMoveWindow("Right Image", 	newSizeX + 2*marginBetweenWindows, 0);
	cvMoveWindow("Depth map", 		2*(newSizeX + 2*marginBetweenWindows), 0);

	cv::waitKey(1);
}

void printTimingStats(){
	timingHistory.resize(globalFrameCounter);
	printf("\n\n\n");
	for(int i = 0; i < globalFrameCounter; i++){
		printf("Frame: %d, Time: %f\n", i, timingHistory[i]);
	}
}

void signalHandler(int signal) {
	if (signal == SIGINT) {
		fprintf(stderr, "# INFO: Quitting...\n");

		printTimingStats();

		quit = true;
		exit(EXIT_SUCCESS);
	}
}

double getTimeNow(){
	struct 	timeval tp;
	double sec, usec, time;

	gettimeofday(&tp, NULL);
	sec = static_cast<double>( tp.tv_sec);
	usec = static_cast<double>( tp.tv_usec) / 1E6;

	time = sec + usec;

	return time;
}

/**
 * @brief Handle incoming MAVLink packets containing images
 *
 */
void imageHandler(const lcm_recv_buf_t* rbuf, const char* channel,
		const mavconn_mavlink_msg_container_t* container, void* user) {

	const mavlink_message_t* msg = getMAVLinkMsgPtr(container);

	// Pointer to shared memory data

	struct united *clientHandler = static_cast<struct united *>(user);
	PxSHMImageClient* client = clientHandler->imageClient;
	lcm_t *lcm = clientHandler->lcm;


	printf("GOT IMG MSG\n");

	// read image data
	Mat imgL = Mat::zeros(480, 640, CV_8UC3);
	Mat imgR = Mat::zeros(480, 640, CV_8UC3);
	Mat imgDepth = Mat::zeros(480, 640, CV_32F);
	cv::Mat imgDepthColor;
	cv::Mat imgRectified;
	cv::Mat imgToSave;
	cv::Mat intrinsicMat;


	StereoProc imgproc;
	imgproc.init("/home/sinan/src/data_sets/myTemplate/calib_stereo_bravo_bluefox.scf");
	//imgproc.init("/home/sinan/src/data_sets/newData/20111122_112212/calib_stereo_bravo_bluefox.scf");
	//imgproc.init("/home/pixhawk/pixhawk/ai_vision/release/config/calib_stereo_bravo_front.scf");
	imgproc.getImageInfo(intrinsicMat);
	cv::Mat inverseIntrinsicMat;
	inverseIntrinsicMat = intrinsicMat;
	cv::invert(inverseIntrinsicMat, inverseIntrinsicMat);

	if (client->readStereoImage(msg, imgL, imgR)) {

		mavlink_set_local_position_setpoint_t pos;
		mavlink_set_local_position_setpoint_t lastPosition;
		mavlink_message_t msgp;

		if(initialize){
			client->getRollPitchYaw(msg, roll, pitch, yaw);
			float x, y, z;
			client->getGroundTruth(msg, x, y, z);
			pos.x = x;
			pos.y = y;
			pos.z = z;
			pos.yaw = yaw;
			pos.target_system = getSystemID();
			pos.target_component = 200;
			pos.coordinate_frame = 1;


			initialize = false;

		}


		double startTime, endTime;


		// Compute Stereo and store the processed frames in buffer
		imgproc.process(imgL, imgR, imgRectified, imgDepth);

		colorDepthImage(imgDepth, imgDepthColor);
		cv::Mat img3 = cv::Mat::zeros(imgDepthColor.size(), imgDepthColor.type());
		imgDepthColor.copyTo(img3);

		if( g_plot){
			plotInputImages(imgL, imgR, imgDepthColor);
		}

		createControlPanel();
		double angle_treshold = ((double)g_angle_threshold_ratio)/10;

		startTime = getTimeNow();
		CodeContainer myCode1 = CodeContainer(imgRectified, img3, imgDepth,
				inverseIntrinsicMat,
				client,
				msg,
				g_minimumLengthOfAcceptedContours,
				g_maximumLengthOfAcceptedContours,
				g_CannyThreshold,
				g_contourApproxOrder,
				g_dilate,
				g_line_size_for_plotting,
				g_distance_between_lines,
				g_plot,
				angle_treshold
		);

		/*
		if(g_plot){
			char filename [21];
			sprintf(filename, "recording/toto_%d.jpg", globalFrameCounter);
			imwrite(filename, myCode1.depthWithEllipse );
		}
		 */

		// Timestamp after the computation.
		endTime = getTimeNow();

		// Time calculation in Seconds.
		double time = endTime - startTime;

		std::cout<<"############################# Total Time: "<<time<<"############################# \n";
		timingHistory[globalFrameCounter] = time;
		globalFrameCounter++;

		if( myCode1.endPoint.z != 0){

			pos.x = myCode1.endPoint.x;
			pos.y = myCode1.endPoint.y;
			pos.z = myCode1.endPoint.z;
			pos.yaw = yaw;
			pos.target_system = getSystemID();
			pos.target_component = 200;
			pos.coordinate_frame = 1;

			mavlink_msg_set_local_position_setpoint_encode(getSystemID(),compid, &msgp, &pos);
			sendMAVLinkMessage(lcm, &msgp);

			lastPosition = pos;

			printf("Sent a message with destination point: x: %f, y: %f, z: %f. \n", pos.x, pos.y, pos.z);
		} else {
			mavlink_msg_set_local_position_setpoint_encode(getSystemID(),compid, &msgp, &lastPosition);
			sendMAVLinkMessage(lcm, &msgp);

			printf("didn't send a new message, point is zero");
		}


		struct timeval tv;
		gettimeofday(&tv, NULL);
		uint64_t currTime = ((uint64_t) tv.tv_sec) * 1000000 + tv.tv_usec;
		uint64_t timestamp = client->getTimestamp(msg);

		uint64_t diff = currTime - timestamp;

		if (verbose) {
			fprintf(
					stderr,
					"# INFO: Time from capture to display: %llu ms for camera %llu\n",
					diff / 1000, client->getCameraID(msg));
		}

		imgL.copyTo(imgToSave);
	}

#ifndef NO_DISPLAY
	int c = cv::waitKey(3);
	switch (static_cast<char>(c)) {
	case 'f': {
		char index[20];
		sprintf(index, "%04d", imageCounter++);
		cv::imwrite(std::string(fileBaseName + index + fileExt).c_str(),
				imgToSave);
	}
	break;
	case 'w': {
		std::cout << "\nFreeze for 2 seconds...\n";
		cv::waitKey(2000);
	}
	break;
	default:
		break;
	}
#endif
}

static void mavlink_handler(const lcm_recv_buf_t *rbuf, const char * channel,
		const mavconn_mavlink_msg_container_t* container, void * user) {
	const mavlink_message_t* msg = getMAVLinkMsgPtr(container);
	mavlink_message_t response;
	lcm_t* lcm = static_cast<lcm_t*>(user);
	//printf("Received message #%d on channel \"%s\" (sys:%d|comp:%d):\n",
	//	msg->msgid, channel, msg->sysid, msg->compid);

	switch (msg->msgid) {
	uint32_t receiveTime;
	uint32_t sendTime;

	case MAVLINK_MSG_ID_COMMAND_LONG: {
		mavlink_command_long_t cmd;
		mavlink_msg_command_long_decode(msg, &cmd);
		printf("Message ID: %d\n", msg->msgid);
		printf("Command ID: %d\n", cmd.command);
		printf("Target System ID: %d\n", cmd.target_system);
		printf("Target Component ID: %d\n", cmd.target_component);
		printf("\n");

		if (cmd.confirmation) {
			printf("Confirmation requested, sending confirmation:\n");
			mavlink_command_ack_t ack;
			ack.command = cmd.command;
			ack.result = 3;
			mavlink_msg_command_ack_encode(getSystemID(), compid, &response, &ack);
			sendMAVLinkMessage(lcm, &response);
		}
	}
	break;
	case MAVLINK_MSG_ID_ATTITUDE:
		gettimeofday(&tv, NULL);
		receiveTime = tv.tv_usec;
		sendTime = mavlink_msg_attitude_get_time_boot_ms(msg);
		printf("Received attitude message, transport took %f ms\n",
				(receiveTime - sendTime) / 1000.0f);
		break;
	case MAVLINK_MSG_ID_GPS_RAW_INT: {
		mavlink_gps_raw_int_t gps;
		mavlink_msg_gps_raw_int_decode(msg, &gps);
		printf("GPS: lat: %f, lon: %f, alt: %f\n", gps.lat / (double) 1E7,
				gps.lon / (double) 1E7, gps.alt / (double) 1E6);
		break;
	}
	case MAVLINK_MSG_ID_RAW_PRESSURE: {
		mavlink_raw_pressure_t p;
		mavlink_msg_raw_pressure_decode(msg, &p);
		printf("PRES: %f\n", p.press_abs / (double) 1000);
	}
	break;
	default:
		//	printf("ERROR: could not decode message with ID: %d\n", msg->msgid);
		break;
	}
}

void* lcm_wait(void* lcm_ptr) {
	lcm_t* lcm = (lcm_t*) lcm_ptr;
	// Blocking wait for new data
	while (1) {
		lcm_handle(lcm);
	}
	return NULL;
}

// Handling Program options
static GOptionEntry entries[] = { { "sysid", 'a', 0, G_OPTION_ARG_INT, &sysid,
		"ID of this system, 1-255", "42" }, { "compid", 'c', 0,
				G_OPTION_ARG_INT, &compid, "ID of this component, 1-255", "55" }, {
						"verbose", 'v', 0, G_OPTION_ARG_NONE, &verbose, "Be verbose",
						(verbose) ? "true" : "false" }, { "debug", 'd', 0,
								G_OPTION_ARG_NONE, &debug, "Debug mode, changes behaviour",
								(debug) ? "true" : "false" }, { "config", 'g', 0,
										G_OPTION_ARG_STRING, configFile, "Filename of paramClient config file",
										"config/parameters_2pt.cfg" },
										{ "plot", 'p', 0,
												G_OPTION_ARG_NONE, &g_plot, "Plot everything",
												(g_plot) ? "true" : "false" },
												{ NULL } };

int main(int argc, char* argv[]) {


	timingHistory.resize(maxNumberOfFrames);

	GError *error = NULL;
	GOptionContext *context;

	context = g_option_context_new("- localize based on natural features");
	g_option_context_add_main_entries(context, entries, "Localization");
	if (!g_option_context_parse(context, &argc, &argv, &error)) {
		g_print("Option parsing failed: %s\n", error->message);
		exit(EXIT_FAILURE);
	}
	g_option_context_free(context);

	// Handling Program options
	struct united clientHandler;
	lcm_t* lcm;
	lcm = lcm_create("udpm://");
	clientHandler.lcm = lcm;

	if (!lcm) {
		fprintf(stderr, "# ERROR: Cannot initialize LCM.\n");
		exit(EXIT_FAILURE);
	}

	mavconn_mavlink_msg_container_t_subscription_t * comm_sub =
			mavconn_mavlink_msg_container_t_subscribe(lcm, MAVLINK_MAIN,
					&mavlink_handler, lcm);

	// Thread
	GThread* lcm_thread;
	GError* err;

	if (!g_thread_supported()) {
		g_thread_init(NULL);
		// Only initialize g thread if not already done
	}

	if ((lcm_thread =
			g_thread_create((GThreadFunc)lcm_wait, (void *)lcm, TRUE, &err))
			== NULL) {
		printf("Thread create failed: %s!!\n", err->message);
		g_error_free(err);
	}

	PxSHMImageClient client;
	client.init(true, PxSHM::CAMERA_FORWARD_LEFT, PxSHM::CAMERA_FORWARD_RIGHT);
	// Ready to roll
	fprintf(stderr, "# INFO: Image client ready, waiting for images..\n");
	clientHandler.imageClient = &client;

	// Subscribe to MAVLink messages on the image channel
	mavconn_mavlink_msg_container_t_subscription_t* imgSub =
			mavconn_mavlink_msg_container_t_subscribe(lcm, MAVLINK_IMAGES,
					&imageHandler, &clientHandler);

	signal(SIGINT, signalHandler);

	while (!quit) {
		// Block waiting for new image messages,
		// once an image is received it will be
		// displayed
		lcm_handle(lcm);
	}

	mavconn_mavlink_msg_container_t_unsubscribe(lcm, imgSub);
	mavconn_mavlink_msg_container_t_unsubscribe(lcm, comm_sub);
	lcm_destroy(lcm);
	g_thread_join(lcm_thread);
	return 0;
}
