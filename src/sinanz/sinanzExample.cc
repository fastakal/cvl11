/*=====================================================================

 MAVCONN Micro Air Vehicle Flying Robotics Toolkit
 Please see our website at <http://MAVCONN.ethz.ch>

 (c) 2009, 2010 MAVCONN PROJECT

 This file is part of the MAVCONN project

 MAVCONN is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 MAVCONN is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with MAVCONN. If not, see <http://www.gnu.org/licenses/>.

 ======================================================================*/

/**
 * @file
 *   @brief LCM example
 *
 *   @author Lorenz Meier <mavteam@student.ethz.ch>
 *
 */

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

using namespace cv;
// Timer for benchmarking
struct timeval tv;

int sysid = 42;
int compid = 112;
bool verbose = false;
bool debug = 0;

static GString* configFile = g_string_new("conf/abc.cfg");

int imageCounter = 0;
std::string fileBaseName("frame");
std::string fileExt(".png");

bool quit = false;

void signalHandler(int signal) {
	if (signal == SIGINT) {
		fprintf(stderr, "# INFO: Quitting...\n");
		quit = true;
		exit(EXIT_SUCCESS);
	}
}

/**
 * @brief Handle incoming MAVLink packets containing images
 *
 */
void imageHandler(const lcm_recv_buf_t* rbuf, const char* channel,
		const mavconn_mavlink_msg_container_t* container, void* user) {
	const mavlink_message_t* msg = getMAVLinkMsgPtr(container);

	// Pointer to shared memory data
	PxSHMImageClient* client = static_cast<PxSHMImageClient*>(user);

	cv::Mat imgToSave;

	printf("GOT IMG MSG\n");
	// read mono image data
	Mat img;
	Mat temp;
	Mat cnt_img;

	// Added Declarations
	//////////////////////////////////////////////////////////////////////////////////////
	// CONTOURS
	vector<vector<Point> > contours;
	vector<vector<Point> > contours0;
	vector<vector<Point> > contours_filtered;

	vector<Vec4i> hierarchy;
	vector<Vec4i> hierarchy_filtered;
	cv::namedWindow( "contours", 1 );
	cv::namedWindow("Thresholded Image", 1);

	// Create a font to be used...
	CvFont font;
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 1.0, 1.0, 0, 1, CV_AA);
	//////////////////////////////////////////////////////////////////////////////////////

	if (client->readMonoImage(msg, img)) {
		/*
		 * The pipeline is the following:
		 * 1. Pre-processing: This is mainly gaussian blurring of the image to reduce noise before processing.		 *
		 */
		GaussianBlur(img, img, Size(3, 3), 1, 0, BORDER_DEFAULT);
img.copyTo(temp);

		/*
		 * 2. Detecting Ellipses:
		 * 	2.1 find edges.
		 * 	2.2 regroup them in contours.
		 * 	2.3 approximate contours by a polynomial (curves/lines).
		 * 	2.4 regroup contours into potential ellipses.
		 */

		// 2.1 find edges.
		Canny(temp, temp, 100, 200, 5);

		// 2.2 regroup them in contours.
		findContours( temp, contours0, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

		// 2.3 approximate contours by a polynomial (curves/lines).
		if (contours0.size()){

			contours.resize(contours0.size());

			// Approximate the contours by a polynomial.
			for( size_t k = 0; k < contours0.size(); k++ ) {
				if (contours0[k].size()>100){
					approxPolyDP(Mat(contours0[k]), contours[k], 4, false);


				}else{
					contours[k] = contours0[k];
				}
			}


			// 2.4 regroup contours into potential ellipses.

			/*
			 * 3. Recognizing Ellipses:
			 *  3.1 Fit an ellipse on the potential ellipses by taking 6 random points from a blob and fit an ellipse on it.
			 *  3.2 Check if the mean squared error is within a threshold.
			 *  3.3 accept or reject an ellipse.
			 *
			 * 4.
			 *
			 *
			 */



			imshow("Thresholded Image", temp);


			// For each high-level contour, draw it with a random colour.
			int idx = 0;
			for( ; idx >= 0; idx = hierarchy[idx][0] )
			{
				Scalar color(255,255,255);
				drawContours( cnt_img, contours, idx, color, 0, 8, hierarchy );
			}
		}


		imshow("contours", cnt_img);

		// Add text for info.
		putText(img, "Testing Contours", Point(200, 200), 2, 1, Scalar(0,0,0, 0), 1, 8);

		//////////////////////////////////////////////////////////////////////////////////////




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

		// Display if switched on
#ifndef NO_DISPLAY
		if ((client->getCameraConfig() & PxSHM::CAMERA_FORWARD_LEFT)
				== PxSHM::CAMERA_FORWARD_LEFT) {
			cv::namedWindow("Left Image (Forward Camera)");
			cv::imshow("Left Image (Forward Camera)", img);
		} else {
			cv::namedWindow("Left Image (Downward Camera)");
			cv::imshow("Left Image (Downward Camera)", img);
		}
#endif

		img.copyTo(imgToSave);
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
	case 'a': {
		std::cout << "You just pressed 'a', OMG!\n";
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
	printf("Received message #%d on channel \"%s\" (sys:%d|comp:%d):\n",
			msg->msgid, channel, msg->sysid, msg->compid);

	switch (msg->msgid) {
	uint32_t receiveTime;
	uint32_t sendTime;
	case MAVLINK_MSG_ID_COMMAND_SHORT: {
		mavlink_command_short_t cmd;
		mavlink_msg_command_short_decode(msg, &cmd);
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
		printf("ERROR: could not decode message with ID: %d\n", msg->msgid);
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
										"config/parameters_2pt.cfg" }, { NULL } };

int main(int argc, char* argv[]) {
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
	lcm_t* lcm = lcm_create("udpm://");
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
	client.init(true, PxSHM::CAMERA_DOWNWARD_LEFT);

	// Ready to roll
	fprintf(stderr, "# INFO: Image client ready, waiting for images..\n");

	// Subscribe to MAVLink messages on the image channel
	mavconn_mavlink_msg_container_t_subscription_t* imgSub =
			mavconn_mavlink_msg_container_t_subscribe(lcm, MAVLINK_IMAGES,
					&imageHandler, &client);

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

