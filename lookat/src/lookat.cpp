/*
 * lookat.cpp
 *
 *  Created on: 4 ene. 2017
 *      Author: robert
 */
//OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "opencv2/objdetect.hpp"
#include "opencv2/imgproc.hpp"

//std
#include <iostream>
#include <cmath>

//**************************************************
// DAVID CODE
/*#include "ros/ros.h"
#include "ros/package.h"
#include "geometry_msgs/Point.h"*/
// End DAVID CODE
//**************************************************

//**************************************************
// DAVID CODE
// Defines to control the source of the images

// If Camera is active, the image source is the cámera

//#define Camera

// If the Video is active, the image source is the video file
#define Video


// Max variation constant
const int TOTAL_VARIATION = 10;

using namespace cv;
using namespace std;

/** Function Headers */
Point detectFace(Mat frame, Point lastPoint);
int distEuclidean(int x1,int y1,int x2, int y2);

/** Global variables */
//String face_cascade_name = "haarcascade_frontalface_alt.xml";
CascadeClassifier face_cascade;

int main(int argc, char **argv)
{
	VideoCapture camera;
	Mat frame;
	int cam_id;
	Point prevPoint;

	// Variable to indicate the face is found
	bool found = false;

	// Kalman filter definition

	// Variable to adjust dT
    double ticks = 0;

    // Number of frames without face detetion
    int numFrames = 0;

    //Define the Kalman filter
    cv::KalmanFilter KF(4,2,0);	// 4 dynamic parameters
    							// 2 measure parameters
    							// 0 control variables

    // Variable to store the measures
    cv::Mat measure(2, 1, CV_32F); // 2 rows x 1 column of float

	//**************************************************
	// DAVID CODE
	/*ros::init(argc, argv, "point_publisher");
	ros::NodeHandle n;


	ros::Publisher point_pub = n.advertise<geometry_msgs::Point>("pointpub", 1000);
	ros::Rate loop_rate(10);

	std::string face_cascade_name_std;
	if (!n.getParam("face_cascade_name", face_cascade_name_std))
	      ROS_ERROR("Could not get face_cascade_name");
	cv::String face_cascade_name = face_cascade_name_std;*/

	//Check args
	switch(argc)
	{
		case 1: //no argument provided, so try /dev/video0
			cam_id = 0;
			break;
		case 2: //an argument is provided. Get it and set cam_id
			cam_id = atoi(argv[1]);
			break;
		default:
			std::cout << "Invalid number of arguments. Call program as: webcam_capture [video_device_id]. " << std::endl;
			std::cout << "EXIT program." << std::endl;
			break;
	}


	cout << "Opening video device " << cam_id << endl;

	// Init de Kalman filter
    // Initialize the transition matrix as a identity matrix 4 x 4
    cv::setIdentity(KF.transitionMatrix, cv::Scalar::all(1.0f));

    // Set the identity matrix to meaasurement
    cv::setIdentity(KF.measurementMatrix);

    // Init the measurement Noise covariance matrix as identity matrix of 1e-1
    cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-1));

    // Init the process noise covariance matrix as identity matrix of 1e-2
    cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-2));

    // Init the measure results
    measure.setTo(cv::Scalar::all(0.0f));

	//**************************************************
	// DAVID CODE
    /*//open the video stream and make sure it's opened
    if( !camera.open(cam_id) )
	{
        std::cout << "Error opening the camera. May be invalid device id. EXIT program." << std::endl;
        return -1;
    }

    // Set the size of the captures
	camera.set(CV_CAP_PROP_FRAME_WIDTH, 640);
   	camera.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

//	Load the Cascades
	if( !face_cascade.load( face_cascade_name ) )
	{ 
		printf("--(!)Error loading face cascade\n"); 
		return -1; 
	}

	while (ros::ok())*/


	//**************************************************
	// DAVID CODE

	#if defined Video
		// Init the video file
		std::cout << "The input source is the Video File\n";

		// Loading the video
		camera = cv::VideoCapture("../img/video.mpeg");

		// Verify if the video is opened
		if(!camera.isOpened())                              // Check for invalid video
	    {
	        std::cout <<  "Could not open or find the video" << std::endl;
	        return -1;
	    }
	#elif defined Camera
		// Inicializar la cámara
		std::cout << "The input source is the camera\n";
	    // Open the video stream and make sure it's opened
    	if( !camera.open(cam_id) ) 
		{
	        std::cout << "Error opening the camera. May be invalid device id. EXIT program." << std::endl;
        	return -1;
		}
	#else
		// There is no define uncomented
		std::cout << "Error in the source of video!!\nExit!!";
		return -1;
	#endif

	// Load the XML file to detect faces
    if(!face_cascade.load("../haarcascade_frontalface_default.xml"))
    {
    	std::cout <<  "Could not open of find the XML file" << std::endl;
    	return -1;
    }		

	while(1)
    {
		//**************************************************
		// DAVID CODE
		///geometry_msgs::Point p;
		cv::Point p;

		// Kalman filter center
		cv::Point kalmanCenter(320,240);

		// dT calculation
    	double preTick = ticks;
    	ticks = (double) cv::getTickCount();

        double dT = (ticks - preTick) / cv::getTickFrequency(); // Seconds

		bool bSuccess = camera.read(frame);

	    if (!bSuccess) //if not success, break loop
	    {
		   cout << "Cannot read the frame from video file" << endl;
		   break;
		}

		
	    Point pnt = detectFace(frame, prevPoint);

	    // Verify the detection is correct
	    if((pnt.x!=1000)&&(pnt.y!=1000))
	    {
	    	// There are a face detection
	    	if(found)
		    {
		    	// Only detect if the face is between a margin
		        if(distEuclidean(pnt.x, prevPoint.x, pnt.y, prevPoint.y) <= TOTAL_VARIATION)
		        {
		        	// The point is correct
		        }
		    	// Update the Kalman filter values

		        KF.transitionMatrix.at<float>(3) = dT;
		        KF.transitionMatrix.at<float>(7) = dT;

		        // Predict the new position
		        cv::Mat state(4 , 1, CV_32F); // 4 Results, 1 column, float

		        state = KF.predict();

		        kalmanCenter = cv::Point(state.at<float>(0), state.at<float>(1));

		    }

	    }
	    else
	    {
	    	numFrames++;
	    	// The detection is lost more than 10 frames
	    	if( numFrames >= 10) 
	    	{
	    		found = false;
	    	}
	    }

	    //**************************************************
		// DAVID CODE
		/*point_pub.publish(p);

		ros::spinOnce();
		loop_rate.sleep();*/

		// If the 'q' or 'Esc' is pressed, exit the loop
        int c = waitKey(10);
        if( (char)c == 27 || (char)c == 'q') 
        { 
        	break; 
        } // escape
    } 
	return 0;
}

/** @function detectFacePosition */
// This function returns 1000,1000 if there is no face
// else, returns the nearest face detected from the last detection
Point detectFace(Mat frame, Point lastPoint)
{
	vector<Rect> faces;
    Mat frame_gray;
    Point nearest_face(1000,1000);

    int min_dist=1000;

    cvtColor( frame, frame_gray, COLOR_BGR2GRAY );
    equalizeHist( frame_gray, frame_gray );

    //-- Detect faces
    face_cascade.detectMultiScale(	frame_gray, // Image
									faces, 		// Faces location
									1.1, 		// Scale factor
									4,			// min Neighbors
									0|CV_HAAR_DO_CANNY_PRUNING,			// Flags
									Size(30,30));	// Min Size
    
    if(faces.size()==1){
    		rectangle(frame, faces[0], cv::Scalar(255,0,0), 2);
			nearest_face.x = faces[0].x + faces[0].width*0.5;
			nearest_face.y = faces[0].y + faces[0].height*0.5;
    }else if(faces.size()>1){
    	for( size_t i = 0; i < faces.size(); i++ ){
    		rectangle(frame, faces[i], cv::Scalar(255,0,0), 2);
    		int distEucl = distEuclidean(	faces[i].x + faces[i].width*0.5,
    										lastPoint.x,
    										faces[i].y + faces[i].height*0.5,
    										lastPoint.y);
    		if(distEucl < min_dist){
    			nearest_face.x = faces[i].x;
    			nearest_face.y = faces[i].y;
    			min_dist = distEucl;
    		}
    	}
    }

    imshow( "test", frame);
    return nearest_face;
}

int distEuclidean(int x1,int y1,int x2,int y2){
	return sqrt(pow(x2-x1,2)+pow(y2-y1,2));
}



