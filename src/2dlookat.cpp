/*
*		Face Detector
*			Program to detect faces
*			By David Ortiz Martínez
*
*/

// OpenCV Libraries
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//std../../
#include <iostream>
#include <cstdlib>

// Defines to control the source of the images

// If Camera is active, the image source is the cámera

//#define Camera

// If the Video is active, the image source is the video file
#define Video

int main(int argc, char *argv[]) 
{
	// OpenCV video capture object
    cv::VideoCapture capture;
	
	// OpenCV image object
    cv::Mat image;

    // Gray scale image
    cv::Mat gray_image;

	
	// Camera id . Associated to device number in /dev/videoX
	int cam_id; 

	// Variable to store the face clasifier
	cv::CascadeClassifier face_detect;

	// Variable to store the detected faces
	std::vector<cv::Rect> faces;

	//check user args
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
	
	// Advertising to the user 
	std::cout << "Opening video file " << cam_id << std::endl;

	
	#if defined Video
		// Init the video file
		std::cout << "The input source is the Video File\n";

		// Loading the video
		capture = cv::VideoCapture("../img/video.mpeg");

		// Verify if the video is opened
		if(!capture.isOpened())                              // Check for invalid video
	    {
	        std::cout <<  "Could not open or find the video" << std::endl;
	        return -1;
	    }
	#elif defined Camera
		// Inicializar la cámara
		std::cout << "The input source is the camera\n";
	    // Open the video stream and make sure it's opened
    	if( !capture.open(cam_id) ) 
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
    if(!face_detect.load("../haarcascade_frontalface_default.xml"))
    {
    	std::cout <<  "Could not open of find the XML file" << std::endl;
    	return -1;
    }

    // Program Loop
    while(1)
    {
	    // Get the next image
	  	capture.read(image);

  		// Verify if the video is finished or webcam is closed
	  	if(!image.data)
	  	{
	  		std::cout << "The video is finished!! Bye" << std::endl;
	  		break;
	  	}


		// Convert the image to Gray scale
		cv::cvtColor(image, gray_image, CV_BGR2GRAY);

		// Detect the faces as rectangles
		face_detect.detectMultiScale(	gray_image, // Image
										faces, 		// Faces location
										1.3, 		// Scale factor
										4,			// min Neighbors
										0 | cv::CASCADE_SCALE_IMAGE,			// Flags
										cv::Size(30,30));	// Min Size

		// Print all the detected faces
		for(int i = 0; i < faces.size(); i++)
		{
			cv::Rect temp = faces[i];
			// Print a rectangle 
			cv::rectangle(	image, 			// Destination image
							temp, 			// face rectangle
							CV_RGB(0,255,0), 	// Color
							2); 				// thickness	 
		}

	    cv::namedWindow( "Face Detector", cv::WINDOW_AUTOSIZE );// Create a window for display.
	    cv::imshow( "Face Detector", image );                   // Show our image inside it.

	    // Test to check if we detect more than 2 faces
	    if(faces.size() > 1)
	    {
	    	std::cout << "Se han detectado " << faces.size() << " caras.\n";
	    }
	    // If the 'q' is pressed, exit the loop
	    if(cv::waitKey(1) == 'q')
	    {
	    	// Exit the loop
	    	break;
	    }
	}

	// The end of the program
	return 0; 
}
