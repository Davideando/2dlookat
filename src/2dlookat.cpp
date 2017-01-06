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
#include <cmath>

// Defines to control the source of the images

// If Camera is active, the image source is the cámera

//#define Camera

// If the Video is active, the image source is the video file
#define Video

int main(int argc, char *argv[]) 
{
	// All the variables

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

	// Variable to store the last value of the image
    cv::Point last_Center(320,240); // Image size = 640 x 480

    // Variable to store the Rect value
    cv::Rect detectedFace;

    // bool to control the detection
    bool found = false;

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
										5,			// min Neighbors
										0 | cv::CASCADE_SCALE_IMAGE,			// Flags
										cv::Size(1,1));	// Min Size

		// Number of faces detector
		int faceSize = faces.size();

		switch(faceSize)
		{
			case 0:
				// TODO: KALMAN DETECTION!!!
				// There is no face detected
				std::cout << "Face lost!!! \n";
				// Update the state
				found = false;
				// Restore the center position
				last_Center = cv::Point(320,240);
				break;
			case 1:
				// It's only one face
				if (found) // If 
				{
					// If there is a face detected before

					// Get the center of the face detected
					cv::Point face_center;
					face_center.x = faces[0].x + (faces[0].width/2);
					face_center.y = faces[0].y + (faces[0].height/2);
					// Check if this detection is correct or a false detection
					// A correct detection has a difference of +/- 5 pixels max, else the value is discarded
					if((abs(face_center.x - last_Center.x) < 6) and (abs(face_center.y - last_Center.y) < 6))
					{
						std::cout << "Is in marge!!\n";
						// Update the value of the center of the image
						last_Center = face_center;
						// Update the print rectangle
						detectedFace = faces[0];
					}
					else
					{
						// If the face detected is incorrect
						found = false;
					}
					// Debug

				}
				else
				{
					// If there is the first detection
					// Get the center of the face detected
					cv::Point face_center;
					face_center.x = faces[0].x + (faces[0].width/2);
					face_center.y = faces[0].y + (faces[0].height/2);
					// Update the value of the center of the image
					last_Center = face_center;
					// Update the print rectangle
					detectedFace = faces[0];
					// Update the status as detected
					found = true;
				}
				break;
			default:
				// There is more than 1 face detected
				if(found)
				{
					// If there is a face detected before
					std::cout << "Entro!!\n";
					// Variable to check the correct detection
					bool correctDetect = false;

					// Check in all the positions and get the close position to the last detection
					for(int i = 0; i < faceSize; i++)
					{
						cv::Point face_center;
						face_center.x = faces[i].x + (faces[i].width/2);
						face_center.y = faces[i].y + (faces[i].height/2);
						// Check if this detection is correct or a false detection
						// A correct detection has a difference of +/- 5 pixels max, else the value is discarded
						if((abs(face_center.x - last_Center.x) < 6) and (abs(face_center.y - last_Center.y) < 6))
						{
							std::cout << "Is in marge with more than 1 faces!!\n";
							// Update the value of the center of the image
							last_Center = face_center;
							correctDetect = true;
							// Update the print rectangle
							detectedFace = faces[i];
							// exit loop
							break;
						}
					}
					if(correctDetect)
					{
						std::cout << "He detectado algo?\n";
					}
					else
					{
						std::cout << "O no?\n";
						found = false;
					}

				}
				else
				{
					// If there is the first detection

					//  Detect position closest to the center	
					int lastDistance = 1500; // Init with a big value

					// Check in all the positions and get the closest position to the center
					for(int i = 0; i < faceSize; i++)
					{
						// Get the center position of the detection
						cv::Point face_center;
						face_center.x = faces[i].x + (faces[i].width/2);
						face_center.y = faces[i].y + (faces[i].height/2);

						// Get the absolute distance to the center of the image
						int temp = abs(face_center.x - last_Center.x) + abs(face_center.y - last_Center.y);

						// Update the value if the distance is less than the last one detected
						if(lastDistance > temp)
						{
							// Update the distance
							lastDistance = temp;
							// Get the correct value
							last_Center = face_center;
							// Update the print rectangle
							detectedFace = faces[i];
						}
					}
					found = true;
				}
				break;

		}

		// The rect is printed if something is found
		if(found)
		{
			// Print a rectangle 
			cv::rectangle(	image, 				// Destination image
							detectedFace, 		// face rectangle
							CV_RGB(0,255,0), 	// Color
							2);
		}

		// Show the image
		cv::namedWindow( "Face Detector", cv::WINDOW_AUTOSIZE );// Create a window for display.
		cv::imshow( "Face Detector", image );                   // Show our image inside it.

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
