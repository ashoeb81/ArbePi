//============================================================================
// Name        : main.cpp
// Author      : Nati Dar
// Version     :
// Copyright   : Arbe Robotics
// Description : Arbe Robotics API C++
//============================================================================

#include "radar_interface.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <ctime>

using namespace cv;
using namespace std;

int main() {
    uart_init();
    AR_RADAR->SetTransmitBufferHandler(&TransmitUartBuffer);
    AR_RADAR->SetTargetsMessageReceivedHandler(&TargetsMessageReceived);
    AR_RADAR->SetStatusUpdatedHandler(&RadarStatusUpdated);
    AR_RADAR->SetErrorHandler(&RadarError);

    if (AR_RADAR->Connect(10000000, 50)) {
        cout << "Radar Connected!" << endl;
        if (AR_RADAR->ConfigureRadar(RadarConfigurations::DEFAULT_CONFIG_FRONT_SECTOR)) {
            cout << "Radar Configured!" << endl;
            AR_RADAR->StartRadar(RadarConfigurations::DEFAULT_CONFIG_FRONT_SECTOR);
        }
    }
    VideoCapture capture(0);
    double dWidth = capture.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
    double dHeight = capture.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video
    Size frameSize(static_cast<int>(dWidth), static_cast<int>(dHeight));

    time_t now = time(0);
    char* dt = ctime(&now);
    string timestamp(dt);
    string video_fname = "/media/usb/my_video_"  +  timestamp + ".avi";
    replace(video_fname.begin(), video_fname.end(), ' ', '_');
    replace(video_fname.begin(), video_fname.end(), ':', '_');
    video_fname.erase(remove(video_fname.begin(), video_fname.end(), '\n'), video_fname.end());
    VideoWriter oVideoWriter(video_fname, CV_FOURCC('M','J','P','G'), 20, frameSize, true);
    cout << "Starting Video Capture Process: " << video_fname << endl;

    if (!capture.isOpened()) {
      cout << "Failed to open capture device" << endl;
      return -1;
    }

    if ( !oVideoWriter.isOpened() ) //if not initialize the VideoWriter successfully, exit the program
    {
         cout << "ERROR: Failed to write the video" << endl;
         return -1;
    }

    while (1)
    {
        get_uart_data();
        Mat frame;

        bool bSuccess = capture.read(frame); // read a new frame from video
        if (!bSuccess) //if not success, break loop
       {
             cout << "ERROR: Cannot read a frame from video file" << endl;
             break;
        }
        time_t now = time(0);
        char* dt = ctime(&now);
        string timestamp(dt);
        putText(frame, timestamp, Point(30,30) , FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 0), 2);
        oVideoWriter.write(frame); //writer the frame into the file
        if (waitKey(50) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
       {
            cout << "esc key is pressed by user" << endl;
            break; 
       }
    }
}
