#include "radar_interface.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <ctime>

using namespace cv;
using namespace std;

// Video frame width and height
const int kFrameWidth = 320;
const int kFrameHeight = 240;

/* Generate timestamp from current time.
 * @returns string of current time and date.
 */
string getTimestamp() {
    time_t now = time(0);
    char* dt = ctime(&now);
    string timestamp(dt);
    replace((*timestamp).begin(), (*timestamp).end(), '?', '');
    return timestamp;
}

/* Generate names for radar log and video files
 * @param pointer to string that will hold radar log file name.
 * @param pointer to string that will hold video file name.
 */
void getFileNames(string* log_fname, string* video_fname) {
    // Get timestamp to add to file names.
    string timestamp = getTimestamp();

    *video_fname = "/media/usb/video_"  +  timestamp + ".avi";
    replace((*video_fname).begin(), (*video_fname).end(), ' ', '_');
    replace((*video_fname).begin(), (*video_fname).end(), ':', '_');

    *log_fname = "/media/usb/log_"  +  timestamp + ".txt";
    replace((*log_fname).begin(), (*log_fname).end(), ' ', '_');
    replace((*log_fname).begin(), (*log_fname).end(), ':', '_');
}

/* Given a video frame add current data and time as text overlay.*/
void addTimestampToFrame(Mat* video_frame) {
    // Get timestamp to add to frame.
    string timestamp = getTimestamp();
    putText(*video_frame, timestamp, Point(5,30) , FONT_HERSHEY_SIMPLEX, 0.60, Scalar(255, 0, 0), 1);
    // Add Number of targets detected to frame
    ostringstream oss;
    oss << "Detected: " << AR_RADAR->num_targets;
    putText(*video_frame, oss.str(), Point(5, 50), FONT_HERSHEY_SIMPLEX, 0.60, Scalar(0, 0, 255), 1);
}

int main() {
    // Generate names of log files.
    string log_fname, video_fname;
    getFileNames(&log_fname, &video_fname);

    // Setup serial port for communication with Arbe radar.
    uart_init(log_fname);

    // Setup callbacks that communicate with Arbe radar.
    AR_RADAR->SetTransmitBufferHandler(&TransmitUartBuffer);
    AR_RADAR->SetTargetsMessageReceivedHandler(&TargetsMessageReceived);
    AR_RADAR->SetStatusUpdatedHandler(&RadarStatusUpdated);
    AR_RADAR->SetErrorHandler(&RadarError);

    // Attempt to connect to the Arbe radar.
    if (AR_RADAR->Connect(10000000, 50)) {
        cout << "Connected to Radar." << endl;
        if (AR_RADAR->ConfigureRadar(RadarConfigurations::HIGH_CONFIDANCE_CONFIG_FRONT_SECTOR)) {
            cout << "Configured Radar." << endl;
            AR_RADAR->StartRadar(RadarConfigurations::HIGH_CONFIDANCE_CONFIG_FRONT_SECTOR);
        } else {
            cout << "Failed to Configure Radar and Exiting." << endl;
            return -1;
        }
    } else {
        cout << "Failed to Connect to Radar and Exiting." << endl;
    }

    // Setup USB camera video capture.
    VideoCapture capture(0);
    Size frameSize(kFrameWidth, kFrameHeight);

    // Add window for video frame display
    namedWindow("Video Frame", WINDOW_AUTOSIZE);

    // Setup video file writer.
    VideoWriter oVideoWriter(video_fname, CV_FOURCC('M','J','P','G'), 20, frameSize, true);


    if (!oVideoWriter.isOpened() || !capture.isOpened())
    {
         cout << "Failed to setup video capture and writer." << endl;
         // Shutdown the radar
         AR_RADAR->StopRadar();
         return -1;
    }

    while (1)
    {
        // Check for any messages from the Arbe radar.
        get_uart_data();

        // Grab frame from the camera.
        Mat frame;
        if (!capture.read(frame))
       {
             cout << "ERROR: Could not read a frame from camera" << endl;
             break;
        }

        // Add a timestamp and number of detected targets to frame.
        resize(frame, frame, Size(kFrameWidth, kFrameHeight));
        addTimestampToFrame(&frame);
 
        // Display video frame 
        imshow("Video Frame", frame);

        // Save video frame
        oVideoWriter.write(frame);

        //
        if (waitKey(50) == 27) //wait for 'esc' key press for 50ms.
       {
            cout << "Exiting" << endl;
            break; 
       }
    }

    // Shutdown the radar
    AR_RADAR->StopRadar();
}
