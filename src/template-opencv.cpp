/*
 * Copyright (C) 2020  Christian Berger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// Include the single-file, header-only middleware libcluon to create high-performance microservices
#include "cluon-complete.hpp"

#include "cluon-complete.cpp"
// Include the OpenDLV Standard Message Set that contains messages that are usually exchanged for automotive or robotic applications 
#include "opendlv-standard-message-set.hpp"

// Include the GUI and image processing header files from OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Include the string stream library to use instead of buffer
#include <sstream> 

// HSV values for yellow cones
int YELLOW_MIN_HUE_VALUE = 0;
int YELLOW_MAX_HUE_VALUE = 42;
int YELLOW_MIN_SAT_VALUE = 75;
int YELLOW_MAX_SAT_VALUE = 221;
int YELLOW_MIN_VAL_VALUE = 170;
int YELLOW_MAX_VAL_VALUE = 255;

// HSV values for blue cones
int BLUE_MIN_HUE_VALUE = 102;
int BLUE_MAX_HUE_VALUE = 150;
int BLUE_MIN_SAT_VALUE = 88;
int BLUE_MAX_SAT_VALUE = 165;
int BLUE_MIN_VAL_VALUE = 43;
int BLUE_MAX_VAL_VALUE = 222;

int numberOfFrames = 0; // used to count starting frames
int maxFrames = 5;      // initial number of frames used to determine direction
int totalFrames = 0;
int withinRangeFrames = 0;

int identifiedShape = 60;     // pixel size used to determine cones
bool yellowConeFound = false; // flag to check if blue cones have been detected, make it a bool

// Variables for steering angle calculation
float steeringWheelAngle = 0.0;
float maxSteering = 0.3;
float minSteering = -0.3;
int carDirection = -1; // left car direction is negative (counterclockwise), default value

// Variables for turning
float turnRight = 0.025;
float turnLeft = -0.025;

// Vectors used for storing cone contours
std::vector<std::vector<cv::Point>> contours;
std::vector<cv::Vec4i> hierarchy;

std::vector<double> calculatedAngles;
std::vector<double> actualAngles;
std::vector<double> timestamps;

// Define the region of interest by providing a rectabgular region with 4 parameter of x,y coordinates and width and height
cv::Rect rightRegionOfInterest = cv::Rect(415, 265, 150, 125);

// Define the region of interest by providing a rectabgular region with 4 parameter of x,y coordinates and width and height
cv::Rect centerRegionOfInterest = cv::Rect(200, 245, 230, 115);


int32_t main(int32_t argc, char **argv) {
    int32_t retCode{1};
    // Parse the command line parameters as we require the user to specify some mandatory information on startup.
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ( (0 == commandlineArguments.count("cid")) ||
         (0 == commandlineArguments.count("name")) ||
         (0 == commandlineArguments.count("width")) ||
         (0 == commandlineArguments.count("height")) ) {
        std::cerr << argv[0] << " attaches to a shared memory area containing an ARGB image." << std::endl;
        std::cerr << "Usage:   " << argv[0] << " --cid=<OD4 session> --name=<name of shared memory area> [--verbose]" << std::endl;
        std::cerr << "         --cid:    CID of the OD4Session to send and receive messages" << std::endl;
        std::cerr << "         --name:   name of the shared memory area to attach" << std::endl;
        std::cerr << "         --width:  width of the frame" << std::endl;
        std::cerr << "         --height: height of the frame" << std::endl;
        std::cerr << "Example: " << argv[0] << " --cid=253 --name=img --width=640 --height=480 --verbose" << std::endl;
    }
    else {
        // Extract the values from the command line parameters
        const std::string NAME{commandlineArguments["name"]};
        const uint32_t WIDTH{static_cast<uint32_t>(std::stoi(commandlineArguments["width"]))};
        const uint32_t HEIGHT{static_cast<uint32_t>(std::stoi(commandlineArguments["height"]))};
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};

        // Attach to the shared memory.
        std::unique_ptr<cluon::SharedMemory> sharedMemory{new cluon::SharedMemory{NAME}};
        if (sharedMemory && sharedMemory->valid()) {
            std::clog << argv[0] << ": Attached to shared memory '" << sharedMemory->name() << " (" << sharedMemory->size() << " bytes)." << std::endl;

            // Interface to a running OpenDaVINCI session where network messages are exchanged.
            // The instance od4 allows you to send and receive messages.
            cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

            opendlv::proxy::GroundSteeringRequest gsr;
            std::mutex gsrMutex;
            auto onGroundSteeringRequest = [&gsr, &gsrMutex](cluon::data::Envelope &&env){
                // The envelope data structure provide further details, such as sampleTimePoint as shown in this test case:
                // https://github.com/chrberger/libcluon/blob/master/libcluon/testsuites/TestEnvelopeConverter.cpp#L31-L40
                std::lock_guard<std::mutex> lck(gsrMutex);
                gsr = cluon::extractMessage<opendlv::proxy::GroundSteeringRequest>(std::move(env));
                std::cout << "lambda: groundSteering = " << gsr.groundSteering() << std::endl;
            };

            od4.dataTrigger(opendlv::proxy::GroundSteeringRequest::ID(), onGroundSteeringRequest);

            // Endless loop; end the program by pressing Ctrl-C.
            while (od4.isRunning()) {

        // Increase the frameCounter variable to get our sample frames for carDirection
        numberOfFrames++;

        // OpenCV data structure to hold an image.
        cv::Mat img;
        cv::Mat croppedImg;

        // Wait for a notification of a new frame.
        sharedMemory->wait();

        // Lock the shared memory.
        sharedMemory->lock();{
          // Copy the pixels from the shared memory into our own data structure.
          cv::Mat wrapped(HEIGHT, WIDTH, CV_8UC4, sharedMemory->data());
          img = wrapped.clone();
          croppedImg = img(cv::Rect(0, 0, img.cols, img.rows - 80));
          }

        std::pair<bool, cluon::data::TimeStamp> sTime = sharedMemory->getTimeStamp(); // Saving current time in sTime var

        // Convert TimeStamp obj into microseconds
        uint64_t sMicro = cluon::time::toMicroseconds(sTime.second);

        // Shared memory is unlocked
        sharedMemory->unlock();

        // Define the images we need for yellow cones on the right
        cv::Mat yellowHsvImage;
        cv::Mat detectYellowImg;

        // Define the images we need for center cones detecetion
        cv::Mat hsvCenterImg;
        cv::Mat detectCenterImg;

    // loop runs until frame counter is greater than the sample size of 5, used to determine direction (counterclockwise, clockwise etc...)
        if (numberOfFrames < maxFrames){

          // Capture the image of the region of interest we defined before
          cv::Mat yellowConeImage = img(rightRegionOfInterest);

          // converting the BRG colors of the image to HSV values for better image processing
          cv::cvtColor(yellowConeImage, yellowHsvImage, cv::COLOR_BGR2HSV);

          // Threshold the yellowHsvImage based on color ranges we defined and store the result in the rightImage as an input image
          cv::inRange(yellowHsvImage, cv::Scalar(YELLOW_MIN_HUE_VALUE, YELLOW_MIN_SAT_VALUE, YELLOW_MIN_VAL_VALUE), cv::Scalar(YELLOW_MAX_HUE_VALUE, YELLOW_MAX_SAT_VALUE, YELLOW_MAX_VAL_VALUE), detectYellowImg);

          // Applying Gaussian blur to detectRightImg
          cv::GaussianBlur(detectYellowImg, detectYellowImg, cv::Size(5, 5), 0);

          // fill in small gaps in the rightImage and expand the size of the objects in the image
          cv::dilate(detectYellowImg, detectYellowImg, 0);

          // Applying erosion to avoid the appearance of overlapping objects on the cones
          cv::erode(detectYellowImg, detectYellowImg, 0);

           // find the contours of the cones in rightImage and store in contours vector
          cv::findContours(detectYellowImg, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

          // Create a new image to store the detected yellow cones contours with the same size as rightImage
          cv::Mat rightYellowConesContourImg = cv::Mat::zeros(detectYellowImg.rows, detectYellowImg.cols, CV_8UC3);

          // Loops over the contours vector
          for (unsigned int i = 0; i < contours.size(); i++) {

            // If the current index of the vector has a contour area that is larger than the defined number of pixels in identifiedShape, we have a cone
            if (cv::contourArea(contours[i]) > identifiedShape) {
              // Draws the contour of the cone on the image
              cv::Scalar colour(255, 255, 0);
              cv::drawContours(rightYellowConesContourImg, contours, i, colour, -1, 8, hierarchy);
              
            }
          }

        }








            }
        }
        retCode = 0;
    }
    return retCode;
}