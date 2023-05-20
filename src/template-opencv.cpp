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

//string stream library
#include <sstream>
// Include the GUI and image processing header files from OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>



/* HSV values for yellow cones */
const cv::Scalar YELLOW_MIN(20, 100, 150);
const cv::Scalar YELLOW_MAX(40, 255, 255);

/* HSV values for blue cones */
const cv::Scalar BLUE_MIN(100, 50, 50);
const cv::Scalar BLUE_MAX(130, 150, 220);

/* Variables for steering angle calculation */
int carDir = -1;
const float MAX_STEERING = 0.3;
const float MIN_STEERING = -0.3;
const float TURN_RIGHT = 0.025;
const float TURN_LEFT = -0.025;
float steeringAngle = 0.0;

/* Define variables for processing yellow color and center detection  */
  cv::Mat yellowHsvImage;
  cv::Mat detectYellowImg;
  cv::Mat hsvCenterImg;
  cv::Mat detectCenterImg;

/* Define variables for frames */
int numberOfFrames = 0; 
int maxFrames = 5; 
int totalFrames = 0;
int withinRangeFrames = 0;

 // pixel size to determine cones
int coneShape = 60; 
bool yellowConeFound = false; 
bool equilibrium = false;

/* Vectors used for storing cone contours */
std::vector<std::vector<cv::Point>> contours;
std::vector<cv::Vec4i> hierarchy;

std::vector<double> calculatedAngles;
std::vector<double> actualAngles;
std::vector<double> timestamps;

/* Define the region of interest by providing a rectangular region with 4 parameters: x, y coordinates, width, and height */
cv::Rect rightRegionOfInterest = cv::Rect(505, 265, 50, 125);
cv::Rect centerRegionOfInterest = cv::Rect(300, 275, 100, 90);


int32_t main(int32_t argc, char **argv) {
  int32_t retCode = 1;

  /* Parse the command line parameters as we require the user to specify some mandatory information on startup */
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if ((0 == commandlineArguments.count("cid")) ||
      (0 == commandlineArguments.count("name")) ||
      (0 == commandlineArguments.count("width")) ||
      (0 == commandlineArguments.count("height"))) {
    std::cerr << argv[0] << " attaches to a shared memory area containing an ARGB image." << std::endl;
    std::cerr << "Usage:   " << argv[0] << " --cid=<OD4 session> --name=<name of shared memory area> [--verbose]" << std::endl;
    std::cerr << "         --cid:    CID of the OD4Session to send and receive messages" << std::endl;
    std::cerr << "         --name:   name of the shared memory area to attach" << std::endl;
    std::cerr << "         --width:  width of the frame" << std::endl;
    std::cerr << "         --height: height of the frame" << std::endl;
    std::cerr << "Example: " << argv[0] << " --cid=253 --name=img --width=640 --height=480 --verbose" << std::endl;
  } 
  else 
  {
    /* Extract the values from the command line parameters */
    const std::string NAME = commandlineArguments["name"];
    const uint32_t WIDTH = static_cast<uint32_t>(std::stoi(commandlineArguments["width"]));
    const uint32_t HEIGHT = static_cast<uint32_t>(std::stoi(commandlineArguments["height"]));
    const bool VERBOSE = commandlineArguments.count("verbose") != 0;


    // Attach to the shared memory.
    std::unique_ptr<cluon::SharedMemory> sharedMemory{
        new cluon::SharedMemory{
            NAME}};
    if (sharedMemory && sharedMemory->valid())
    {
      std::clog << argv[0] << ": Attached to shared memory '" << sharedMemory->name() << " (" << sharedMemory->size() << " bytes)." << std::endl;

      // Interface to a running OpenDaVINCI session where network messages are exchanged.
      // The instance od4 allows you to send and receive messages.
      cluon::OD4Session od4{
          static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

      opendlv::proxy::GroundSteeringRequest gsr;
      std::mutex gsrMutex;
      auto onGroundSteeringRequest = [&gsr, &gsrMutex](cluon::data::Envelope &&env)
      {
        // The envelope data structure provide further details, such as sampleTimePoint as shown in this test case:
        // https://github.com/chrberger/libcluon/blob/master/libcluon/testsuites/TestEnvelopeConverter.cpp#L31-L40
        std::lock_guard<std::mutex> lck(gsrMutex);
        gsr = cluon::extractMessage<opendlv::proxy::GroundSteeringRequest>(std::move(env));
      };


      od4.dataTrigger(opendlv::proxy::GroundSteeringRequest::ID(), onGroundSteeringRequest);

      // Endless loop; end the program by pressing Ctrl-C.
      while (od4.isRunning())
      {

        // Increase the frameCounter variable to get our sample frames for carDirection
        numberOfFrames++;

        // OpenCV data structure to hold an image.
        cv::Mat img;
        cv::Mat croppedImg;

        // Wait for a notification of a new frame.
        sharedMemory->wait();

        // Lock the shared memory.
        sharedMemory->lock();
        {
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


        if (numberOfFrames < maxFrames)
        {

          cv::Mat yellowConeImage = img(rightRegionOfInterest);

          cv::cvtColor(yellowConeImage, yellowHsvImage, cv::COLOR_BGR2HSV);

          cv::inRange(yellowHsvImage, YELLOW_MIN, YELLOW_MAX, detectYellowImg);

          cv::GaussianBlur(detectYellowImg, detectYellowImg, cv::Size(5, 5), 0);

          cv::dilate(detectYellowImg, detectYellowImg, 0);
          cv::erode(detectYellowImg, detectYellowImg, 0);

          cv::findContours(detectYellowImg, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
          cv::Mat rightYellowConesContourImg = cv::Mat::zeros(detectYellowImg.rows, detectYellowImg.cols, CV_8UC3);


          for (unsigned int i = 0; i < contours.size(); i++)
          {
            if (cv::contourArea(contours[i]) > coneShape)
            {

              // Draws the contour of the cone on the image
              cv::Scalar colour(255, 255, 0);
              cv::drawContours(rightYellowConesContourImg, contours, i, colour, -1, 8, hierarchy);

              yellowConeFound = true;

              if (yellowConeFound == true)
              {
                carDir = 1;
              }
            } 
          }
        }
      
        // If the number of frames is larger than or equal to the fixed frame size
        if (numberOfFrames >= maxFrames)
        {

          cv::Mat centreImg = img(centerRegionOfInterest);

          cv::cvtColor(centreImg, hsvCenterImg, cv::COLOR_BGR2HSV);
          
          cv::inRange(hsvCenterImg, BLUE_MIN, BLUE_MAX, detectCenterImg);

          cv::GaussianBlur(detectCenterImg, detectCenterImg, cv::Size(5, 5), 0);

          cv::dilate(detectCenterImg, detectCenterImg, 0);
          cv::erode(detectCenterImg, detectCenterImg, 0);

          cv::findContours(detectCenterImg, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

          cv::Mat blueContourImg = cv::Mat::zeros(detectCenterImg.rows, detectCenterImg.cols, CV_8UC3);

          bool blueConeCenter = false;

  
          for (unsigned int i = 0; i < contours.size(); i++)
          {
            if (cv::contourArea(contours[i]) > coneShape)
            {
              cv::Scalar colour(255, 255, 0);
              cv::drawContours(blueContourImg, contours, i, colour, -1, 8, hierarchy);

              if (steeringAngle > MIN_STEERING && steeringAngle < MAX_STEERING)
              {

                blueConeCenter = true; 
                if (carDir == 1)
                {                                 
                  steeringAngle -= TURN_RIGHT;
 
                }
                else if (carDir == -1)
                {                                
                  steeringAngle -= TURN_LEFT;
          
                }
              }
            }
          }

          if (VERBOSE)
          {
            cv::imshow("Blue Contours", blueContourImg);
            cv::waitKey(1);
          }

          if (blueConeCenter == false)
          {

            cv::cvtColor(centreImg, hsvCenterImg, cv::COLOR_BGR2HSV);
    
            cv::inRange(hsvCenterImg, YELLOW_MIN, YELLOW_MAX, detectCenterImg);

            cv::GaussianBlur(detectCenterImg, detectCenterImg, cv::Size(5, 5), 0);

            cv::dilate(detectCenterImg, detectCenterImg, 0);
            cv::erode(detectCenterImg, detectCenterImg, 0);
      
            cv::findContours(detectCenterImg, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

            cv::Mat yellowContourImg = cv::Mat::zeros(detectCenterImg.rows, detectCenterImg.cols, CV_8UC3);

            bool yellowConeCenter = false;

            for (unsigned int i = 0; i < contours.size(); i++)
            {
              if (cv::contourArea(contours[i]) > coneShape)
              {
                cv::Scalar colour(255, 255, 0);
                cv::drawContours(yellowContourImg, contours, i, colour, -1, 8, hierarchy);

                // when current steeringAngle is less than MAX_STEERING and more than MIN_STEERING
                if (steeringAngle > MIN_STEERING && steeringAngle < MAX_STEERING)
                {
                  yellowConeCenter = true;
                  if (carDir == 1)
                  {                                
                    steeringAngle -= TURN_LEFT;
               
                  }
                  else if (carDir == -1)
                  {
                    steeringAngle -= TURN_RIGHT; 
                  }

                }
              }
            }
            if (VERBOSE)
            {
              cv::imshow("Yellow Contours", yellowContourImg);
              cv::waitKey(1);
            }

            if (yellowConeCenter == false && blueConeCenter == false)
            {
              steeringAngle = 0.00;
            }
          }
        }

        cv::Mat hsvImg;
        img.copyTo(hsvImg);
        cv::cvtColor(hsvImg, hsvImg, cv::COLOR_BGR2HSV);


        //  ------create string stream input, put values in, to be used in printing---------

        std::ostringstream calcGroundSteering;
        std::ostringstream actualSteering;
        std::ostringstream timestamp;

        calcGroundSteering << steeringAngle;
        actualSteering << gsr.groundSteering();
        timestamp << sMicro;

        std::string time = " Time Stamp: ";
        std::string calculatedGroundSteering = "Calculated Ground Steering: ";
        std::string actualGroundSteering = " Actual Ground Steering: ";
        std::string groundSteeringAngle = std::to_string(steeringAngle);

        calculatedGroundSteering.append(groundSteeringAngle);
        calculatedGroundSteering.append(calcGroundSteering.str());
        actualGroundSteering.append(actualSteering.str());
        time.append(timestamp.str());



        // --------------   Checking performance  --------------------

        double allowedDeviation;
        if (std::abs(steeringAngle) <= 0.01)
        {
          allowedDeviation = 0.05;
        }
        else
        {
          allowedDeviation = 0.3 * std::abs(steeringAngle);
        }

        double actualAngle = gsr.groundSteering();
        if (std::abs(actualAngle - steeringAngle) <= allowedDeviation)
          withinRangeFrames++;

        totalFrames++;


        // -----------  Displaying performance info  ------------


        std::string percentMsg = "Performance: ";
        double percent = (double)withinRangeFrames / (double)totalFrames * 100;
        if (percent >= 40)
        {
          percentMsg += std::to_string(percent) + "%";
          cv::putText(img, percentMsg, cv::Point(80, 140), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 250, 154), 1);
        }
        else
        {
          percentMsg += std::to_string(percent) + "% (Insufficient frames within range)";
          cv::putText(img, percentMsg, cv::Point(80, 140), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(255, 0, 0), 1);
        }


        // ----------  Displays information on video  -----------

        cv::putText(img, calculatedGroundSteering, cv::Point(80, 50), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 250, 154), 1);
        cv::putText(img, actualGroundSteering, cv::Point(80, 80), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 250, 154), 1);
        cv::putText(img, time, cv::Point(80, 110), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 250, 154), 1);

        {
          std::lock_guard<std::mutex> lck(gsrMutex);
          //  std::cout << "group_09;" << sMicro << ";" << steeringAngle << std::endl;
          std::cout << "group_09;" << sMicro << ";" << steeringAngle << ";" << gsr.groundSteering() << std::endl;
          //   std::cout << "group_09;"  << "\t" << sMicro << "\t" << steeringAngle << "\t" << gsr.groundSteering() << std::endl;
        }

        cv::Rect combinedRegionOfInterest(
            std::min(rightRegionOfInterest.x, centerRegionOfInterest.x),
            std::min(rightRegionOfInterest.y, centerRegionOfInterest.y),
            std::max(rightRegionOfInterest.x + rightRegionOfInterest.width, centerRegionOfInterest.x + centerRegionOfInterest.width) - std::min(rightRegionOfInterest.x, centerRegionOfInterest.x),
            std::max(rightRegionOfInterest.y + rightRegionOfInterest.height, centerRegionOfInterest.y + centerRegionOfInterest.height) - std::min(rightRegionOfInterest.y, centerRegionOfInterest.y));

        cv::Mat overlay = img.clone();
        cv::Rect color = cv::Rect(combinedRegionOfInterest.x, combinedRegionOfInterest.y, combinedRegionOfInterest.width, combinedRegionOfInterest.height);
        cv::rectangle(overlay, color, cv::Scalar(0, 0, 255, 128), -1);
        double alpha = 0.5;
        cv::addWeighted(overlay, alpha, img, 1 - alpha, 0, img);

        // cv::Rect greenColor = cv::Rect(actualCenterRegionOfInterest.x, actualCenterRegionOfInterest.y, actualCenterRegionOfInterest.width, actualCenterRegionOfInterest.height);
        // cv::rectangle(overlay, greenColor, cv::Scalar(0, 255, 0, 128), -1); // Change the Scalar to (0, 255, 0, 128) for green
        // cv::addWeighted(overlay, alpha, img, 1 - alpha, 0, img);

        // Displays debug window on screen
        if (VERBOSE)
        {
          cv::imshow("Main", img);
          cv::waitKey(1);
        }
      }
    }
    retCode = 0;
  }
  return retCode;
}
