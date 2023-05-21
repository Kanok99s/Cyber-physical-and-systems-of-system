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
// Include the string stream library
#include <sstream>


/* HSV values for yellow cones */
const cv::Scalar YELLOW_MIN(20, 80, 150);
const cv::Scalar YELLOW_MAX(25, 190, 255);

/* HSV values for blue cones */
const cv::Scalar BLUE_MIN(95, 110, 50);
const cv::Scalar BLUE_MAX(150, 245, 255);

/* Define variables for frames */
int numberOfFrames = 0;
int maxFrames = 5;     
int totalFrames = 0;
int withinRangeFrames = 0;

/* Variables for steering angle calculation */
int carDirection = -1; 
double alpha = 0.5;
float turnRight = 0.045;
float turnLeft = -0.045;
float steeringWheelAngle = 0.0;

 // pixel size to determine cones
int coneShape = 60;
bool yellowConeFound = false;
int blueConeCenter = 0;
int yellowConeCenter = 0;

/* Define variables for processing yellow color and center detection  */
cv::Mat yellowHsvImage;
cv::Mat detectYellowImg;
cv::Mat hsvCenterImg;
cv::Mat detectCenterImg;

/* Define the region of interest by providing a rectangular region with 4 parameters: x, y coordinates, width, and height */
cv::Rect rightROI = cv::Rect(415, 265, 150, 125);
cv::Rect centerROI = cv::Rect(200, 245, 200, 115);

/*  
    this function applies Canny edge detection to an input image and searches for contours.
    It then identifies contours with an area greater than a threshold value, draws them on a separate image, and displays the result in a window. 
    The function returns true if a cone is found and false otherwise.

    ==============================STEPS====================================
    Declares two vectors: contours to store the contours found in the image and hierarchy to store the hierarchy of contours.
    Creates a new cv::Mat object named cannyImg to store the result of applying Canny edge detection to the input image using the provided threshold values.
    Uses the cv::findContours function to find contours in the cannyImg image.
    Checks if the area of the current contour (cv::contourArea(contours[i])) is greater than 60. If it is, it means that a potential cone has been found.
    If a potential cone is found, the contour is drawn. The contour is filled with a blue color represented by the colour scalar.
    Sets coneFound to true to indicate that a cone has been found.
    Displays the rightContourImg in a window with the name specified by windowName using the cv::imshow function.
    Returns the value of coneFound, indicating whether a cone was found in the image.
*/
bool isCone(int thresh1, int thresh2, const cv::Mat &img, const std::string& windowName)
{

  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;

  cv::Mat cannyImg;
  cv::Canny(img, cannyImg, thresh1, thresh2);

  cv::findContours(cannyImg, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
  cv::Mat rightContourImg = cv::Mat::zeros(cannyImg.rows, cannyImg.cols, CV_8UC3);

  bool coneFound = false;

  for (unsigned int i = 0; i < contours.size(); i++)
  {
    if (cv::contourArea(contours[i]) > 60)
    {

      cv::Scalar colour(255, 0, 0);
      cv::drawContours(rightContourImg, contours, i, colour, -1, 8, hierarchy);
      coneFound = true;
    }
  }

      cv::imshow(windowName, rightContourImg);
      cv::waitKey(1);
    
  return coneFound;
}

int32_t main(int32_t argc, char **argv)
{
  int32_t retCode{1};
  // Parse the command line parameters as we require the user to specify some mandatory information on startup.
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if ((0 == commandlineArguments.count("cid")) ||
      (0 == commandlineArguments.count("name")) ||
      (0 == commandlineArguments.count("width")) ||
      (0 == commandlineArguments.count("height")))
  {
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
    // Extract the values from the command line parameters
    const std::string NAME{
        commandlineArguments["name"]};
    const uint32_t WIDTH{
        static_cast<uint32_t>(std::stoi(commandlineArguments["width"]))};
    const uint32_t HEIGHT{
        static_cast<uint32_t>(std::stoi(commandlineArguments["height"]))};
    const bool VERBOSE{
        commandlineArguments.count("verbose") != 0};

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
        // std::cout << "lambda: groundSteering = " << gsr.groundSteering() << std::endl;
      };

      od4.dataTrigger(opendlv::proxy::GroundSteeringRequest::ID(), onGroundSteeringRequest);

      // Endless loop; end the program by pressing Ctrl-C.
      while (od4.isRunning())
      {
        
        // OpenCV data structure to hold an image.
        cv::Mat img;

        // Wait for a notification of a new frame.
        sharedMemory->wait();

        // Lock the shared memory.
        sharedMemory->lock();
        {
          // Copy the pixels from the shared memory into our own data structure.
          cv::Mat wrapped(HEIGHT, WIDTH, CV_8UC4, sharedMemory->data());
          img = wrapped.clone();
        }

        std::pair<bool, cluon::data::TimeStamp> sTime = sharedMemory->getTimeStamp(); // Saving current time in sTime var
        uint64_t sMicro = cluon::time::toMicroseconds(sTime.second);

        // Shared memory is unlocked
        sharedMemory->unlock();

        // --------------------------------------   Determine car direction  ------------------------------------------------------------
        // Capture the right side of the frame to search for yellow cones, 
        // if a yellow cone is found, it means the car is moving in a clockwise direction,
        // if not and there is a blue cone, the car is anti-clockwise.
        
        if (numberOfFrames < maxFrames)
        {

          int thresh1 = 50;
          int thresh2 = 150;
          std::string rightWindow = "Right Contour Image";
          bool coneFound = false;
          cv::Mat yellowConeImage = img(rightROI);
          // -----------------------------------   Yellow cones detection -----------------------------------------------------------------
          // Conversion of the yellow cone image from the BGR color space to the HSV color space
          // Color thresholding to define the lower and upper bounds of the yelow color range
          // Reduce noise with Gaussian Blur function
          // Dilation to improve and connect the yellow regions
          // Apply erosion to refine the yellow regions and eliminate any small noisy areas
          cv::cvtColor(yellowConeImage, yellowHsvImage, cv::COLOR_BGR2HSV);
          cv::inRange(yellowHsvImage, YELLOW_MIN, YELLOW_MAX, detectYellowImg);
          cv::GaussianBlur(detectYellowImg, detectYellowImg, cv::Size(5, 5), 0);
          cv::dilate(detectYellowImg, detectYellowImg, 0);
          cv::erode(detectYellowImg, detectYellowImg, 0);

          bool result = isCone(thresh1, thresh2, detectYellowImg, rightWindow);

          if (result)
          {
            carDirection = 1;
          }
        }

        if (numberOfFrames >= maxFrames)
        {

          cv::Mat centreImg = img(centerROI);
          // -----------------------------------   Center image detection targeting the blue color range -----------------------------------------------------------------
          // Conversion of the center image from the BGR color space to the HSV color space
          // Color thresholding to define the lower and upper bounds of the blue color range
          // Reduce noise with Gaussian Blur function
          // Dilation to improve and connect the blue regions
          // Apply erosion to refine the blue regions and eliminate any small noisy areas
          cv::cvtColor(centreImg, hsvCenterImg, cv::COLOR_BGR2HSV);
          cv::inRange(hsvCenterImg, BLUE_MIN, BLUE_MAX, detectCenterImg);
          cv::GaussianBlur(detectCenterImg, detectCenterImg, cv::Size(5, 5), 0);
          cv::dilate(detectCenterImg, detectCenterImg, 0);
          cv::erode(detectCenterImg, detectCenterImg, 0);

          int thresh1 = 50;
          int thresh2 = 150;
          bool coneFound = false;
          std::string blueWindow = "Blue Center Image";

          bool result = isCone(thresh1, thresh2, detectCenterImg, blueWindow );

          if (result)
          {
            blueConeCenter = 1;

            if (blueConeCenter == 1)
            {
              
                if (carDirection == 1)
                {
                  steeringWheelAngle -= turnRight;
                }
                else if (carDirection == -1)
                {
                  steeringWheelAngle -= turnLeft;
                }
              
            }
          }
          else
          {
            blueConeCenter = 0;
          }

          if (blueConeCenter == 0)
          {
          // -----------------------------------   Center image detection targeting the yellow color range -----------------------------------------------------------------
          // Conversion of the center image from the BGR color space to the HSV color space
          // Color thresholding to define the lower and upper bounds of the yellow color range
          // Reduce noise with Gaussian Blur function
          // Dilation to improve and connect the yellow regions
          // Apply erosion to refine the yellow regions and eliminate any small noisy areas
            cv::cvtColor(centreImg, hsvCenterImg, cv::COLOR_BGR2HSV);
            cv::inRange(hsvCenterImg, YELLOW_MIN, YELLOW_MAX , detectCenterImg);
            cv::GaussianBlur(detectCenterImg, detectCenterImg, cv::Size(5, 5), 0);
            cv::dilate(detectCenterImg, detectCenterImg, 0);
            cv::erode(detectCenterImg, detectCenterImg, 0);

            int thresh1 = 50;
            int thresh2 = 150;
            bool coneFound = false;

            std::string yellowWindow = "Yellow Center Image";

          bool result = isCone(thresh1, thresh2, detectCenterImg, yellowWindow );

            if (result)
            {
              yellowConeCenter = 1;

              if (yellowConeCenter == 1)
              {
               
                  if (carDirection == 1)
                  {
                    steeringWheelAngle -= turnLeft;
                  }
                  else if (carDirection == -1)
                  {
                    steeringWheelAngle -= turnRight;
                  }
               
              }
            }
            else
            {
              yellowConeCenter = 0;
            }

            if (yellowConeCenter == 0 && blueConeCenter == 0)
            {
              steeringWheelAngle = 0.00;
            }
          }
        }

        numberOfFrames++;


/* --------------------------- Creating and formatting strings for printing ---------------------------- 
Creates string stream input to be used in printing, 
puts values in the string stream,
creates the string variables,
and append values to the string variables */
        std::ostringstream calcGroundSteering;
        std::ostringstream actualSteering;
        std::ostringstream timestamp;

        calcGroundSteering << steeringWheelAngle;
        actualSteering << gsr.groundSteering();
        timestamp << sMicro;

        std::string time = " Time Stamp: ";
        std::string calculatedGroundSteering = "Calculated Ground Steering: ";
        std::string actualGroundSteering = " Actual Ground Steering: ";
        std::string groundSteeringAngle = std::to_string(steeringWheelAngle);

        calculatedGroundSteering.append(groundSteeringAngle);
        calculatedGroundSteering.append(calcGroundSteering.str());
        actualGroundSteering.append(actualSteering.str());
        time.append(timestamp.str());


        /* ----------------------------------   Checking performance  ----------------------------------- 
       Checks performance deviation based on steering wheel angle value,
       If the absolute value of steeringWheelAngle is less than or equal to 0.01, the allowed deviation is set to 0.05,
       if steeringWheelAngle is greater than 0.01, the allowed deviation is 0.3(percentage of deviation allowed)
       and updates the number of frames within range and the total frames processed, respectively */

        double allowedDeviation;
        if (std::abs(steeringWheelAngle) <= 0.01)
        {
          allowedDeviation = 0.05;
        }
        else
        {
          allowedDeviation = 0.3 * std::abs(steeringWheelAngle);
        }

        double actualAngle = gsr.groundSteering();
        if (std::abs(actualAngle - steeringWheelAngle) <= allowedDeviation)
          withinRangeFrames++;

        totalFrames++;


         /*  -------------------------------  Displaying performance info  ------------------------------  
        calculates the percentage of frames within the desired range 
        and displays a performance message on the image based on the performance value.
        Color and message varies depending on the 40% threshold of frames and if met or not */

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



         /* -------------------------------  Display information on video  ---------------------------
         displays various information (calculated ground steering, actual ground steering, and timestamp) on the video image,
         and prints group number, and steering wheel angle */

        cv::putText(img, calculatedGroundSteering, cv::Point(80, 50), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 250, 154), 1);
        cv::putText(img, actualGroundSteering, cv::Point(80, 80), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 250, 154), 1);
        cv::putText(img, time, cv::Point(80, 110), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 250, 154), 1);

        {
          std::lock_guard<std::mutex> lck(gsrMutex);
          std::cout << "group_09;" << sMicro << ";" << steeringWheelAngle << std::endl;
         // std::cout << "group_09;" << sMicro << ";" << steeringWheelAngle << ";" << gsr.groundSteering() << std::endl;
        }

        cv::Mat hsvImg;
        img.copyTo(hsvImg);
        cv::cvtColor(hsvImg, hsvImg, cv::COLOR_BGR2HSV);


        // --------------------------------------   Display center image  ------------------------------------------------------------
        // Create a separate copy of the image to overlay, define a rectangle representing the region of interest (ROI),
        // Draw a filled rectangle on the overlay image with a partially transparent red color,
        // and then overlay the modified image onto the original image using alpha blending.

        cv::Mat overlay = img.clone();
        cv::Rect color = cv::Rect(centerROI.x, centerROI.y, centerROI.width, centerROI.height);
        cv::rectangle(overlay, color, cv::Scalar(0, 0, 255, 128), -1);
        cv::addWeighted(overlay, alpha, img, 1 - alpha, 0, img);

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
