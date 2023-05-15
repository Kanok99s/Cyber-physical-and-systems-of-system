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
int YELLOW_MIN_HUE_VALUE = 15;
int YELLOW_MAX_HUE_VALUE = 25;
int YELLOW_MIN_SAT_VALUE = 75;
int YELLOW_MAX_SAT_VALUE = 185;
int YELLOW_MIN_VAL_VALUE = 147;
int YELLOW_MAX_VAL_VALUE = 255;

// HSV values for blue cones
int BLUE_MIN_HUE_VALUE = 100;
int BLUE_MAX_HUE_VALUE = 140;
int BLUE_MIN_SAT_VALUE = 120;
int BLUE_MAX_SAT_VALUE = 255;
int BLUE_MIN_VAL_VALUE = 40;
int BLUE_MAX_VAL_VALUE = 255;

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

// stores image when identifying car direction

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
// cv::Rect centerRegionOfInterest = cv::Rect(200, 245, 250, 115);

int32_t main(int32_t argc, char **argv)
{
  int32_t retCode{
      1};
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

        // Define the images we need for yellow cones on the right
        cv::Mat yellowHsvImage;
        cv::Mat detectYellowImg;

        // Define the images we need for center cones detecetion
        cv::Mat hsvCenterImg;
        cv::Mat detectCenterImg;

        // we first need to find yellow cones in HSV image

        // loop runs until frame counter is greater than the sample size of 5, used to determine direction (counterclockwise, clockwise etc...)
        if (numberOfFrames < maxFrames)
        {

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
          for (unsigned int i = 0; i < contours.size(); i++)
          {

            // If the current index of the vector has a contour area that is larger than the defined number of pixels in identifiedShape, we have a cone
            if (cv::contourArea(contours[i]) > identifiedShape)
            {

              // Draws the contour of the cone on the image
              cv::Scalar colour(255, 255, 0);
              cv::drawContours(rightYellowConesContourImg, contours, i, colour, -1, 8, hierarchy);

              // Set yellowConeFound totrue to indicate that we have found a flag
              yellowConeFound = true;

              // If yellow cones are detected, that means the car direction is clockwise and the carDirection must be set as 1
              if (yellowConeFound == true)
              {
                carDirection = 1;
              }
            }
          }
        }

        // If the number of frames is larger than or equal to the fixed frame size
        if (numberOfFrames >= maxFrames)
        {

          // Capture the image of the region of interest we defined before
          cv::Mat centreImg = img(centerRegionOfInterest);

          // converting the BRG colors of the image to HSV values for better image processing
          cv::cvtColor(centreImg, hsvCenterImg, cv::COLOR_BGR2HSV);

          // Threshold the blueHsvImage based on color ranges we defined and store the result in the centerImage as an input image
          cv::inRange(hsvCenterImg, cv::Scalar(BLUE_MIN_HUE_VALUE, BLUE_MIN_SAT_VALUE, BLUE_MIN_VAL_VALUE), cv::Scalar(BLUE_MAX_HUE_VALUE, BLUE_MAX_SAT_VALUE, BLUE_MAX_VAL_VALUE), detectCenterImg);

          // Applying Gaussian blur to detectCenterImg
          cv::GaussianBlur(detectCenterImg, detectCenterImg, cv::Size(5, 5), 0);

          // Applying dilate and erode to detectCenterImg to remove holes from foreground

          // fill in small gaps in the centerImage and expand the size of the objects in the image
          cv::dilate(detectCenterImg, detectCenterImg, 0);

          // Applying erosion to avoid the appearance of overlapping objects on the cones
          cv::erode(detectCenterImg, detectCenterImg, 0);

          // finding the contours of the cones in centerImage and store them in the contours vector
          cv::findContours(detectCenterImg, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

          // Create a new image to store the detected blue cones contours with the same size as center image
          cv::Mat blueContourImg = cv::Mat::zeros(detectCenterImg.rows, detectCenterImg.cols, CV_8UC3);

          bool blueConeCenter = false; // Flag for whether blue cones are detected in the image

          // Loops over the contours vector
          for (unsigned int i = 0; i < contours.size(); i++)
          {

            // If the current index of the vector has a contour area that is larger than the defined number of pixels in identifiedShape, we have a cone
            if (cv::contourArea(contours[i]) > identifiedShape)
            {
              // Draws the contour of the cone on the image
              cv::Scalar colour(255, 255, 0);
              cv::drawContours(blueContourImg, contours, i, colour, -1, 8, hierarchy);

              blueConeCenter = true; // set to true because a cone has been detected

              // when current steeringAngle is less than maxSteering and more than minSteering
              if (steeringWheelAngle > minSteering && steeringWheelAngle < maxSteering)
              {

                // If a blue cone is not yet been detected on the center

                  if (carDirection == 1)
                  {                                  // if the car direction is clockwise
                    //steeringWheelAngle -= turnRight; // the car is turning right
                    steeringWheelAngle = (steeringWheelAngle*0.5) - turnRight;
                  }
                  else if (carDirection == -1)
                  {                                 // If a blue cone not detected yet & car direction is counterclockwise
                    //steeringWheelAngle -= turnLeft; // the car is turning left
                    steeringWheelAngle = (steeringWheelAngle*0.5) - turnLeft;
                  
                }

              } // If the current steering angle is less than steeringMin or more than steeringMax
            }
          }
          // Pop up window used for testing
          // If verbose is included in the command line, a window showing only the blue contours will appear
          if (VERBOSE)
          {
            cv::imshow("Blue Contours", blueContourImg);
            cv::waitKey(1);
          }

          // If a blue cone is not detected on the center ROI, we check for yellow cones
          if (blueConeCenter == false)
          {

            // converting the BRG colors of the image to HSV values for better image processing
            cv::cvtColor(centreImg, hsvCenterImg, cv::COLOR_BGR2HSV);

            // Threshold the hsvCenterImg based on color ranges we defined and store the result in the detectCenterImage as an input image
            cv::inRange(hsvCenterImg, cv::Scalar(YELLOW_MIN_HUE_VALUE, YELLOW_MIN_SAT_VALUE, YELLOW_MIN_VAL_VALUE), cv::Scalar(YELLOW_MAX_HUE_VALUE, YELLOW_MAX_SAT_VALUE, YELLOW_MAX_VAL_VALUE), detectCenterImg);

            // Applying Gaussian blur to detectCenterImg
            cv::GaussianBlur(detectCenterImg, detectCenterImg, cv::Size(5, 5), 0);

            // Applying dilate and erode to detectCenterImg to remove holes from foreground

            // fill in small gaps in the centerImage and expand the size of the objects in the image
            cv::dilate(detectCenterImg, detectCenterImg, 0);

            // Applying erosion to avoid the appearance of overlapping objects on the cones
            cv::erode(detectCenterImg, detectCenterImg, 0);

            // The below will find the contours of the cones in detectLeftImg and store them in the contours vector
            cv::findContours(detectCenterImg, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

            // Creates a mat object of the same size as detectCenterImg used for storing the drawn contours
            cv::Mat yellowContourImg = cv::Mat::zeros(detectCenterImg.rows, detectCenterImg.cols, CV_8UC3);

            bool yellowConeCenter = false; // Flag for whether yellow cones are detected in the image

            // Loops over the contours vector
            for (unsigned int i = 0; i < contours.size(); i++)
            {
              // If the current index of the vector has a contour area that is larger than the defined number of pixels in identifiedShape, we have a cone
              if (cv::contourArea(contours[i]) > identifiedShape)
              {
                // Draws the contour of the cone on the image
                cv::Scalar colour(255, 255, 0);
                cv::drawContours(yellowContourImg, contours, i, colour, -1, 8, hierarchy);

                yellowConeCenter = true; // set to true because a cone has been detected


                // when current steeringAngle is less than maxSteering and more than minSteering
                if (steeringWheelAngle > minSteering && steeringWheelAngle < maxSteering)
                {
                  // If a yellow cone is not yet been detected on the center

                    if (carDirection == 1)
                    {                                 // if the car direction is clockwise
                      //steeringWheelAngle -= turnLeft; // the car is turning left
                      steeringWheelAngle = (steeringWheelAngle*0.5) - turnLeft;
                    }
                    else if (carDirection == -1)
                    {                                  // If car direction is counterclockwise
                      //steeringWheelAngle -= turnRight; // the car is turning right
                      steeringWheelAngle = (steeringWheelAngle*0.5) - turnRight;
                    }
                  

                } // If the current steering angle is less than steeringMin or more than steeringMax
              }
            }
            // Pop up window used for testing
            // If verbose is included in the command line, a window showing only the yellow contours will appear
            if (VERBOSE)
            {
              cv::imshow("Yellow Contours", yellowContourImg);
              cv::waitKey(1);
            }

            // If no blue or yellow cones have been detected in the center
            if (yellowConeCenter == false && blueConeCenter == false)
            {
              // If no cones are present, the steeringWheelAngle is set to 0
              steeringWheelAngle = 0.00;
            }
          }
        }

        // creates string stream input, optimized buffer, convert whatever is coming in as string
        std::ostringstream calcGroundSteering;
        std::ostringstream actualSteering;
        std::ostringstream timestamp;

        // putting values into stream
        calcGroundSteering << steeringWheelAngle;
        actualSteering << gsr.groundSteering();
        timestamp << sMicro;

        // creating strings for printing
        std::string time = " Time Stamp: ";
        std::string calculatedGroundSteering = "Calculated Ground Steering: ";
        std::string actualGroundSteering = " Actual Ground Steering: ";
        std::string groundSteeringAngle = std::to_string(steeringWheelAngle);

        // appending into one string to display
        calculatedGroundSteering.append(groundSteeringAngle);
        calculatedGroundSteering.append(calcGroundSteering.str());
        actualGroundSteering.append(actualSteering.str());
        time.append(timestamp.str());

        cv::Mat hsvImg;
        // Copy the original image to the new one
        img.copyTo(hsvImg);
        // Convert the new image to the hsv color space
        cv::cvtColor(hsvImg, hsvImg, cv::COLOR_BGR2HSV);

        // Checking performance
          double allowedDeviation = 0.05;
          //double allowedDeviation = 0.3;
          double actualAngle = gsr.groundSteering();
          if (std::abs(actualAngle - steeringWheelAngle) <= allowedDeviation)
            withinRangeFrames++;

        totalFrames++;

        // Displaying performance info
        std::string percentMsg = "Performance: ";
        double percent = (double)withinRangeFrames / (double)totalFrames * 100;
        percentMsg += std::to_string(percent) + "%";
        cv::putText(img, percentMsg, cv::Point(80, 140), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 250, 154), 1);

        // Displays information on video
        cv::putText(img, calculatedGroundSteering, cv::Point(80, 50), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 250, 154), 1);
        cv::putText(img, actualGroundSteering, cv::Point(80, 80), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 250, 154), 1);
        cv::putText(img, time, cv::Point(80, 110), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 250, 154), 1);

        {
          std::lock_guard<std::mutex> lck(gsrMutex);
          std::cout << "group_09;" << sMicro << ";" << steeringWheelAngle << std::endl;
         //std::cout << "group_09;" << sMicro << ";" << steeringWheelAngle << ";" << gsr.groundSteering() << ";" << " car direction: " << carDirection << std::endl;
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
