#include <stdio.h>
#include <sstream>
#include <string>
//#include <stlib.h>
#include <iostream>
//http://wiki.ros.org/action/fullsearch/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages?action=fullsearch&context=180&value=linkto%3A%22cv_bridge%2FTutorials%2FUsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages%22
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio.hpp>

static const std::string IMAGE_WINDOW = "Image window";
static const std::string HSV_WINDOW = "HSV window";
const std::string trackbarWindowName = "Trackbars";
//initial min and max HSV filter values, changed with trackbars later
int iLowH = 0;
int iHighH = 179;

int iLowS = 0;
int iHighS = 255;

int iLowV = 0;
int iHighV = 255;

void on_trackbar(int, void*);          //Refresh track bar

//cv::Mat previousFrame(1920,1080,CV_8UC1, cv::Scalar(0,0,0));        //Absolute difference of two HSV images
cv::Mat previousFrame;
cv::Mat absoluteDiff;
int counter = 0;
bool flag = 0;
//Use this to perform action with video sub everytime it's called
void imageCallBack(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
        try
        {
          cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
        }
    // Draw an example circle on the video stream
//    if (cv_ptr->image.rows > cv_ptr->image.rows/2 && cv_ptr->image.cols > cv_ptr->image.cols/2)
//        cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
    //cv::circle(cv_ptr->image,cv::Point(cv_ptr->image.rows/2,cv_ptr->image.cols/2), 10, CV_RGB(255,0,0));

    //Illumination correction (https://stackoverflow.com/questions/24341114/simple-illumination-correction-in-images-opencv-c)
//    Convert BGR to Lab
//    cv::Mat lab_image;
//    cv::cvtColor(cv_ptr->image, lab_image, CV_BGR2Lab);
//    //Extract L channel
//    std::vector<cv::Mat> lab_planes(3);
//    cv::split(lab_image, lab_planes);
//    //Apply CLAHE algorithm to L Channel
//    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
//    clahe->setClipLimit(4);
//    cv::Mat dst;
//    clahe->apply(lab_planes[0], dst);
//    // Merge the the color planes back into an Lab image
//    dst.copyTo(lab_planes[0]);
//    cv::merge(lab_planes, lab_image);
//    // convert back to RGB
//    cv::Mat image_clahe;
//    cv::cvtColor(lab_image, image_clahe, CV_Lab2BGR);


    cv::Mat imgHSV;
    //Convert BGR to HSV
    cvtColor(cv_ptr->image, imgHSV, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
    //Change HSV values
    cv::Mat imgThresholded;
    cv::inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

    if (flag == 0){
        previousFrame = imgThresholded;
        flag = 1;
    }
    cv::absdiff(imgThresholded,previousFrame, absoluteDiff);
    //Update previous frame every counter iterations
    if (counter == 1){
        previousFrame = imgThresholded;
        counter = 0;
    }
    counter++;

    cv::erode(imgThresholded,imgThresholded,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(5,5)),cv::Point(-1,-1),5); //1 at end is iterations, increase for more erosion
    cv::dilate(imgThresholded,imgThresholded,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(7,7)),cv::Point(-1,-1),3); //1 at end is iterations, increase for more erosion

//    flag = 1;

    //int type = imgThresholded.type();     //Check mat type //imgThresholded = 0 = CV_8UC1
//    std::cout << type << std::endl;
    // Update GUI Window
    cv::imshow(IMAGE_WINDOW, absoluteDiff);
//    cv::imshow(IMAGE_WINDOW,cv_ptr->image);
//    cv::imshow(HSV_WINDOW,image_clahe);
    cv::imshow(HSV_WINDOW, imgThresholded);
    cv::waitKey(3);

}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "hsv_converter");
    ros::NodeHandle nh;

    cv::namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

 //Create trackbars in "Control" window
 cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
 cvCreateTrackbar("HighH", "Control", &iHighH, 179);

 cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
 cvCreateTrackbar("HighS", "Control", &iHighS, 255);

 cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
 cvCreateTrackbar("HighV", "Control", &iHighV, 255);

  cv::namedWindow(IMAGE_WINDOW);
  cv::namedWindow(HSV_WINDOW);
  while (ros::ok())
  {
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("hsv",1);
  image_transport::Subscriber sub = it.subscribe("/kinect2/hd/image_color_rect", 1, imageCallBack);
  ros::spin();
  }
  cv::destroyWindow(IMAGE_WINDOW);
  cv::destroyWindow(HSV_WINDOW);
  cv::destroyWindow(trackbarWindowName);
}



