#include <stdio.h>
#include <sstream>
#include <string>
#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/PointStamped.h>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>

//SYNCRHONISER FOR DEPTH AND IMAGE
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

static const std::string IMAGE_WINDOW = "Image window";
static const std::string HOUGH_WINDOW = "Hough window";
static const std::string DEPTH_WINDOW = "Depth window";
static const uint16_t MAX_KINECT_RANGE = 10000;
static const uint QUEUE_SIZE = 10;

//KINECT2 [RGB-D Camera]
/*
 CALIBRATION MATRIX
  Normal camera matrix
  [ focal_length_x             0                principal_point_x;
              0            focal_length_y       principal_point_y;
              0                0                        1         ]
  For our Kinect 2 [HD Resolution (1920 x 1080)]
  [ 1.0605856774255615e+03             0                9.6239554359031615e+02;
              0            1.0634180897705726e+03       5.2329256537105312e+02;
              0                        0                          1            ]
*/
//HD
//static const double FOCAL_LENGTH_X = 1.0605856774255615e+03;
//static const double FOCAL_LENGTH_Y = 1.0634180897705726e+03;
//static const double PRINCIPAL_POINT_X = 9.6239554359031615e+02/2;
//static const double PRINCIPAL_POINT_Y = 5.2329256537105312e+02/2;
//QHD
//static const double FOCAL_LENGTH_X = 1.0605856774255615e+03/2;
//static const double FOCAL_LENGTH_Y = 1.0634180897705726e+03/2;
//static const double PRINCIPAL_POINT_X = 9.6239554359031615e+02/2;
//static const double PRINCIPAL_POINT_Y = 5.2329256537105312e+02/2;

//ZED [Stereo Camera]
static const int baseline = 120;    //mm
//Depth and image registered on left camera
//Left camera matrix [VGA Resolution (672 x 376)]
//[         350.49             0                336.826;
//            0              350.49             183.383;
//            0                0                   1   ]
//static const double FOCAL_LENGTH_X = 350.49;
//static const double FOCAL_LENGTH_Y = 350.49;
//static const double PRINCIPAL_POINT_X = 336.826;
//static const double PRINCIPAL_POINT_Y = 183.383;

//Left camera matrix [HD Resolution (1280 x 720)]
//[         698.655             0                642.677;
//            0               698.655             331.06;
//            0                 0                   1   ]
static const double FOCAL_LENGTH_X = 698.655;
static const double FOCAL_LENGTH_Y = 698.655;
static const double PRINCIPAL_POINT_X = 642.677;
static const double PRINCIPAL_POINT_Y = 331.06;

class ImageConverter
{
  //ROS Variables
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  image_transport::Subscriber depth_sub_;
  image_transport::Publisher depth_pub_;
  ros::Publisher ball_xyz_;
  geometry_msgs::PointStamped ballMsg;

  //Message filters for synchronisation (https://gist.github.com/tdenewiler/e2172f628e49ab633ef2786207793336)
  message_filters::Subscriber<sensor_msgs::Image> colorMsg;
  message_filters::Subscriber<sensor_msgs::Image> depthMsg;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;

  //OpenCV Variables
  //Thresholds for the HSV filtering
  cv::Scalar hsvMin;
  cv::Scalar hsvMax;
  //HSV image
  cv::Mat imageHsv;
  //Filtered HSV image
  cv::Mat filteredHsv;
  //Structuring Element - dilation
  cv::Mat dilateElement;
  //Structuring Element - erosion
  cv::Mat erodeElement;
  //Gaussian kernel
  cv::Size gaussianKernel;
  //Background subtractor
  cv::Ptr<cv::BackgroundSubtractor> subtractor;
  //Mask for background subtractor
  cv::Mat mask;
  //Contours of ball
  std::vector<std::vector<cv::Point> > contoursBall;
  //largest radius of circle;
  int largestRadius;

  //Flag for ensuring ball point is actually right
  bool flag;

  //Keeps track of messages sent out
  unsigned int counter;

  //HSV for Kinect 2 hsvMin(cv::Scalar(12,74,222)), hsvMax(cv::Scalar(38,255,255))
  //HSV for ZED hsvMin(cv::Scalar(13,184,187)), hsvMax(cv::Scalar(35,255,255)

public:
  ImageConverter()
      : it_(nh_), hsvMin(cv::Scalar(13,184,187)), hsvMax(cv::Scalar(35,255,255)),
        dilateElement(cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(3,3))), erodeElement(cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(3,3))),
        gaussianKernel(cv::Size(9,9)), subtractor(cv::createBackgroundSubtractorMOG2()), counter(0), flag(false)
  {
      ROS_INFO("Beginning Tracking");
      
      //Subscribing to ZED image topic
      image_sub_ = it_.subscribe("/zed/zed_node/stereo/image_rect_color",1,&ImageConverter::imageCallBack,this);

      //PUBLISHERS
      //Publish the XYZ coordinates from camera + header
      ball_xyz_ = nh_.advertise<geometry_msgs::PointStamped>("/ball_XYZ", 1);

      //Debug windows
//      cv::namedWindow(IMAGE_WINDOW, cv::WINDOW_NORMAL);
//      cv::resizeWindow(IMAGE_WINDOW,1920,1080);
//      cv::namedWindow(DEPTH_WINDOW, cv::WINDOW_NORMAL);
//      cv::resizeWindow(DEPTH_WINDOW,1920,1080);
//      cv::namedWindow(HOUGH_WINDOW, cv::WINDOW_NORMAL);
//      cv::resizeWindow(HOUGH_WINDOW,1920,1080);
  }

  ~ImageConverter()
  {
      //DEBUG, uncomment required windows when you need them
      //      cv::destroyWindow(IMAGE_WINDOW);
      //      cv::destroyWindow(DEPTH_WINDOW);
//          cv::destroyWindow(HOUGH_WINDOW);
  }

  //Call back for ZED stereo images
  void imageCallBack(const sensor_msgs::ImageConstPtr& colorMsg)
  {
      ROS_INFO("Topics matched, ID: %u",counter);
      counter++;
      //Grab our images from messages
      cv_bridge::CvImagePtr cv_color_ptr;
      try
      {
          cv_color_ptr = cv_bridge::toCvCopy(colorMsg, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception& e)
      {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
      }
      //Do filtering of both images
      filterImage(cv_color_ptr);
      //Split the images [Full image]
//      cv::Mat leftImage = mask(cv::Range(0,mask.rows), cv::Range(0,mask.cols/2));
//      cv::Mat rightImage = mask(cv::Range(0,mask.rows),cv::Range(mask.cols/2,mask.cols));

      //Split images [Region of Interest, middle of the image]
      cv::Mat leftImage = mask(cv::Range(0,mask.rows), cv::Range(cvRound(mask.cols/8),cvRound(mask.cols/2 - mask.cols/8)));
      cv::Mat rightImage = mask(cv::Range(0,mask.rows), cv::Range(cvRound(mask.cols/2 + mask.cols/8),cvRound(mask.cols - mask.cols/8)));
      int xOffset = cvRound(mask.cols/8);  //Required for calculation of XYZ coordinates
//      imshow("Left",leftImage);
//      imshow("Right",rightImage);

//      cv::Mat stereoImage = cv_color_ptr->image;
      //Full split image
//      cv::Mat leftImageColor = stereoImage(cv::Range(0,stereoImage.rows), cv::Range(0,stereoImage.cols/2));
//      cv::Mat rightImageColor = stereoImage(cv::Range(0,stereoImage.rows),cv::Range(stereoImage.cols/2,stereoImage.cols));

      //Region of interest split image
//      cv::Mat leftImageColor = stereoImage(cv::Range(0,stereoImage.rows), cv::Range(cvRound(stereoImage.cols/8),cvRound(stereoImage.cols/2 - stereoImage.cols/8)));
//      cv::Mat rightImageColor = stereoImage(cv::Range(0,stereoImage.rows),cv::Range(cvRound(stereoImage.cols/2 + stereoImage.cols/8),cvRound(stereoImage.cols - stereoImage.cols/8)));

      //Find ball
      cv::Point leftBall = findBallCropped(leftImage,xOffset);
      std::vector<std::vector<cv::Point> > leftContours = contoursBall;
      cv::Point rightBall = findBallCropped(rightImage,xOffset);
      std::vector<std::vector<cv::Point> > rightContours = contoursBall;

      //Show contours on color image
//      if (flag == true){
//          cv::drawContours(leftImageColor, leftContours, -1, cv::Scalar(0,0,255));
//          cv::drawContours(rightImageColor, rightContours, -1, cv::Scalar(0,0,255));
//      }

//      cv::circle(leftImageColor,leftBall,6,cv::Scalar(0,0,255),-1, 8, 0 );
//      cv::circle(rightImageColor,rightBall,6,cv::Scalar(0,0,255),-1, 8, 0);
//      imshow("Left", leftImageColor);
//      imshow("Right", rightImageColor);

      //Disparity
      //Rows are the same, only look at columns
      int disparity = leftBall.x - rightBall.x;

      //Depth Formula [Left Camera]
      double depth = (FOCAL_LENGTH_X*baseline)/disparity;

      //Publish depth
      //Only publish if flag = true [depth reading correct, contour seen]
      if (flag == true){
          cv::Point3d XYZ = findXYZ(leftBall,depth);
          ballMsg.point.x = XYZ.x;
          ballMsg.point.y = XYZ.y;
          ballMsg.point.z = XYZ.z;
          ballMsg.header = cv_color_ptr->header;
          ball_xyz_.publish(ballMsg);

      }

      //DEBUG
//      cv::Mat imageCircle = cv_color_ptr->image;
////      Ball centre point
////      cv::circle(imageCircle, ball, 6, cv::Scalar(0,0,255), -1, 8, 0 );
////      Principal Point
//      cv::circle(imageCircle, cv::Point(cvRound(PRINCIPAL_POINT_X),cvRound(PRINCIPAL_POINT_Y)), 3, cv::Scalar(255,0,204), -1, 8, 0);
//      // Ball value zone
//      cv::circle(imageCircle,ball,15,cv::Scalar(255),-1);
//      //      cv::imshow(IMAGE_WINDOW, cv_color_ptr->image);
//      //Draw contours on imageCircle
//      if (flag == true){
//      cv::drawContours(imageCircle, contoursBall, -1, cv::Scalar(0,0,255));
//      }
//      cv::imshow(IMAGE_WINDOW, imageCircle);
//      cv::imshow(DEPTH_WINDOW, cv_depth_ptr->image);
//      cv::imshow(HOUGH_WINDOW, filteredHsv);
//      cv::imshow(HOUGH_WINDOW, mask);
//      cv::imshow("Test",cv::Mat::zeros(800,800, CV_8U));  //Display a black window
//      cv::waitKey(3);
  }

  //Filter input image
  void filterImage(const cv_bridge::CvImagePtr& cv_ptr){
      //Convert image to HSV
      cvtColor(cv_ptr->image, imageHsv, cv::COLOR_BGR2HSV);

      //Filtering to find orange ping pong ball
      cv::inRange(imageHsv, hsvMin, hsvMax, filteredHsv);

      //Subtractor for masking out static objects
//      subtractor->apply(filteredHsv, mask);
      mask = filteredHsv;

//      //Erode and dilate (remove background noise) [ZED]
      cv::erode(mask,mask,erodeElement,cv::Point(-1,-1),6);
      cv::dilate(mask,mask,dilateElement,cv::Point(-1,-1),6);

      //Smooth edges of ball
      cv::GaussianBlur(mask,mask,gaussianKernel,9,9);
  }

 //Find ball using contours and centre of mass [Full image]
  cv::Point findBall(cv::Mat filteredImage){
      cv::Point ball(0,0);
      //Centre of Mass using Moments (https://docs.opencv.org/2.4/doc/tutorials/imgproc/shapedescriptors/moments/moments.html)
      std::vector<std::vector<cv::Point> > contours;
      //Find Contours
      cv::findContours(filteredImage, contours, CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE, cv::Point(0,0));
//      cv::Mat contourTest = cv::Mat::zeros(mask.rows,mask.cols,CV_8UC3);

//      std::cout << contours.size() << std::endl;
      //Only do this if contours are available
      if (contours.size() > 0){
          flag = true;
          //Get moments
          //      std::vector<cv::Moments> mu(contours.size());
          //      for (int i = 0; i < contours.size(); i++){
          //          mu[i] = cv::moments(contours[i], false);
          //      }
          cv::Moments mu = cv::moments(contours[0],true);
          //Get mass centres
          //      std::vector<cv::Point2f> mc(contours.size());
          //      for (int i = 0; i < contours.size(); i++){
          //          mc[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
          //      }
          cv::Point2f mc = cv::Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );
          //Include xOffset for cropped images (offset required)
          ball = cv::Point(cvRound(mc.x),cvRound(mc.y));
          //Draw contours and centre
//          cv::drawContours(contourTest, contours, -1, cv::Scalar(0,0,255));
//          cv::circle(contourTest, mc, 3, cv::Scalar(0,255,0));
          contoursBall = contours;
//          cv::imshow("Contours", contourTest);
      }
      else{
//          cv::imshow("Contours",cv::Mat::zeros(mask.rows,mask.cols,CV_8UC3));
          flag = false;

      }
      //Return ball centre point
      return ball;
  }

  //Find ball using using contours and centre of mass [Cropped images]
  cv::Point findBallCropped(cv::Mat filteredImage,int xOffset){
      cv::Point ball(0,0);
      //Centre of Mass using Moments (https://docs.opencv.org/2.4/doc/tutorials/imgproc/shapedescriptors/moments/moments.html)
      std::vector<std::vector<cv::Point> > contours;
      //Find Contours
      cv::findContours(filteredImage, contours, CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE, cv::Point(0,0));
   
      //Only do this if contours are available
      if (contours.size() > 0){
          flag = true;
          //Get moments
          //      std::vector<cv::Moments> mu(contours.size());
          //      for (int i = 0; i < contours.size(); i++){
          //          mu[i] = cv::moments(contours[i], false);
          //      }
          cv::Moments mu = cv::moments(contours[0],true);
          //Get mass centres
          //      std::vector<cv::Point2f> mc(contours.size());
          //      for (int i = 0; i < contours.size(); i++){
          //          mc[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
          //      }
          cv::Point2f mc = cv::Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );
          //Include xOffset for cropped images (offset required)
          ball = cv::Point(cvRound(mc.x)+xOffset,cvRound(mc.y));
          //Draw contours and centre
//          cv::drawContours(contourTest, contours, -1, cv::Scalar(0,0,255));
//          cv::circle(contourTest, mc, 3, cv::Scalar(0,255,0));
          contoursBall = contours;
//          cv::imshow("Contours", contourTest);
      }
      else{
//          cv::imshow("Contours",cv::Mat::zeros(mask.rows,mask.cols,CV_8UC3));
          flag = false;

      }
      //Return ball centre point
      return ball;
  }

  //Finding the XYZ coordinates of ball based on intrinsic parameters
  cv::Point3d findXYZ(cv::Point ball, double depth){
      //Pixel coordinates where ball was seen
      /*
       IMAGE FRAME
       0/0-----------> u axis (columns)
       |
       |
       |
       |
       |
       v
         v axis (rows)
       */
      int u = ball.x;
      int v = ball.y;
   
      //(X,Y,Z) from camera.
      double X = depth*(u - PRINCIPAL_POINT_X) / FOCAL_LENGTH_X;
      double Y = depth*(v - PRINCIPAL_POINT_Y) / FOCAL_LENGTH_Y;

      //Point of XYZ values
      cv::Point3d XYZ(X,Y,depth);
      return XYZ;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
