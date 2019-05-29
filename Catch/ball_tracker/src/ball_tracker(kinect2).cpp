#include <stdio.h>
#include <sstream>
#include <string>
//#include <stlib.h>
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
//ZED camera
//#include <sl_zed/Camera.hpp>
//#include "background_segm.hpp"
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
static const double FOCAL_LENGTH_X = 1.0605856774255615e+03/2;
static const double FOCAL_LENGTH_Y = 1.0634180897705726e+03/2;
static const double PRINCIPAL_POINT_X = 9.6239554359031615e+02/2;
static const double PRINCIPAL_POINT_Y = 5.2329256537105312e+02/2;

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
//static const double FOCAL_LENGTH_X = 698.655;
//static const double FOCAL_LENGTH_Y = 698.655;
//static const double PRINCIPAL_POINT_X = 642.677;
//static const double PRINCIPAL_POINT_Y = 331.06;

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
  //Circles found by Hough Circles Transform
  std::vector<cv::Vec3f> circles;
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
      : it_(nh_), hsvMin(cv::Scalar(12,74,222)), hsvMax(cv::Scalar(38,255,255)),
        dilateElement(cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(3,3))), erodeElement(cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(3,3))),
        gaussianKernel(cv::Size(9,9)), subtractor(cv::createBackgroundSubtractorMOG2()), counter(0), flag(false)
  {
      ROS_INFO("Beginning Tracking");
      //SYNCHRONISING [MAIN SUBSCIRBER]
      //HD Topics, slower publishing but more resolution
//      colorMsg.subscribe(nh_,"/kinect2/hd/image_color_rect",1);
//      depthMsg.subscribe(nh_,"/kinect2/hd/image_depth_rect",1);
      //Quarter HD topics, (Way faster)
      colorMsg.subscribe(nh_,"/kinect2/qhd/image_color_rect",1);
      depthMsg.subscribe(nh_,"/kinect2/qhd/image_depth_rect",1);

      //ZED VGA topics
//      colorMsg.subscribe(nh_,"/zed/zed_node/rgb/image_rect_color",1);
//      depthMsg.subscribe(nh_,"/zed/zed_node/depth/depth_registered",1);

      //Note, change MySyncPolicy(queueSize) to change how many messages it compares
      sync_.reset(new Sync(MySyncPolicy(QUEUE_SIZE), colorMsg, depthMsg));
      //boost::bind uses generic numbers _1, _2, ..., _9 to represent arguments
      //After specifying function, first argument must be an instance of the member function's class
      //That's why 'this' is used because it references an instance of the class
      sync_->registerCallback(boost::bind(&ImageConverter::SyncCallBack, this, _1, _2));

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

  //Only perform this call back when synchronisation is successful
  void SyncCallBack(const sensor_msgs::ImageConstPtr& colorMsg, const sensor_msgs::ImageConstPtr& depthMsg)
  {
//      ROS_INFO("Topics matched, ID: %u",counter);
//      counter++;
      //Grab our images from messages
      cv_bridge::CvImagePtr cv_color_ptr;
      cv_bridge::CvImagePtr cv_depth_ptr;
      try
      {
          cv_color_ptr = cv_bridge::toCvCopy(colorMsg, sensor_msgs::image_encodings::BGR8);
          cv_depth_ptr = cv_bridge::toCvCopy(depthMsg, sensor_msgs::image_encodings::TYPE_16UC1);
      }
      catch (cv_bridge::Exception& e)
      {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
      }
      cv::Point ball;
      ball = findBall(cv_color_ptr);
      if (flag == true){
//      cv::Point3d XYZ = findXYZ(ball, cv_depth_ptr);      //Kinect 2
      std::vector<cv::Point> ballPoints = allCirclePoints(ball,mask.rows,mask.cols);
//        std::vector<cv::Point> ballPoints = allContourPoints(contoursBall,mask.rows,mask.cols);
//      largestRadius = 0;        //reset largestRadius
//      std::cout << ballPoints.size() << std::endl;
      cv::Point3d XYZ = findXYZAround(ballPoints, cv_depth_ptr);
//      cv::Point3d XYZ = findXYZZed(ball,depthMsg);          //ZED
//      std::cout << "[" << XYZ.x << "," << XYZ.y << "," << XYZ.z << "]" << std::endl;
      ballMsg.point.x = XYZ.x;
      ballMsg.point.y = XYZ.y;
      ballMsg.point.z = XYZ.z;
      ballMsg.header = cv_depth_ptr->header;
      ball_xyz_.publish(ballMsg);
      }
      //Flag = false, ball point is not good
      else{
          ballMsg.point.x = 0;
          ballMsg.point.y = 0;
          ballMsg.point.z = 0;
          ballMsg.header = cv_depth_ptr->header;
          ball_xyz_.publish(ballMsg);
      }


      //DEBUG
      cv::Mat imageCircle = cv_color_ptr->image;
//      Ball centre point
//      cv::circle(imageCircle, ball, 6, cv::Scalar(0,0,255), -1, 8, 0 );
//      Principal Point
      cv::circle(imageCircle, cv::Point(cvRound(PRINCIPAL_POINT_X),cvRound(PRINCIPAL_POINT_Y)), 3, cv::Scalar(255,0,204), -1, 8, 0);
      // Ball value zone
      cv::circle(imageCircle,ball,15,cv::Scalar(255),-1);
      //      cv::imshow(IMAGE_WINDOW, cv_color_ptr->image);
      //Draw contours on imageCircle
      if (flag == true){
      cv::drawContours(imageCircle, contoursBall, -1, cv::Scalar(0,0,255));
      }
      cv::imshow(IMAGE_WINDOW, imageCircle);
//      cv::imshow(DEPTH_WINDOW, cv_depth_ptr->image);
//      cv::imshow(HOUGH_WINDOW, filteredHsv);
      cv::imshow(HOUGH_WINDOW, mask);
//      cv::imshow("Test",cv::Mat::zeros(800,800, CV_8U));  //Display a black window
      cv::waitKey(3);
  }

  //Find ball using HSV filter for colour
  //Background subtraction and erode + dilate for noise reduction
  //Hough circle detector for finding ball

  cv::Point findBall(const cv_bridge::CvImagePtr& cv_ptr){
      //Convert image to HSV
      cvtColor(cv_ptr->image, imageHsv, cv::COLOR_BGR2HSV);

      //Filtering to find orange ping pong ball
      cv::inRange(imageHsv, hsvMin, hsvMax, filteredHsv);

      //Subtractor for masking out static objects
//      subtractor->apply(filteredHsv, mask);
      mask = filteredHsv;

      //Erode and dilate (remove background noise) [KINECT 2]
      cv::erode(mask,mask,erodeElement,cv::Point(-1,-1),2);
      cv::dilate(mask,mask,dilateElement,cv::Point(-1,-1),8);

//      //Erode and dilate (remove background noise) [ZED]
//      cv::erode(mask,mask,erodeElement,cv::Point(-1,-1),4);
//      cv::dilate(mask,mask,dilateElement,cv::Point(-1,-1),6);

      //Smooth edges of ball
      cv::GaussianBlur(mask,mask,gaussianKernel,9,9);
      //Hough Circle filter (find and return largest circle) [KINECT 2]
//      cv::HoughCircles(mask, circles, CV_HOUGH_GRADIENT, 1, mask.rows/2, 30, 15, 5, 100);

      cv::Point ball(0,0);
      //Centre of Mass using Moments (https://docs.opencv.org/2.4/doc/tutorials/imgproc/shapedescriptors/moments/moments.html)
      std::vector<std::vector<cv::Point> > contours;
      //Find Contours
      cv::findContours(filteredHsv, contours, CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE, cv::Point(0,0));
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
      //Hough Circle [ZED]
//      cv::HoughCircles(mask, circles, CV_HOUGH_GRADIENT, 1, mask.rows/2, 40, 20, 5, 100);

//      for( size_t i = 0; i < circles.size(); i++ )
//      {
//          //std::cout << "circle: " << i << std::endl;
//          cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
//          //          std::cout << center << std::endl;
//          int radius = cvRound(circles[i][2]);
//          if (radius > largestRadius)
//          {
//              largestRadius = radius;
//              ball = center;
//          }
//      }
//      //Just in case
//      if (circles.size() == 0) {
//          largestRadius = 0;
//          flag = false;
//      }
//      int pixelColor = mask.at<uchar>(ball);
//      std::cout << pixelColor[0] << ";" << pixelColor[1] << ";" << pixelColor[2] << std::endl;
//      std::cout << pixelColor << std::endl;
//      std::cout << "X: " << ball.x << " , Y: " << ball.y << std::endl;
      //Return ball centre point
      return ball;
  }

  //Find all points within circle of Hough Circle transform
  //https://stackoverflow.com/questions/20378050/access-pixel-values-inside-the-detected-object-c
  std::vector<cv::Point> allCirclePoints(cv::Point ball, int image_height, int image_width){
      int radius = 15;
      std::vector<cv::Point> ballPoints;
      cv::Mat mask = cv::Mat::zeros(image_height,image_width, CV_8U);
//      std::cout << cvRound(radius/2) << std::endl;
      cv::circle(mask,ball,cvRound(radius),cv::Scalar(255),-1);
//      imshow("Test",mask);
//      cv::waitKey(3);
      for(unsigned int y=0; y<image_height; ++y)
        for(unsigned int x=0; x<image_width; ++x)
          if(mask.at<unsigned char>(y,x) > 0)
          {
            cv::Point ballPoint(x,y);
            ballPoints.push_back(ballPoint);
          }
      return ballPoints;
  }

  //Find all points inside a contour
  std::vector<cv::Point> allContourPoints(std::vector<std::vector<cv::Point> > contours,int image_height, int image_width){
      std::vector<cv::Point> ballPoints;
      cv::Mat mask = cv::Mat::zeros(image_height,image_width, CV_8U);
      cv::drawContours(mask, contours, -1, cv::Scalar(255),-1);
//      cv::imshow("test", mask);
      for(unsigned int y=0; y<image_height; ++y)
        for(unsigned int x=0; x<image_width; ++x)
          if(mask.at<unsigned char>(y,x) > 0)
          {
            cv::Point ballPoint(x,y);
            ballPoints.push_back(ballPoint);
          }
      return ballPoints;
  }

  //Finds the smallest depth value using a vector of image points
  cv::Point3d findXYZAround(std::vector<cv::Point> ballPoints, const cv_bridge::CvImagePtr& cv_ptr){
      unsigned short currentDepth;
      unsigned short depth;
      int currentU;
      int currentV;
      int u;
      int v;
      cv::Point ball;
      bool zeroFlag = false;
      //Kinect
      for (int i = 0; i < ballPoints.size(); i++){
          ball = ballPoints.at(i);
          currentU = ball.x;
          currentV = ball.y;
          currentDepth = cv_ptr->image.at<unsigned short>(ball);
          //Need to find depth value that isn't 0 to fill depth with
          if (currentDepth > 0 && zeroFlag == false){
              zeroFlag = true;
              depth = currentDepth;
          }
          //Skip until you find a value that isn't zero depth
          else if (zeroFlag == false){
              continue;
          }
          if (currentDepth < depth && currentDepth != 0){
              depth = currentDepth;
              u = currentU;
              v = currentV;
          }
      }
      //Just in case we exit loop with no depth
      if (zeroFlag == false){
          depth = 0;
      }
      //Depth is a crazy value, don't consider
      if (depth > 2300){
          std::cout << depth << std::endl;
          depth = 0;
      }
      //Using intrinsic parameters of camera we now convert (u,v) and depth into
      //(X,Y,Z) from camera.
      double X = depth*(u - PRINCIPAL_POINT_X) / FOCAL_LENGTH_X;
      double Y = depth*(v - PRINCIPAL_POINT_Y) / FOCAL_LENGTH_Y;

      //Vector of XYZ values
      cv::Point3d XYZ(X,Y,depth);
      return XYZ;

  }

  //Finding the XYZ coordinates of ball based on Kinect intrinsic parameters
  cv::Point3d findXYZ(cv::Point ball, const cv_bridge::CvImagePtr& cv_ptr){
      //Pixel coordinates where ball was seen
      /*
       IMAGE FRAME
       ------------> u axis (columns)
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
      //      std::cout << depth << std::endl;
      //Depth at ball's 'centre' [Kinect 2]
      unsigned short depth = cv_ptr->image.at<unsigned short>(ball);
      //Using intrinsic parameters of camera we now convert (u,v) and depth into
      //(X,Y,Z) from camera.
      double X = depth*(u - PRINCIPAL_POINT_X) / FOCAL_LENGTH_X;
      double Y = depth*(v - PRINCIPAL_POINT_Y) / FOCAL_LENGTH_Y;

      //Vector of XYZ values
      cv::Point3d XYZ(X,Y,depth);
      return XYZ;

  }
  //Finding the XYZ coordinates of ball based on ZED intrinsic parameters
  //NOTE: ZED uses message->data instead of message->image
  cv::Point3d findXYZZed(cv::Point ball, const sensor_msgs::Image::ConstPtr& msg){
      //Pixel coordinates where ball was seen
      /*
       IMAGE FRAME
       ------------> u axis (columns)
       |
       |
       |
       |
       |
       v
         v axis (rows)
       */


      //      std::cout << depth << std::endl;
      //Depth [ZED VGA]
//      float *depths = (float*)(&msg->data[0]);
//      int u;
//      int v;
//      int currentU;
//      int currentV;
//      float currentDepth;
//      float depth;
//      cv::Point ball;
//      int ballPoint;
//      //Find the smallest depth in the vector
//      for (int i = 0; i < ballPoints.size(); i++){
//          ball = ballPoints.at(i);
//          currentU = ball.x;
//          currentV = ball.y;
//          //Find index of data array that corresponds to the depth of the ball centre pixel
//          ballPoint = currentU + msg->width * currentV;
//          currentDepth = depths[ballPoint];
//          if (i == 0){
//              //Load up the first depth value to compare
//               depth = currentDepth;
//          }
//          if (depth > currentDepth){
//              u = currentU;
//              v = currentV;
//              depth = currentDepth;
//          }
//      }
//      //No ball points found, we assume 0 values
//      if (ballPoints.size() == 0){
//          depth = 0;
//      }
      float *depths = (float*)(&msg->data[0]);
      int u = ball.x;
      int v = ball.y;
      int centre = u + msg->width * v;
      float depth = depths[centre];
      //Using intrinsic parameters of camera we now convert (u,v) and depth into
      //(X,Y,Z) from camera.
      double X = depth*(u - PRINCIPAL_POINT_X) / FOCAL_LENGTH_X;
      double Y = depth*(v - PRINCIPAL_POINT_Y) / FOCAL_LENGTH_Y;

      //Vector of XYZ values
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
