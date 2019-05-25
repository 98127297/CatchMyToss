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
//#include "background_segm.hpp"
//SYNCRHONISER FOR DEPTH AND IMAGE
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

static const std::string IMAGE_WINDOW = "Image window";
static const std::string PROCESSED_WINDOW = "Processed";
static const std::string DEPTH_WINDOW = "Depth window";
static const uint16_t MAX_KINECT_RANGE = 10000;
static const uint QUEUE_SIZE = 10;

//KINECT2
/*
 CALIBRATION MATRIX
  Normal camera matrix
  [ focal_length_x             0                principal_point_x;
              0            focal_length_y       principal_point_y;
              0                0                        1         ]
  For our Kinect 2
  [ 1.0605856774255615e+03             0                9.6239554359031615e+02;
              0            1.0634180897705726e+03       5.2329256537105312e+02;
              0                        0                          1            ]
*/
static const double FOCAL_LENGTH_X = 1.0605856774255615e+03;
static const double FOCAL_LENGTH_Y = 1.0634180897705726e+03;
static const double PRINCIPAL_POINT_X = 9.6239554359031615e+02;
static const double PRINCIPAL_POINT_Y = 5.2329256537105312e+02;

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
  //Depth map
  cv::Mat depthMap;
  //Circle image
  cv::Mat imageCircle;
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


public:
  ImageConverter()
      : it_(nh_), hsvMin(cv::Scalar(12,74,222)), hsvMax(cv::Scalar(38,255,255)),
        dilateElement(cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(3,3))), erodeElement(cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(3,3))),
        gaussianKernel(cv::Size(9,9)), subtractor(cv::createBackgroundSubtractorMOG2())
  {
      //SUBSCRIBERS
      // Subscrive to input video feed and publish output video feed
      //    image_sub_ = it_.subscribe("/kinect2/hd/image_color_rect", 1,
      //      &ImageConverter::imageCb, this);

      //    depth_sub_ = it_.subscribe("/kinect2/hd/image_depth_rect", 1,
      //      &ImageConverter::depthCb, this);

      //SYNCHRONISING [MAIN SUBSCIRBER]
      colorMsg.subscribe(nh_,"/kinect2/hd/image_color_rect",1);
      depthMsg.subscribe(nh_,"/kinect2/hd/image_depth_rect",1);
      //Note, change MySyncPolicy(queueSize) to change how many messages it compares
      sync_.reset(new Sync(MySyncPolicy(QUEUE_SIZE), colorMsg, depthMsg));
      //boost::bind uses generic numbers _1, _2, ..., _9 to represent arguments
      //After specifying function, first argument must be an instance of the member function's class
      //That's why 'this' is used because it references an instance of the class
      sync_->registerCallback(boost::bind(&ImageConverter::SyncCallBack, this, _1, _2));

      //PUBLISHERS
      //Publish the XYZ coordinates from camera + header
      ball_xyz_ = nh_.advertise<geometry_msgs::PointStamped>("/ball_XYZ", 1);

      //    image_pub_ = it_.advertise("/image_converter/output_image", 1);

      //    depth_pub_ = it_.advertise("/depth_converter/output_depth",1);

      cv::namedWindow(IMAGE_WINDOW);
      //    cv::namedWindow(PROCESSED_WINDOW);
      cv::namedWindow(DEPTH_WINDOW);
  }

  ~ImageConverter()
  {
      cv::destroyWindow(IMAGE_WINDOW);
      cv::destroyWindow(DEPTH_WINDOW);
      //    cv::destroyWindow(PROCESSED_WINDOW);
  }

  //Only perform this call back when synchronisation is successful
  void SyncCallBack(const sensor_msgs::ImageConstPtr& colorMsg, const sensor_msgs::ImageConstPtr& depthMsg)
  {
      //    ROS_INFO("Synchronization successful");
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
      cv::Point3d XYZ = findXYZ(ball, cv_depth_ptr);
//      std::cout << "[" << XYZ.x << "," << XYZ.y << "," << XYZ.z << "]" << std::endl;
      ballMsg.point.x = XYZ.x;
      ballMsg.point.y = XYZ.y;
      ballMsg.point.z = XYZ.z;
      ballMsg.header = cv_depth_ptr->header;
      ball_xyz_.publish(ballMsg);
      imageCircle = cv_color_ptr->image;
      cv::circle( imageCircle, ball, 3, cv::Scalar(0,255,0), -1, 8, 0 );


      //      cv::imshow(IMAGE_WINDOW, cv_color_ptr->image);
      cv::imshow(IMAGE_WINDOW, imageCircle);
      cv::imshow(DEPTH_WINDOW, cv_depth_ptr->image);
      cv::waitKey(3);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
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
      //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      // cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
      //cv::putText(cv_ptr->image,
      //	"Here is some text",
      //	cv::Point(cv_ptr->image.cols/2,cv_ptr->image.rows/2), // Coordinates
      //	cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
      //	1.0, // Scale. 2.0 = 2x bigger
      //	cv::Scalar(255,0,0), // BGR Color
      //	1, // Line Thickness (Optional)
      //	CV_AA); // Anti-alias (Optional)

      imageCircle = cv_ptr->image;
      //Convert image to HSV
      cvtColor(cv_ptr->image, imageHsv, cv::COLOR_BGR2HSV);

      //Filtering to find orange golf ball
      cv::inRange(imageHsv, hsvMin, hsvMax, filteredHsv);

      //Subtractor for masking out static
      subtractor->apply(filteredHsv, mask);

      //erode and dilate
      cv::erode(mask,mask,erodeElement,cv::Point(-1,-1),6);
      cv::dilate(mask,mask,dilateElement,cv::Point(-1,-1),6);

      cv::GaussianBlur(mask,mask,gaussianKernel,9,9);
      //    Hough Circle filter
      cv::HoughCircles(mask, circles, CV_HOUGH_GRADIENT, 1, mask.rows/2, 20, 10, 5, 100);

      //    if (flag == 0){
      //        previousFrame = filteredHsv;
      ////        std::cout << "Here" << std::endl;
      //        flag = 1;
      //    }
      //    cv::absdiff(filteredHsv,previousFrame, absoluteDiff);

      //    previousFrame = filteredHsv;
      //    cv::erode(absoluteDiff,absoluteDiff,erodeElement,cv::Point(-1,-1),1);
      //    cv::dilate(absoluteDiff,absoluteDiff,erodeElement,cv::Point(-1,-1),1);

      //Erode --> dilate = reduce white noise (morphological opening)
      //Erode to take away white noise
      //    cv::erode(filteredHsv,filteredHsv,erodeElement,cv::Point(-1,-1),3); //1 at end is iterations, increase for more erosion
      //Dilate to enhance white golf ball
      //    cv::dilate(filteredHsv,filteredHsv,dilateElement,cv::Point(-1,-1),5);
      //Fill out the golf ball
      //    cv::dilate(filteredHsv,filteredHsv,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(11,11)),cv::Point(-1,-1),5);

      //Dilate --> erode = increase white spots (morphological closing)
      //    cv::dilate(filteredHsv,filteredHsv,dilateElement,cv::Point(-1,-1),7);
      //cv::erode(filteredHsv,filteredHsv,erodeElement,cv::Point(-1,-1),1); //1 at end is iterations, increase for more erosion

      //Smoothing image
      //    cv::GaussianBlur(filteredHsv,filteredHsv,gaussianKernel,9,9);

      //    int type = filteredHsv.type();
      //    std::cout << type << std::endl;
      //    Hough Circle filter
      //    cv::HoughCircles(filteredHsv, circles, CV_HOUGH_GRADIENT, 1, filteredHsv.rows/8, 50, 25, 20, 100);

      //std::cout << circles.size() << std::endl;
      //Post circles to IMAGE_WINDOW
      for( size_t i = 0; i < circles.size(); i++ )
      {
          //std::cout << "circle: " << i << std::endl;
          cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
          int radius = cvRound(circles[i][2]);
          // circle center
          cv::circle( imageCircle, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
          // circle outline
          cv::circle( imageCircle, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
          cv::putText( imageCircle, "Test", center, cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(255,0,0),2);
      }
      //    // Update GUI Window
      cv::imshow(IMAGE_WINDOW, imageCircle);
      //    cv::imshow(IMAGE_WINDOW, cv_ptr->image);
      //    cv::imshow(PROCESSED_WINDOW, absoluteDiff);
      //    cv::imshow(IMAGE_WINDOW, filteredHsv);
      cv::imshow(PROCESSED_WINDOW, mask);
      cv::waitKey(3);

      // Output modified video stream
      image_pub_.publish(cv_ptr->toImageMsg());
  }
  
  void depthCb(const sensor_msgs::ImageConstPtr& msg)
  {
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
          cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
      }
      catch (cv_bridge::Exception& e)
      {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
      }

      // Draw an example circle on the video stream
      // if (cv_ptr->image.rows == image.rows/2 && cv_ptr->image.cols == image.cols/2)
      //   cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

      // Print pixel value of depth image at centre
      //std::cout << cv_ptr->image.at<uint16_t>(cv_ptr->image.rows/2,cv_ptr->image.cols/2) << std::endl;

      //Print pixel values at centre of screen
      //std::cout << cv_ptr->image.cols/2 << std::endl;	//960 pixel
      //std::cout << cv_ptr->image.rows/2 << std::endl;	//540 pixel

      depthMap = cv_ptr->image;   //Depth map represents each pixel as it's distance in millimeters
      //Brighten depth map by bumping up pixel values using percentage from max value, max value kinect2 can read accurately = 5 metres
      //    for(int i = 1; depthMap.cols < i; i++)
      //    {
      //        for(int j = 1; depthMap.rows < j; j++)
      //        {
      //            if(depthMap.at<uint16_t>(i,j) > MAX_KINECT_RANGE){
      //                depthMap.at<uint16_t>(i,j) = MAX_KINECT_RANGE;
      //            }
      //            depthMap.at<uint16_t>(i,j) = cvRound((depthMap.at<uint16_t>(i,j) / MAX_KINECT_RANGE)*65536);
      //        }
      //    }

      // Update GUI Window
      cv::imshow(DEPTH_WINDOW, depthMap);
      cv::waitKey(3);

      // Output modified video stream
      depth_pub_.publish(cv_ptr->toImageMsg());
  }

  cv::Point findBall(const cv_bridge::CvImagePtr& cv_ptr){
      //Convert image to HSV
      cvtColor(cv_ptr->image, imageHsv, cv::COLOR_BGR2HSV);

      //Filtering to find orange ping pong ball
      cv::inRange(imageHsv, hsvMin, hsvMax, filteredHsv);

      //Subtractor for masking out static objects
      subtractor->apply(filteredHsv, mask);

      //Erode and dilate (remove background noise)
      cv::erode(mask,mask,erodeElement,cv::Point(-1,-1),6);
      cv::dilate(mask,mask,dilateElement,cv::Point(-1,-1),6);

      //Smooth edges of ball
      cv::GaussianBlur(mask,mask,gaussianKernel,9,9);
      //Hough Circle filter (find and return largest circle)
      cv::HoughCircles(mask, circles, CV_HOUGH_GRADIENT, 1, mask.rows/2, 20, 10, 5, 100);
      int largestRadius = 0;
      cv::Point ball;
      for( size_t i = 0; i < circles.size(); i++ )
      {
          //std::cout << "circle: " << i << std::endl;
          cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
          //          std::cout << center << std::endl;
          int radius = cvRound(circles[i][2]);
          if (radius > largestRadius)
          {
              largestRadius = radius;
              ball = center;
          }
      }
      return ball;
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
      //      uint16_t depth = cv_depth_ptr->image.at<uint16_t>(ball);
      //      std::cout << depth << std::endl;
      //Depth at ball's 'centre'
      unsigned short depth = cv_ptr->image.at<unsigned short>(ball);
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
