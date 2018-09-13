#include <stdio.h>
#include <string>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>
#include <image_transport/image_transport.h>
#include <boost/optional.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <camera_info_manager/camera_info_manager.h>

#define WIDTH_ID 3
#define HEIGHT_ID 4
#define FPS_ID 5

namespace zed_ros
{
class StereoCamera
{
public:
  /**
   * @brief      { stereo camera driver }
   *
   * @param[in]  resolution  The resolution
   * @param[in]  frame_rate  The frame rate
   */
  StereoCamera(int device_id, int resolution, double frame_rate) : frame_rate_(30.0)
  {
    camera_ = new cv::VideoCapture(device_id);
    cv::Mat raw;
    cv::Mat left_image;
    cv::Mat right_image;
    setResolution(resolution);
    // // this function doesn't work very well in current Opencv 2.4, so, just use ROS to control frame rate.
    // setFrameRate(frame_rate);

    ROS_INFO("Stereo Camera Set Resolution %d, width %f, height %f", resolution, camera_->get(WIDTH_ID),
             camera_->get(HEIGHT_ID));
  }

  ~StereoCamera()
  {
    delete camera_;
  }

  /**
   * @brief      Sets the resolution.
   *
   * @param[in]  type  The type
   */
  void setResolution(int type)
  {
    switch (type)
    {
      case 0:
        width_ = 4416;
        height_ = 1242;
        break;
      case 1:
        width_ = 3840;
        height_ = 1080;
        break;
      case 2:
        width_ = 2560;
        height_ = 720;
        break;
      case 3:
        width_ = 1344;
        height_ = 376;
        break;
      default:
        ROS_FATAL("Unknow resolution passed to camera: %d", type);
    }

    camera_->set(WIDTH_ID, width_);
    camera_->set(HEIGHT_ID, height_);
    // make sure that the number set are right from the hardware
    width_ = camera_->get(WIDTH_ID);
    height_ = camera_->get(HEIGHT_ID);
  }

  /**
   * @brief      Sets the frame rate.
   *
   * @param[in]  frame_rate  The frame rate
   */
  void setFrameRate(double frame_rate)
  {
    camera_->set(FPS_ID, frame_rate);
    frame_rate_ = camera_->get(FPS_ID);
  }

  /**
   * @brief      Gets the images.
   *
   * @param      left_image   The left image
   * @param      right_image  The right image
   *
   * @return     The images.
   */
  bool getImages(cv::Mat& left_image, cv::Mat& right_image)
  {
    cv::Mat raw;
    if (camera_->grab())
    {
      camera_->retrieve(raw);
      cv::Rect left_rect(0, 0, width_ / 2, height_);
      cv::Rect right_rect(width_ / 2, 0, width_ / 2, height_);
      left_image = raw(left_rect);
      right_image = raw(right_rect);
      cv::waitKey(10);
      return true;
    }
    else
    {
      return false;
    }
  }
  int width_;
  int height_;

private:
  cv::VideoCapture* camera_;

  double frame_rate_;
  bool cv_three_;
};

/**
 * @brief       the camera ros warpper class
 */
class ZedCameraROS
{
public:
  /**
   * @brief      { function_description }
   *
   * @param[in]  resolution  The resolution
   * @param[in]  frame_rate  The frame rate
   */
  ZedCameraROS()
  {
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    // get ros param
    private_nh.param("resolution", resolution_, 1);
    private_nh.param("frame_rate", frame_rate_, 30.0);
    private_nh.param("config_file_location", config_file_location_, std::string(""));
    private_nh.param("left_frame_id", left_frame_id_, std::string("left_camera"));
    private_nh.param("right_frame_id", right_frame_id_, std::string("right_camera"));
    private_nh.param("show_image", show_image_, false);
    private_nh.param("use_zed_config", use_zed_config_, true);
    private_nh.param("device_id", device_id_, 0);
    private_nh.param("rectify_image", rectify_image_, true);


    correctFramerate(resolution_, frame_rate_);

    ROS_INFO("Try to initialize the camera");
    StereoCamera zed(device_id_, resolution_, frame_rate_);
    ROS_INFO("Initialized the camera");

    WIDTH=zed.width_;
    HEIGHT=zed.height_;


    // setup publisher stuff
    image_transport::ImageTransport it(nh);
    image_transport::Publisher left_image_pub = it.advertise("left/image_raw", 1);
    image_transport::Publisher right_image_pub = it.advertise("right/image_raw", 1);

    image_transport::Publisher rec_left_image_pub = it.advertise("left/image_rect", 1);
    image_transport::Publisher rec_right_image_pub = it.advertise("right/image_rect", 1);

    ros::Publisher left_cam_info_pub = nh.advertise<sensor_msgs::CameraInfo>("left/camera_info", 1);
    ros::Publisher right_cam_info_pub = nh.advertise<sensor_msgs::CameraInfo>("right/camera_info", 1);

    sensor_msgs::CameraInfo left_info, right_info;

    ROS_INFO("Try load camera calibration files");
    if (use_zed_config_)
    {
      ROS_INFO("Loading from zed calibration files");
      // get camera info from zed
      if (!config_file_location_.empty())
      {
//        ROS_WARN("location: %s", config_file_location_.c_str());
        try
        {
          getZedCameraInfo(config_file_location_, resolution_, left_info, right_info);
          getUndistortedMaps();
        }
        catch (std::runtime_error& e)
        {
          ROS_INFO("Can't load camera info");
          ROS_ERROR("%s", e.what());
          throw e;
        }
      }
      else
      {
        ROS_FATAL("Please input zed config file path");
      }
    }
    else
    {
      ROS_INFO("Loading from ROS calibration files");
      // here we just use camera information manager to load info
      // get config from the left, right.yaml in config
      ros::NodeHandle left_nh("left");
      ros::NodeHandle right_nh("right");
      camera_info_manager::CameraInfoManager left_info_manager(left_nh, "zed/left",
                                                               "package://zed_cpu_ros/config/left.yaml");
      left_info = left_info_manager.getCameraInfo();

      camera_info_manager::CameraInfoManager right_info_manager(right_nh, "zed/right",
                                                                "package://zed_cpu_ros/config/right.yaml");
      right_info = right_info_manager.getCameraInfo();

      left_info.header.frame_id = left_frame_id_;
      right_info.header.frame_id = right_frame_id_;
    }

    ROS_INFO("Got camera calibration files");
    // loop to publish images;
    cv::Mat left_image, right_image;
    ros::Rate r(frame_rate_);

    while (nh.ok())
    {
      ros::Time now = ros::Time::now();
      if (!zed.getImages(left_image, right_image))
      {
        ROS_INFO_ONCE("Can't find camera");
      }
      else
      {
        ROS_INFO_ONCE("Success, found camera");
      }
      if (show_image_)
      {
        cv::imshow("left", left_image);
        cv::imshow("right", right_image);
      }

        if(rectify_image_){
            cv::Mat imLeftRec, imRightRec;
            cv::remap(left_image, imLeftRec, M1l, M2l, cv::INTER_LINEAR);
            cv::remap(right_image,imRightRec, M1r, M2r, cv::INTER_LINEAR);
            //cv::imshow("left_rectified", imLeftRec);
            if (rec_left_image_pub.getNumSubscribers() > 0) {
                publishImage(imLeftRec, rec_left_image_pub, "rec_left_frame", now);
            }
            if (rec_right_image_pub.getNumSubscribers() > 0) {
                publishImage(imRightRec, rec_right_image_pub, "rec_right_frame", now);
            }
        }

      if (left_image_pub.getNumSubscribers() > 0)
      {
        publishImage(left_image, left_image_pub, "left_frame", now);
      }
      if (right_image_pub.getNumSubscribers() > 0)
      {
        publishImage(right_image, right_image_pub, "right_frame", now);
      }
      if (left_cam_info_pub.getNumSubscribers() > 0)
      {
        publishCamInfo(left_cam_info_pub, left_info, now);
      }
      if (right_cam_info_pub.getNumSubscribers() > 0)
      {
        publishCamInfo(right_cam_info_pub, right_info, now);
      }
      r.sleep();
      // since the frame rate was set inside the camera, no need to do a ros sleep
    }
  }

  /**
   * @brief      Gets the camera information From Zed config.
   *
   * @param[in]  config_file         The configuration file
   * @param[in]  resolution          The resolution
   * @param[in]  left_cam_info_msg   The left camera information message
   * @param[in]  right_cam_info_msg  The right camera information message
   */
  void getZedCameraInfo(const std::string &config_file, int resolution, sensor_msgs::CameraInfo& left_info,
                        sensor_msgs::CameraInfo& right_info)
  {
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(config_file, pt);
    std::string left_str = "LEFT_CAM_";
    std::string right_str = "RIGHT_CAM_";
    std::string reso_str = "";

    switch (resolution)
    {
      case 0:
        reso_str = "2K";
        break;
      case 1:
        reso_str = "FHD";
        break;
      case 2:
        reso_str = "HD";
        break;
      case 3:
        reso_str = "VGA";
        break;
    }
    // left value
    double l_cx = pt.get<double>(left_str + reso_str + ".cx");
    ROS_WARN("l_cx: %f", l_cx);
    double l_cy = pt.get<double>(left_str + reso_str + ".cy");
    double l_fx = pt.get<double>(left_str + reso_str + ".fx");
    double l_fy = pt.get<double>(left_str + reso_str + ".fy");
    double l_k1 = pt.get<double>(left_str + reso_str + ".k1");
    double l_k2 = pt.get<double>(left_str + reso_str + ".k2");
    // right value
    double r_cx = pt.get<double>(right_str + reso_str + ".cx");
    double r_cy = pt.get<double>(right_str + reso_str + ".cy");
    double r_fx = pt.get<double>(right_str + reso_str + ".fx");
    double r_fy = pt.get<double>(right_str + reso_str + ".fy");
    double r_k1 = pt.get<double>(right_str + reso_str + ".k1");
    double r_k2 = pt.get<double>(right_str + reso_str + ".k2");

    // get baseline and convert mm to m
    boost::optional<double> baselineCheck;
    double baseline = 0.12;
    // some config files have "Baseline" instead of "BaseLine", check accordingly...
//    if (baselineCheck == pt.get_optional<double>("STEREO.BaseLine"))
//    {
//      baseline = pt.get<double>("STEREO.BaseLine") * 0.001;
//      ROS_WARN("baseline: %f", baseline);
//    }
//    else if (baselineCheck == pt.get_optional<double>("STEREO.Baseline"))
//    {
//      baseline = pt.get<double>("STEREO.Baseline") * 0.001;
//      ROS_WARN("baseline: %f", baseline);
//    }


    // get Rx and Rz
    double rx = pt.get<double>("STEREO.RX_" + reso_str);
    double rz = pt.get<double>("STEREO.RZ_" + reso_str);
    double ry = pt.get<double>("STEREO.CV_" + reso_str);

    // assume zeros, maybe not right
    double p1 = 0, p2 = 0, k3 = 0;

    left_info.height=HEIGHT;
    left_info.width=WIDTH;
    right_info.height=HEIGHT;
    right_info.width=WIDTH;

    left_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    right_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

    // distortion parameters
    // For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3).
    left_info.D.resize(5);
    left_info.D[0] = l_k1;
    left_info.D[1] = l_k2;
    left_info.D[2] = k3;
    left_info.D[3] = p1;
    left_info.D[4] = p2;

    D_l=(cv::Mat1d(1,5) << l_k1, l_k2, k3, p1, p2);
    D_r=(cv::Mat1d(1,5) << r_k1, r_k2, k3, p1, p2);

    right_info.D.resize(5);
    right_info.D[0] = r_k1;
    right_info.D[1] = r_k2;
    right_info.D[2] = k3;
    right_info.D[3] = p1;
    right_info.D[4] = p2;

    // Intrinsic camera matrix
    // 	[fx  0 cx]
    // K =  [ 0 fy cy]
    //	[ 0  0  1]
    left_info.K.fill(0.0);
    left_info.K[0] = l_fx;
    left_info.K[2] = l_cx;
    left_info.K[4] = l_fy;
    left_info.K[5] = l_cy;
    left_info.K[8] = 1.0;

    right_info.K.fill(0.0);
    right_info.K[0] = r_fx;
    right_info.K[2] = r_cx;
    right_info.K[4] = r_fy;
    right_info.K[5] = r_cy;
    right_info.K[8] = 1.0;

    K_l = (cv::Mat1d(3, 3) << l_fx, 0, l_cx, 0, l_fy, l_cy, 0, 0, 1);
    K_r = (cv::Mat1d(3, 3) << r_fx, 0, r_cx, 0, r_fy, r_cy, 0, 0, 1);

    // rectification matrix
    // Rl = R_rect, R_r = R * R_rect
    // since R is identity, Rl = Rr;
    left_info.R.fill(0.0);
    right_info.R.fill(0.0);
    cv::Mat rvec = (cv::Mat_<double>(3, 1) << rx, ry, rz);
    cv::Mat rmat(3, 3, CV_64F);
    cv::Rodrigues(rvec, rmat);
    int id = 0;
    cv::MatIterator_<double> it, end;
    for (it = rmat.begin<double>(); it != rmat.end<double>(); ++it, id++)
    {
      left_info.R[id] = *it;
      right_info.R[id] = *it;
    }

    R_r = rmat;
    R_l = rmat;

    // Projection/camera matrix
    //     [fx'  0  cx' Tx]
    // P = [ 0  fy' cy' Ty]
    //     [ 0   0   1   0]
    left_info.P.fill(0.0);
    left_info.P[0] = l_fx;
    left_info.P[2] = l_cx;
    left_info.P[5] = l_fy;
    left_info.P[6] = l_cy;
    left_info.P[10] = 1.0;

    right_info.P.fill(0.0);
    right_info.P[0] = r_fx;
    right_info.P[2] = r_cx;
    right_info.P[3] = (-1 * l_fx * baseline);
    right_info.P[5] = r_fy;
    right_info.P[6] = r_cy;
    right_info.P[10] = 1.0;

    P_l = (cv::Mat1d(3, 4) << l_fx, 0, l_cx, 0, 0, l_fy, l_cy, 0, 0, 0, 1, 0);
    P_r = (cv::Mat1d(3, 4) << r_fx, 0, r_cx, -1 * l_fx * baseline, 0, r_fy, r_cy, 0, 0, 0, 1, 0);

    left_info.width = right_info.width = width_;
    left_info.height = right_info.height = height_;

    left_info.header.frame_id = left_frame_id_;
    right_info.header.frame_id = right_frame_id_;
  }

    void getUndistortedMaps() {

        if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() ||
            D_r.empty() ||
            WIDTH == 0 || HEIGHT == 0)
        {
            std::cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << "/n";
            return ;

        }
        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(WIDTH/2,HEIGHT),CV_32F,M1l,M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(WIDTH/2,HEIGHT),CV_32F,M1r,M2r);
    }

  /**
   * @brief      { publish camera info }
   *
   * @param[in]  pub_cam_info  The pub camera information
   * @param[in]  cam_info_msg  The camera information message
   * @param[in]  now           The now
   */
  void publishCamInfo(const ros::Publisher& pub_cam_info, sensor_msgs::CameraInfo& cam_info_msg, ros::Time now)
  {
    cam_info_msg.header.stamp = now;
    pub_cam_info.publish(cam_info_msg);
  }

  /**
   * @brief      { publish image }
   *
   * @param[in]  img           The image
   * @param      img_pub       The image pub
   * @param[in]  img_frame_id  The image frame identifier
   * @param[in]  t             { parameter_description }
   * @param[in]  encoding      image_transport encoding
   */
  void publishImage(const cv::Mat& img, image_transport::Publisher& img_pub, const std::string& img_frame_id,
                    ros::Time t)
  {
    cv_bridge::CvImage cv_image;
    // or it seems like CV mat is passing by reference?
    cv_image.image = img;
    cv_image.encoding = sensor_msgs::image_encodings::BGR8;
    cv_image.header.frame_id = img_frame_id;
    cv_image.header.stamp = t;
    img_pub.publish(cv_image.toImageMsg());
  }
  /**
   * @brief      Correct frame rate according to resolution
   *
   * @param[in]  resolution          The resolution
   * @param      frame_rate   			 The camera frame rate
   */
  void correctFramerate(int resolution, double& frame_rate)
  {
    double max_frame_rate;
    std::string reso_str;
    switch (resolution)
    {
      case 0:
        max_frame_rate = 15;
        reso_str = "2K";
        break;
      case 1:
        max_frame_rate = 30;
        reso_str = "FHD";
        break;
      case 2:
        max_frame_rate = 60;
        reso_str = "HD";
        break;
      case 3:
        max_frame_rate = 100;
        reso_str = "VGA";
        break;
      default:
        ROS_FATAL("Unknown resolution passed");
        return;
    }
    if (frame_rate > max_frame_rate)
      ROS_WARN("frame_rate(%fHz) too high for resolution(%s), downgraded to %fHz", frame_rate, reso_str.c_str(),
               max_frame_rate);
    frame_rate = max_frame_rate;
  }

private:
  int device_id_, resolution_;
  double frame_rate_;
  bool show_image_, use_zed_config_;
  double width_, height_;
  std::string left_frame_id_, right_frame_id_;
  std::string config_file_location_;
  int WIDTH, HEIGHT;
  cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
    bool rectify_image_;
    std::string undistort_config_file_location_;
    cv::Mat M1l, M2l, M1r,M2r;
};
}

int main(int argc, char** argv)
{
  try
  {
    ros::init(argc, argv, "zed_camera");
    zed_ros::ZedCameraROS zed_ros;
    return EXIT_SUCCESS;
  }
  catch (std::runtime_error& e)
  {
    ros::shutdown();
    return EXIT_FAILURE;
  }
}
