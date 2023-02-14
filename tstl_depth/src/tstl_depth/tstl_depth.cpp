#include "tstl_depth/tstl_depth.h"

static bool compare_point(const yolov3_trt_ros::BoundingBox& box1, const yolov3_trt_ros::BoundingBox& box2)
{
  return box1.xmin < box2.xmin;
}

namespace xycar
{
DepthEstimator::DepthEstimator()
{
  std::string config_calibration_path;
  nh_.getParam("config_calibration_path", config_calibration_path);
  YAML::Node config_calibration = YAML::LoadFile(config_calibration_path);
  setParams(config_calibration);
  image_sub_ = nh_.subscribe("/usb_cam/image_raw/", 1, &DepthEstimator::imageCallback, this);
  detection_sub_ = nh_.subscribe("/yolov3_trt_ros/detections", 1, &DepthEstimator::detectionCallback, this);
  lidar_sub_ = nh_.subscribe("/scan", 1, &DepthEstimator::lidarCallback, this);
}

void DepthEstimator::setParams(const YAML::Node &config_calibration) {
  IMAGE_WIDTH = config_calibration["image_size"]["width"].as<int>();
  IMAGE_HEIGHT = config_calibration["image_size"]["height"].as<int>();
  FISH_EYE = config_calibration["fish_eye"].as<bool>();

  image_size.width = IMAGE_WIDTH;
  image_size.height = IMAGE_HEIGHT;

  delta = config_calibration["delta"].as<double>();

  camera_matrix_ = config_calibration["camera_matrix"].as<std::vector<double>>();
  dist_coeffs_ = config_calibration["dist_coeffs"].as<std::vector<double>>();
  homography_matrix_ = config_calibration["homography"].as<std::vector<double>>();
  rvec_ = config_calibration["rvec"].as<std::vector<double>>();
  tvec_ = config_calibration["tvec"].as<std::vector<double>>();

  camera_matrix = cv::Mat(3, 3, CV_64F, camera_matrix_.data()).clone();
  dist_coeffs = cv::Mat(dist_coeffs_.size(), 1, CV_64F, dist_coeffs_.data()).clone();
  homography_matrix = cv::Mat(3, 3, CV_64F, homography_matrix_.data()).clone();
  rvec = cv::Mat(3, 1, CV_64F, rvec_.data()).clone();
  tvec = cv::Mat(3, 1, CV_64F, tvec_.data()).clone();

  if (FISH_EYE == true)
  {
    cv::fisheye::estimateNewCameraMatrixForUndistortRectify(camera_matrix, dist_coeffs, image_size, cv::Mat(),
                                                            new_camera_matrix, 0.0);
    cv::fisheye::initUndistortRectifyMap(camera_matrix, dist_coeffs, cv::Mat(), new_camera_matrix, image_size, CV_32FC1,
                                        map_x, map_y);
  }
  else
  {
    new_camera_matrix = cv::getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, image_size, 0.0).clone();
    cv::initUndistortRectifyMap(camera_matrix, dist_coeffs, cv::Mat(), new_camera_matrix, image_size, CV_32FC1, map_x,
                                map_y);
  }

  //findchessboard_flags = cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK;
  //calibration_fisheye_flags = cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC + cv::fisheye::CALIB_CHECK_COND + cv::fisheye::CALIB_FIX_SKEW;
  /*
  rvec = (cv::Mat_<double>(3, 1) << -1.1701, -1.1627, -1.2308);
  tvec = (cv::Mat_<double>(3, 1) << -0.5292, 16.7061, 32.5574);
  */
}

DepthEstimator::~DepthEstimator()
{ }

void DepthEstimator::run()
{
  while (detection_sub_.getNumPublishers() == 0)
  {}

  //cv::namedWindow("undist", cv::WINDOW_NORMAL);

  while (ros::ok())
  {
    ros::spinOnce();
    if (frame_.empty())
    {
      continue;
    }
    cv::Mat undist;
    cv::remap(frame_, undist, map_x, map_y, cv::INTER_LINEAR);

    // std::cout << "new_camera_matrix : \n" << new_camera_matrix << std::endl;
    std::vector<cv::Point3f> lidar_xyz;
    lidar_xyz.reserve(505);

    for (int i = 0; i < 505; ++i)
    {
      float dist = lidar_data.ranges[i];
      
      if (dist < lidar_data.range_min || dist > 2.0)
      {
        // std::cout << dist << std::endl;
        continue;
      }

      dist *= 100;
      float theta = lidar_data.angle_min + i * lidar_data.angle_increment ;  // (rad)

      /*
      float div_delta = delta / 126;
      if (0 <= i && i <= 252)
      {
        delta = delta - (div_delta * i);
      }
      else if (253 <= i && i <= 505)
      {
        delta = -delta + (div_delta * (i - 253));
      }
      */
      
      float x = std::cos(theta) * dist;
      float y = std::sin(theta) * dist;

      //lidar_xyz.push_back(cv::Point3f(-(x - 4.5) , y, -(6.5 + z)));
      lidar_xyz.push_back(cv::Point3f(x, y, 0));
    }

    std::vector<cv::Point2f> proj_points;
    // lidar calibration
    cv::projectPoints(lidar_xyz, rvec, tvec, camera_matrix, dist_coeffs, proj_points);

    for(auto point : proj_points)
    {
      cv::Point2i point_i(point);
      if(point_i.x < 0 || point_i.x >= 640 || point_i.y < 0 || point_i.y >= 480)
      {
        continue;
      }
      //cv::circle(undist, point_i, 1, cv::Scalar(0, 255, 0));
      cv::drawMarker(undist, point_i, cv::Scalar(0,255,0), cv::MARKER_DIAMOND, 5, 1);
    }

    std::vector<std::tuple<float, float, int>> xy_sum(bboxes.size());

    for (int i = 0; i < proj_points.size(); ++i)
    {
      for (int j = 0; j < bboxes.size(); ++j)
      {
        if (bboxes[j].xmin <= proj_points[i].x && 
            proj_points[i].x < bboxes[j].xmax && 
            bboxes[j].ymin <= proj_points[i].y && 
            proj_points[i].y < bboxes[j].ymax)
        {
          std::get<0>(xy_sum[j]) += lidar_xyz[i].x;
          std::get<1>(xy_sum[j]) += lidar_xyz[i].y;
          ++std::get<2>(xy_sum[j]);
          break;
        }
      }
    }

    for (int i = 0; i < bboxes.size(); ++i)
    {
      if (std::get<2>(xy_sum[i]) == 0)
      {
        std::cout << "no corresponding points" << std::endl;
        continue;
      }

      float mean_x = std::get<0>(xy_sum[i]) / static_cast<float>(std::get<2>(xy_sum[i]));
      float mean_y = std::get<1>(xy_sum[i]) / static_cast<float>(std::get<2>(xy_sum[i]));
      
      std::cout << "id: " << bboxes[i].id << " / location : " << mean_x << ", " << mean_y << std::endl;
    }
    std::cout << std::endl;

    
    // // capture image
    // /*
    // cv::imshow("frame", frame_);
    // cv::imwrite("/home/nvidia/contest_image/data_" + std::to_string(ros::Time::now().toSec()) + ".png" , frame_);
    // */

    cv::imshow("undist", undist);
    // cv::waitKey(0);

    while (getchar() != '\n')
    { }

  }
}

void DepthEstimator::imageCallback(const sensor_msgs::Image& msg)
{
  cv::Mat src = cv::Mat(msg.height, msg.width, CV_8UC3, const_cast<uint8_t*>(&msg.data[0]), msg.step);
  cv::cvtColor(src, frame_, cv::COLOR_RGB2BGR);
}

void DepthEstimator::detectionCallback(const yolov3_trt_ros::BoundingBoxes& msg) 
{
  bboxes.clear();
  for (auto& box : msg.bounding_boxes) {
    yolov3_trt_ros::BoundingBox cur_box;
    cur_box.probability = box.probability;
    cur_box.id = box.id;
    cur_box.xmin = box.xmin * 640 / 416;
    cur_box.ymin = box.ymin * 480 / 416;
    cur_box.xmax = box.xmax * 640 / 416;
    cur_box.ymax = box.ymax * 480 / 416;
    
    if (cur_box.xmin < 0 || cur_box.xmin > 639 ||
        cur_box.ymin < 0 || cur_box.ymin > 479 ||
        cur_box.xmax < 0 || cur_box.xmax > 639 ||
        cur_box.ymax < 0 || cur_box.ymax > 479)
    {
      continue;
    }
    
    bboxes.push_back(cur_box);
  }

  std::sort(bboxes.begin(), bboxes.end(), compare_point);
}

void DepthEstimator::lidarCallback(const sensor_msgs::LaserScan& msg)
{
  lidar_data = msg;
}

} /* namespace xycar */
