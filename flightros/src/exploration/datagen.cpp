
// ros
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

// flightlib
#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>

#include "flightlib/bridges/unity_bridge.hpp"
#include "flightlib/bridges/unity_message_types.hpp"
#include "flightlib/common/quad_state.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/objects/quadrotor.hpp"
#include "flightlib/sensors/rgb_camera.hpp"

using namespace flightlib;

void depthToPointCloud(const cv::Mat& depth_map, const cv::Mat& K,
                       std::vector<Eigen::Vector3d>& cloud) {
  int width = depth_map.cols;
  int height = depth_map.rows;

  float fx = K.at<float>(0, 0);
  float fy = K.at<float>(1, 1);
  float cx = K.at<float>(0, 2);
  float cy = K.at<float>(1, 2);

  cloud.resize(width * height);

  for (int v = 0; v < height; ++v) {
    for (int u = 0; u < width; ++u) {
      float Z = depth_map.at<float>(v, u);
      if (Z > 0) {
        float X = (u - cx) * Z / fx;
        float Y = (v - cy) * Z / fy;
        cloud[v * width + u] = Eigen::Vector3d(X, Z, -Y); // flip axes to body frame
      }
    }
  }
}

void addCamera(std::shared_ptr<Quadrotor>& quad,
               std::shared_ptr<RGBCamera>& camera, Vector<3> B_r_BC,
               Matrix<3, 3> R_BC) {
  camera->setFOV(90);
  camera->setWidth(160);
  camera->setHeight(90);
  camera->setRelPose(B_r_BC, R_BC);
  camera->setPostProcesscing(
    std::vector<bool>{true, true, true});  // depth, segmentation, optical flow
  quad->addRGBCamera(camera);
}

int main(int argc, char* argv[]) {
  // initialize ROS
  ros::init(argc, argv, "camera_example");
  ros::NodeHandle nh("");
  ros::NodeHandle pnh("~");
  ros::Rate(50.0);

  // publisher
  image_transport::Publisher depth_pub1;
  image_transport::Publisher depth_pub2;
  image_transport::Publisher depth_pub3;
  image_transport::Publisher depth_pub4;
  image_transport::Publisher rgb_pub;
  ros::Publisher octomap_pub = nh.advertise<octomap_msgs::Octomap>("occupancy",1);

  // unity quadrotor
  std::shared_ptr<Quadrotor> quad_ptr = std::make_shared<Quadrotor>();
  // define quadsize scale (for unity visualization only)
  Vector<3> quad_size(0.5, 0.5, 0.5);
  quad_ptr->setSize(quad_size);
  QuadState quad_state;

  // camera
  std::shared_ptr<RGBCamera> rgb_camera1 = std::make_shared<RGBCamera>();
  std::shared_ptr<RGBCamera> rgb_camera2 = std::make_shared<RGBCamera>();
  std::shared_ptr<RGBCamera> rgb_camera3 = std::make_shared<RGBCamera>();
  std::shared_ptr<RGBCamera> rgb_camera4 = std::make_shared<RGBCamera>();

  // Flightmare(Unity3D)
  std::shared_ptr<UnityBridge> unity_bridge_ptr = UnityBridge::getInstance();
  SceneID scene_id{UnityScene::WAREHOUSE};
  bool unity_ready{false};

  // initialize publishers
  image_transport::ImageTransport it(pnh);
  depth_pub1 = it.advertise("/depth1", 1);
//   depth_pub2 = it.advertise("/depth2", 1);
//   depth_pub3 = it.advertise("/depth3", 1);
//   depth_pub4 = it.advertise("/depth4", 1);
  rgb_pub = it.advertise("/rgb", 1);

  // add cameras
  float fx = 90.0 / 2.0;
  float fy = 160.0 / 2.0;
  cv::Mat K = (cv::Mat_<float>(3, 3) << fx, 0.0, 160.0 / 2.0, 0.0, fy,
                90.0 / 2.0, 0.0, 0.0, 1.0);
  Vector<3> T_body_cam(0.0, 0.0, 0.3);
  Matrix<3, 3> R_body_cam1 = Quaternion(1.0, 0.0, 0.0, 0.0).toRotationMatrix();
  Matrix<3, 3> R_body_cam2 = Quaternion(0.707, 0.0, 0.0, 0.707).toRotationMatrix();
  Matrix<3, 3> R_body_cam3 = Quaternion(0.0, 0.0, 0.0, 1.0).toRotationMatrix();
  Matrix<3, 3> R_body_cam4 = Quaternion(-0.707, 0.0, 0.0, 0.707).toRotationMatrix();
  
  addCamera(quad_ptr, rgb_camera1, T_body_cam, R_body_cam1);
  addCamera(quad_ptr, rgb_camera2, T_body_cam, R_body_cam2);
  addCamera(quad_ptr, rgb_camera3, T_body_cam, R_body_cam3);
  addCamera(quad_ptr, rgb_camera4, T_body_cam, R_body_cam4);

  // initialization
  quad_state.setZero();
  quad_ptr->reset(quad_state);

  // connect unity
  unity_bridge_ptr->addQuadrotor(quad_ptr);
  unity_ready = unity_bridge_ptr->connectUnity(scene_id);

  // occupancy map
  octomap::OcTree tree(0.2);

  FrameID frame_id = 0;
  while (ros::ok() && unity_ready) {
    quad_state.x[QS::POSY] += 0.2;
    Eigen::Vector3f T_world_body = quad_state.p;

    quad_ptr->setState(quad_state);

    unity_bridge_ptr->getRender(frame_id);
    unity_bridge_ptr->handleOutput();

    cv::Mat depth_map1, depth_map2, depth_map3, depth_map4;
    cv::Mat rgb1;

    ros::Time timestamp = ros::Time::now();

    rgb_camera1->getDepthMap(depth_map1);
    rgb_camera2->getDepthMap(depth_map2);
    rgb_camera3->getDepthMap(depth_map3);
    rgb_camera4->getDepthMap(depth_map4);

    rgb_camera1->getRGBImage(rgb1);

    // convert depth maps to point clouds
    std::vector<Eigen::Vector3d> cloud1, cloud2, cloud3, cloud4;
    depthToPointCloud(depth_map1, K, cloud1);
    depthToPointCloud(depth_map2, K, cloud2);
    depthToPointCloud(depth_map3, K, cloud3);
    depthToPointCloud(depth_map4, K, cloud4);

    // octomap
    for (const auto& point : cloud1)
    {
        auto point_tf = R_body_cam1 * point.cast<float>() + T_body_cam + T_world_body;
        tree.updateNode(octomap::point3d(point_tf[0], point_tf[1], point_tf[2]), true);
    }
    for (const auto& point : cloud2)
    {
        auto point_tf = R_body_cam2 * point.cast<float>() + T_body_cam + T_world_body;;
        tree.updateNode(octomap::point3d(point_tf[0], point_tf[1], point_tf[2]), true);
    }
    for (const auto& point : cloud3)
    {
        auto point_tf = R_body_cam3 * point.cast<float>() + T_body_cam + T_world_body;;
        tree.updateNode(octomap::point3d(point_tf[0], point_tf[1], point_tf[2]), true);
    }
    for (const auto& point : cloud4)
    {
        auto point_tf = R_body_cam4 * point.cast<float>() + T_body_cam + T_world_body;;
        tree.updateNode(octomap::point3d(point_tf[0], point_tf[1], point_tf[2]), true);
    }

    // publish
    sensor_msgs::ImagePtr depth_msg =
      cv_bridge::CvImage(std_msgs::Header(), "32FC1", depth_map1).toImageMsg();
    depth_msg->header.stamp = timestamp;
    depth_pub1.publish(depth_msg);
    sensor_msgs::ImagePtr rgb_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgb1).toImageMsg();
    rgb_msg->header.stamp = timestamp;
    rgb_pub.publish(rgb_msg);
    // depth_msg =
    //   cv_bridge::CvImage(std_msgs::Header(), "32FC1", depth_map2).toImageMsg();
    // depth_msg->header.stamp = timestamp;
    // depth_pub2.publish(depth_msg);
    // depth_msg =
    //   cv_bridge::CvImage(std_msgs::Header(), "32FC1", depth_map3).toImageMsg();
    // depth_msg->header.stamp = timestamp;
    // depth_pub3.publish(depth_msg);
    // depth_msg =
    //   cv_bridge::CvImage(std_msgs::Header(), "32FC1", depth_map4).toImageMsg();
    // depth_msg->header.stamp = timestamp;
    // depth_pub4.publish(depth_msg);

    octomap_msgs::Octomap bmap_msg;
    octomap_msgs::binaryMapToMsg(tree, bmap_msg);
    bmap_msg.header.frame_id = "map";
    bmap_msg.header.stamp = timestamp;
    octomap_pub.publish(bmap_msg);

    // save octomap
    // tree.writeBinary("/home/odexter/fm_logs/" + std::to_string(frame_id) + ".bt");

    frame_id += 1;
  }

  return 0;
}
