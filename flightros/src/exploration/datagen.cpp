#include <algorithm>
#include <fstream>

// ros
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>

// flightlib
#include "flightlib/bridges/unity_bridge.hpp"
#include "flightlib/bridges/unity_message_types.hpp"
#include "flightlib/common/quad_state.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/objects/quadrotor.hpp"
#include "flightlib/sensors/rgb_camera.hpp"

using namespace flightlib;

void depthToPointCloud(const cv::Mat& depth_map,
                       const cv::Mat& K,
                       octomap::Pointcloud& cloud,
                       const Eigen::Matrix3f& R_body_cam,
                       const Eigen::Vector3f& T_body_cam,
                       const Eigen::Vector3f& T_world_body)
{
  int width = depth_map.cols;
  int height = depth_map.rows;

  float fx = K.at<float>(0, 0);
  float fy = K.at<float>(1, 1);
  float cx = K.at<float>(0, 2);
  float cy = K.at<float>(1, 2);

  for (int v = 0; v < height; ++v) {
    for (int u = 0; u < width; ++u) {
      float Z = depth_map.at<float>(v, u);
      if (Z > 0 && Z < 20) {
        float X = (u - cx) * Z / fx;
        float Y = (v - cy) * Z / fy;
        // transform to world frame
        Eigen::Vector3f point(X, Z, -Y);
        Eigen::Vector3f point_tf = R_body_cam * point.cast<float>() + T_body_cam + T_world_body;
        cloud.push_back(octomap::point3d(point_tf[0], point_tf[1], point_tf[2])); // flip axes to body frame
      }
    }
  }
}

void addCamera(std::shared_ptr<Quadrotor>& quad,
               std::shared_ptr<RGBCamera>& camera, Vector<3> B_r_BC,
               Matrix<3, 3> R_BC)
{
  camera->setFOV(90);
  camera->setWidth(160);
  camera->setHeight(90);
  camera->setRelPose(B_r_BC, R_BC);
  camera->setPostProcesscing(std::vector<bool>{true, false, false});  // depth, segmentation, optical flow
  quad->addRGBCamera(camera);
}

void octreeToVoxelGrid(const octomap::OcTree& octree, std::vector<std::vector<std::vector<int>>>& voxel_grid)
{
    // loop over all nodes in the octree
    int resolution = 0.2;
    int dim = 100 / resolution;
    for (octomap::OcTree::leaf_iterator it = octree.begin_leafs(), end = octree.end_leafs(); it != end; ++it) {
        // Get the coordinate of this leaf node
        octomap::point3d coord = it.getCoordinate();
        std::cout << "Coordinate: (" << coord.x() << ", " << coord.y() << ", " << coord.z() << ")" << std::endl;
    
        int x = (coord.x() / resolution) + (dim / 2);
        int y = (coord.y() / resolution) + (dim / 2);
        int z = (coord.z() / resolution) + (dim / 2);
        std::cout << x << " " << y << " " << z << std::endl;

        // set occupancy
        if (octree.isNodeOccupied(*it)) {
            voxel_grid[x][y][z] = 2;     // occupied cell
        } else {
            voxel_grid[x][y][z] = 1;    // free cell
        }
    }
}

int main(int argc, char* argv[])
{
  // initialize ROS
  ros::init(argc, argv, "exploration_datagen");
  ros::NodeHandle nh("");
  ros::NodeHandle pnh("~");
  ros::Rate(50.0);

  // publisher
  image_transport::Publisher depth_pub1;
  image_transport::Publisher depth_pub2;
  image_transport::Publisher depth_pub3;
  image_transport::Publisher depth_pub4;
  image_transport::Publisher rgb_pub;
  ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose_topic", 10);
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
  SceneID scene_id{UnityScene::PLANE};
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

  // add objects
  std::shared_ptr<StaticObject> cylinder1 = std::make_shared<StaticObject>("cylinder1", "cylinder");
  cylinder1->setPosition(Eigen::Vector3f(-5, 5, 0));
  cylinder1->setSize(Eigen::Vector3f(2.0, 2.0, 10.0));
  unity_bridge_ptr->addStaticObject(cylinder1);

  std::shared_ptr<StaticObject> cylinder2 = std::make_shared<StaticObject>("cylinder2", "cylinder");
  cylinder2->setPosition(Eigen::Vector3f(5, 5, 0));
  cylinder2->setSize(Eigen::Vector3f(2.0, 2.0, 10.0));
  unity_bridge_ptr->addStaticObject(cylinder2);

  // initialization
  quad_state.setZero();
  quad_ptr->reset(quad_state);
  quad_state.x[QS::POSZ] = 1.0;

  // connect unity
  unity_bridge_ptr->addQuadrotor(quad_ptr);

  unity_ready = unity_bridge_ptr->connectUnity(scene_id);

  // occupancy map
  octomap::OcTree tree(0.2);

  // sleep for a bit
  ros::Duration(1.0).sleep();

  // run loop
  FrameID frame_id = 0;
  while (ros::ok() && unity_ready) {
    quad_state.x[QS::POSY] += 0.1;
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
    octomap::Pointcloud cloud;
    depthToPointCloud(depth_map1, K, cloud, R_body_cam1, T_body_cam, T_world_body);
    depthToPointCloud(depth_map2, K, cloud, R_body_cam2, T_body_cam, T_world_body);
    depthToPointCloud(depth_map3, K, cloud, R_body_cam3, T_body_cam, T_world_body);
    depthToPointCloud(depth_map4, K, cloud, R_body_cam4, T_body_cam, T_world_body);

    // octomap
    octomap::point3d octopose = octomap::point3d(T_world_body[0], T_world_body[1], T_world_body[2] + 0.3); // this is an excellent pun
    tree.insertPointCloud(cloud, octopose);
    // tree.updateInnerOccupancy();

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

    // pose
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = timestamp;
    pose_msg.header.frame_id = "map"; // set the frame ID for the pose
    pose_msg.pose.position.x = quad_state.x[QS::POSX];
    pose_msg.pose.position.y = quad_state.x[QS::POSY];
    pose_msg.pose.position.z = quad_state.x[QS::POSZ] + 0.3; // offset to sensor pose
    pose_pub.publish(pose_msg);

    // save octomap
    int dim = 500;
    // std::vector<std::vector<std::vector<int>>> voxel_grid(dim, std::vector<std::vector<int>>(dim, std::vector<int>(dim, 0)));
    // octreeToVoxelGrid(tree, voxel_grid);
    std::vector<std::vector<std::vector<int>>> voxel_compressed(dim, std::vector<std::vector<int>>(dim, std::vector<int>(4, -1)));
    // loop over all nodes in the octree
    float resolution = 0.2;
    for (octomap::OcTree::leaf_iterator it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it)
    {
        // get the coordinate of this leaf node
        octomap::point3d coord = it.getCoordinate();
        
        // std::cout << "Coordinate: (" << coord.x() << ", " << coord.y() << ", " << coord.z() << ")" << std::endl;
    
        int x = (coord.x() / resolution) + (dim / 2);
        int y = (coord.y() / resolution) + (dim / 2);
        int z = (coord.z() / resolution) + (dim / 2);

        // compute z_max, z_min, z_avg, count for each x and y
        if (voxel_compressed[x][y][3] == -1) 
        {
          voxel_compressed[x][y] = {0, 0, 0, 0};
          if (tree.isNodeOccupied(*it)) {
            voxel_compressed[x][y][0] = coord.z();
            voxel_compressed[x][y][1] = coord.z();
            voxel_compressed[x][y][2] = coord.z();
            voxel_compressed[x][y][3]++;
          }
        }
        else if (tree.isNodeOccupied(*it)) 
        {
          int count = voxel_compressed[x][y][3];
          voxel_compressed[x][y][0] = std::max(static_cast<float>(voxel_compressed[x][y][0]), coord.z());
          voxel_compressed[x][y][1] = std::min(static_cast<float>(voxel_compressed[x][y][1]), coord.z());
          voxel_compressed[x][y][2] = (voxel_compressed[x][y][2] * count + coord.z() ) / (count + 1);
          voxel_compressed[x][y][3]++;
        }
    }

    // write to file
    std::string file_idx = std::string(6 - std::min(6, static_cast<int>(std::to_string(frame_id).length())), '0') + std::to_string(frame_id);
    std::ofstream outFile("/home/odexter/fm_logs/" + file_idx + ".csv");
    for (int i = 0; i < dim; i++) {
      for (int j = 0; j < dim; j++) {
        outFile << i << "," << j << "," << voxel_compressed[i][j][0] << "," << voxel_compressed[i][j][1] << "," << voxel_compressed[i][j][2] << "\n";
      }
      outFile << std::endl;
    }
    outFile.close();
    
    frame_id += 1;
  }

  return 0;
}
