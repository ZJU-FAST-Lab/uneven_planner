#include <iostream>
#include <ros/ros.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include <gazebo_msgs/SetModelState.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;
 
string map_file;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloudMap_3d;
pcl::PointCloud<pcl::PointXY>::Ptr cloudMap_2d;

pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree_3d;
pcl::KdTreeFLANN<pcl::PointXY> kd_tree_2d;

gazebo_msgs::SetModelState command;

geometry_msgs::Point point;
sensor_msgs::PointCloud2 cloud_msg;

ros::ServiceClient client;
ros::Subscriber lisener;
ros::Publisher cloud_pub;
ros::Timer vis_timer;

void callback(const geometry_msgs::Point::ConstPtr &point)
{

// 2d search
  vector<int> Idxs;
  vector<float> SquaredDists;
  pcl::PointXY pxy;
  pxy.x = point->x;
  pxy.y = point->y;

  kd_tree_2d.nearestKSearch(pxy, 1, Idxs, SquaredDists);

  // 3d find
  pcl::PointXYZ pxyz;
  pxyz = cloudMap_3d->points[Idxs[0]];
  
  command.request.model_state.model_name = "racebot";
  command.request.model_state.pose.position.x = pxyz.x;
  command.request.model_state.pose.position.y = pxyz.y;
  command.request.model_state.pose.position.z = pxyz.z + 0.1;
  command.request.model_state.pose.orientation.w = cos(point->z/2.0);
  command.request.model_state.pose.orientation.x = 0;
  command.request.model_state.pose.orientation.y = 0;
  command.request.model_state.pose.orientation.z = sin(point->z/2.0);
  command.request.model_state.twist.linear.x = 0.0;
  command.request.model_state.twist.linear.y = 0.0;
  command.request.model_state.twist.linear.z = 0.0;
  command.request.model_state.twist.angular.x = 0.0;
  command.request.model_state.twist.angular.y = 0.0;
  command.request.model_state.twist.angular.z = 0.0;
  command.request.model_state.reference_frame = "world";
  client.call(command);
}

void visCallback(const ros::TimerEvent& /*event*/)
{
  cloud_pub.publish(cloud_msg);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "searchForSetPoint");
  ros::NodeHandle nh("~");

  lisener = nh.subscribe<geometry_msgs::Point>("/set_model_location",1,callback);
  cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_map", 1);
  client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

  cloudMap_2d.reset(new pcl::PointCloud<pcl::PointXY>());
  cloudMap_3d.reset(new pcl::PointCloud<pcl::PointXYZ>());

  nh.getParam("map_file", map_file);

  pcl::PCDReader reader;
  reader.read<pcl::PointXYZ>(map_file, *cloudMap_3d);
  
  for (size_t i=0; i<cloudMap_3d->points.size(); i++)
  {
    pcl::PointXY p;
    p.x = cloudMap_3d->points[i].x;
    p.y = cloudMap_3d->points[i].y;
    cloudMap_2d->points.emplace_back(p);
  }

  cloudMap_3d->width = cloudMap_3d->points.size();
  cloudMap_3d->height = 1;
  cloudMap_3d->is_dense = true;
  cloudMap_3d->header.frame_id = "world";

  cloudMap_2d->width = cloudMap_2d->points.size();
  cloudMap_2d->height = 1;
  cloudMap_2d->is_dense = true;
  cloudMap_2d->header.frame_id = "world";

  pcl::toROSMsg(*cloudMap_3d, cloud_msg);

  kd_tree_3d.setInputCloud(cloudMap_3d);
  kd_tree_2d.setInputCloud(cloudMap_2d);

  vis_timer = nh.createTimer(ros::Duration(1.0), visCallback);

  ROS_INFO("gazebo simulator ready.");
  ros::spin();

  return 0;
}
