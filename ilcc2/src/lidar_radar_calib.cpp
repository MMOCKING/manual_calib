#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <dynamic_reconfigure/server.h>
#include <ilcc2/LocatorConfig.h>
#include <tf/tf.h>


using namespace std;
using namespace message_filters;

Eigen::Matrix3d R_lidar_to_radar = Eigen::Matrix3d::Identity();
Eigen::Vector3d p_lidar_to_radar = Eigen::Vector3d::Zero();
ros::Publisher transformed_cloud_pub;
std::string radar_frame_id;


void callback_updateExtrinsic(ilcc2::LocatorConfig& config, uint32_t level) {
  double d2r = M_PI / 180.;
  tf::Quaternion quat;
  quat.setRPY(config.roll * d2r, config.pitch * d2r, config.yaw * d2r);
  Eigen::Quaterniond eigen_quat(quat.w(), quat.x(), quat.y(), quat.z());

  
  R_lidar_to_radar = eigen_quat.toRotationMatrix();
  p_lidar_to_radar << config.pos_x, config.pos_y, config.pos_z;

  ROS_INFO("Reconfigure Request (rpyxyz): %.2f %.2f %.2f %.2f %.2f %.2f",
           config.roll, config.pitch, config.yaw, config.pos_x, config.pos_y,
           config.pos_z);
}


void callback_lidar(const sensor_msgs::PointCloud2ConstPtr& lidar_pc)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*lidar_pc, *lidar_cloud);

  // Convert R_lidar_to_radar and p_lidar_to_radar to Eigen::Affine3f objects
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << p_lidar_to_radar(0), p_lidar_to_radar(1), p_lidar_to_radar(2);
  transform.linear() = R_lidar_to_radar.cast<float>();

  // transform lidar cloud to radar frame
  pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_cloud_radar(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(*lidar_cloud, *lidar_cloud_radar, transform);

  // 转换 pcl::PointCloud 到 ROS 消息
  sensor_msgs::PointCloud2 lidar_cloud_radar_msg;
  pcl::toROSMsg(*lidar_cloud_radar, lidar_cloud_radar_msg);
  lidar_cloud_radar_msg.header.frame_id = radar_frame_id;
  
  lidar_cloud_radar_msg.header.stamp = lidar_pc->header.stamp;
  lidar_cloud_radar_msg.header.seq = lidar_pc->header.seq;

  // 发布变换后的点云消息
  transformed_cloud_pub.publish(lidar_cloud_radar_msg);
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_radar_calib");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    std::string lidar_topic;
    nh_private.param<std::string>("image_topic", lidar_topic, "/rslidar_points");
    nh_private.param<std::string>("radar_frame_id", radar_frame_id, "car");
    transformed_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/transformed_lidar_cloud", 1);

    // extrinsic config
    dynamic_reconfigure::Server<ilcc2::LocatorConfig> server;
    dynamic_reconfigure::Server<ilcc2::LocatorConfig>::CallbackType f;
    f = boost::bind(&callback_updateExtrinsic, _1, _2);
    server.setCallback(f);

    // subscribe lidar point cloud
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub(nh, lidar_topic, 1);
    lidar_sub.registerCallback(callback_lidar);


    while (ros::ok()) {
      ros::spin();
    }

    cout << "==========================\n\n";
    cout << "R lidar_to_radar:\n" << R_lidar_to_radar << "\n";
    cout << "P lidar_to_radar:\n" << p_lidar_to_radar.transpose() << "\n";
    cout << "\n==========================\n";

    return 0;
}