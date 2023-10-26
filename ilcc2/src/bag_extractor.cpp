#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <pcl/io/pcd_io.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <rosbag/view.h> 

// /Target_Radar_1
// /Target_Radar_1_extreme
// /camera/color/image_raw
// /camera/depth/image_raw
// /rslidar_points
// /usb_cam/image_raw/compressed


int main(int argc, char** argv)
{
    ros::init(argc, argv, "bag_extractor");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    std::string bag_file_path;
    nh_private.param<std::string>("bag_file_path", bag_file_path, "/media/lh/lh1/dataset/thermal_radar/calib.bag");

    rosbag::Bag bag;
    
    try {
        bag.open(bag_file_path, rosbag::bagmode::Read);
        std::cout << "Open bag file: " << bag_file_path << std::endl;
    } catch (rosbag::BagIOException& e) {
        ROS_ERROR("Error opening bag file: %s", e.what());
        return -1;
    }

    // 创建输出文件夹
    std::string output_folder = bag_file_path.substr(0, bag_file_path.length() - 4) + "/";
    std::string mkdir_cmd = "mkdir -p " + output_folder;
    std::string mkdir_cmd_rgb = "mkdir -p " + output_folder + "rgb/";
    std::string mkdir_cmd_depth = "mkdir -p " + output_folder + "depth/";
    std::string mkdir_cmd_thermal = "mkdir -p " + output_folder + "thermal/";
    std::string mkdir_cmd_lidar = "mkdir -p " + output_folder + "lidar/";
    std::string mkdir_cmd_radar = "mkdir -p " + output_folder + "radar/";

    system(mkdir_cmd.c_str());
    system(mkdir_cmd_rgb.c_str());
    system(mkdir_cmd_depth.c_str());
    system(mkdir_cmd_thermal.c_str());
    system(mkdir_cmd_lidar.c_str());
    system(mkdir_cmd_radar.c_str());

    rosbag::View view(bag);
    for (rosbag::MessageInstance const m : view) {
        if (m.getTopic() == "/camera/color/image_raw") {
            sensor_msgs::Image::ConstPtr image_msg = m.instantiate<sensor_msgs::Image>();
            cv::Mat bgr_image = cv_bridge::toCvCopy(image_msg)->image;
            cv::Mat rgb_image;
            cv::cvtColor(bgr_image, rgb_image, CV_BGR2RGB);
            std::string image_filename = output_folder + "rgb/" + std::to_string(m.getTime().toNSec()) + ".png";
            cv::imwrite(image_filename, rgb_image);
        } 
        else if (m.getTopic() == "/camera/depth/image_raw") {
            sensor_msgs::Image::ConstPtr image_msg = m.instantiate<sensor_msgs::Image>();
            cv::Mat depth_image = cv_bridge::toCvCopy(image_msg)->image;
            std::string image_filename = output_folder + "depth/" + std::to_string(m.getTime().toNSec()) + ".png";
            cv::imwrite(image_filename, depth_image);
        }
        else if (m.getTopic() == "/usb_cam/image_raw/compressed") {
            sensor_msgs::CompressedImage::ConstPtr image_msg = m.instantiate<sensor_msgs::CompressedImage>();
            // cv::Mat bgr_image = cv_bridge::toCvCopy(image_msg)->image;
            // cv::Mat rgb_image;
            // cv::cvtColor(bgr_image, rgb_image, CV_BGR2RGB);
            cv::Mat image = cv_bridge::toCvCopy(image_msg)->image;
            image = image(cv::Rect(640, 300, 640, 480));
            std::string image_filename = output_folder + "thermal/" + std::to_string(m.getTime().toNSec()) + ".png";
            cv::imwrite(image_filename, image);
        }
        else if (m.getTopic() == "/Target_Radar_1") {
            sensor_msgs::PointCloud2::ConstPtr pointcloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
            // 将点云保存为.pcd
            pcl::PCLPointCloud2 pcl_pc2;
            pcl_conversions::toPCL(*pointcloud_msg, pcl_pc2);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
            std::string pcd_filename = output_folder + "radar/" + std::to_string(m.getTime().toNSec()) + ".pcd";
            pcl::io::savePCDFileBinary(pcd_filename, *cloud);
        }
        else if (m.getTopic() == "/rslidar_points") {
            sensor_msgs::PointCloud2::ConstPtr pointcloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
            // 将点云保存为.pcd
            pcl::PCLPointCloud2 pcl_pc2;
            pcl_conversions::toPCL(*pointcloud_msg, pcl_pc2);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
            std::string pcd_filename = output_folder + "lidar/" + std::to_string(m.getTime().toNSec()) + ".pcd";
            pcl::io::savePCDFileBinary(pcd_filename, *cloud);
        }
    }
    

    bag.close();

    return 0;
}