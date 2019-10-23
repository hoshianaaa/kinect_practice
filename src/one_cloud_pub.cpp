#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include <pcl/octree/octree.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>

int main(int argc, char **argv)
{
    std::cout << "start hough" << std::endl;
    ros::init(argc, argv, "one_cloud_pub");
    ros::NodeHandle nh;
    std::string sensor_frame;

    sensor_frame = "camera_depth_optical_frame";

    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("camera/depth/points", 1);

    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::PointXYZ p;

    p.x = 1;
    p.y = 0;
    p.z = 0;
 
    pcl_cloud.push_back(p);

    p.x = 1;
    p.y = 1;
    p.z = 0;
 
    pcl_cloud.push_back(p);

    p.x = 1;
    p.y = 0;
    p.z = 1;
 
    pcl_cloud.push_back(p);


    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(pcl_cloud, ros_cloud);
    ros_cloud.header.frame_id = sensor_frame;
    ros::Rate r(10);
    while(ros::ok()){
        cloud_pub.publish(ros_cloud);
        std::cout << "pub cloud" << std::endl;
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}

/*
x,y,z = (1,0,0) 
phi=0 -> rho=xcos(theta)+ysin(theta)
theta   phi     rho
0       0       1    
90      0       y+z=0
180     0       -1
*/



       

