#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>

class Hough
{
public:
    Hough();
private:
    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    std::string cloud_topic_, image_topic_;
};

Hough::Hough(){
    //cloud_topic_ = "camera/depth_registered/points";
    //image_topic_ = "camera/rgb/image_color";
    image_topic_ = "camera/depth/image";
    image_sub_ = nh_.subscribe(image_topic_, 1, &Hough::imageCallback, this);
}

void Hough::imageCallback(const sensor_msgs::ImageConstPtr& msg){
    std::cout << "image callback" << std::endl;
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat mono8_img = cv::Mat(cv_ptr->image.size(), CV_8UC1);
    cv::convertScaleAbs(cv_ptr->image, mono8_img, 100, 0.0);
    cv::imshow("image", mono8_img);
    cv::waitKey(1);
}
 
int main(int argc, char **argv)
{
    std::cout << "start hough" << std::endl;
    ros::init(argc, argv, "hough");
    Hough rd;
    ros::Rate r(10);
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}


