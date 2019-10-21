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

class ReadData
{
public:
    ReadData();
private:
    ros::NodeHandle nh_;
    ros::Publisher cloud_pub_;
    ros::Publisher sac_cloud_pub_;
    ros::Publisher pass_cloud_pub_;
    ros::Publisher image_pub_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber image_sub_;
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msgs);
    void imageCallback(const sensor_msgs::ImageConstPtr& msgs);
    std::string cloud_topic_, image_topic_;
    void segmentate(pcl::PointCloud<pcl::PointXYZRGB>& cloud, double threshould);
    void passThrough(pcl::PointCloud<pcl::PointXYZRGB>& cloud, const double sx, const double ex, const double sy, const double ey, const double sz, const double ez);
};

ReadData::ReadData(){
    cloud_topic_ = "camera/depth_registered/points";
    image_topic_ = "camera/rgb/image_color";

    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/vox_cloud", 1, false);
    sac_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/sac_cloud", 1, false);
    pass_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/pass_cloud", 1, false);
    image_pub_ = nh_.advertise<sensor_msgs::Image>("/cloud_image", 1, false);
    cloud_sub_ = nh_.subscribe(cloud_topic_, 1, &ReadData::cloudCallback, this);
    image_sub_ = nh_.subscribe(image_topic_, 1, &ReadData::imageCallback, this);

}

void ReadData::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msgs){
    std::cout << "cloud callback" << std::endl;

    std::string sensor_frame = msgs->header.frame_id;

    pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;

    pcl::fromROSMsg(*msgs, pcl_cloud);

    std::cout << "before_down_sampling:" << pcl_cloud.size() << std::endl;

    pcl::VoxelGrid<pcl::PointXYZRGB> vox;
    vox.setInputCloud(pcl_cloud.makeShared());
    vox.setLeafSize(0.05f, 0.05f, 0.05f);
    vox.filter(pcl_cloud);

    std::cout << "after_down_sampling:" << pcl_cloud.size() << std::endl;

    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(pcl_cloud, ros_cloud);
    ros_cloud.header.frame_id = sensor_frame;
    cloud_pub_.publish(ros_cloud);

    pcl::PointCloud<pcl::PointXYZRGB> sac_pcl_cloud;
    sac_pcl_cloud = pcl_cloud;
    segmentate(sac_pcl_cloud, 0.01);
    pcl::toROSMsg(sac_pcl_cloud, ros_cloud);
    ros_cloud.header.frame_id = sensor_frame;
    sac_cloud_pub_.publish(ros_cloud);

    pcl::PointCloud<pcl::PointXYZRGB> pass_pcl_cloud;
    pass_pcl_cloud = pcl_cloud;
    passThrough(pass_pcl_cloud, 0.0,1.0,-1.0,1.0,-5.0,5.0);
    pcl::toROSMsg(pass_pcl_cloud, ros_cloud);
    ros_cloud.header.frame_id = sensor_frame;
    pass_cloud_pub_.publish(ros_cloud);

    sensor_msgs::Image image;
    image.header.frame_id = sensor_frame;
    std::cout << "pcl_cloud_size:" << pcl_cloud.size() << std::endl;
    std::cout << "pcl_cloud_height:" << pcl_cloud.height << std::endl;
    std::cout << "pcl_cloud_width:" << pcl_cloud.width << std::endl;
    std::cout << "image:" <<  image.step << std::endl;
    pcl::toROSMsg(pcl_cloud, image);
    std::cout << "image:" <<  image.step << std::endl;
    image_pub_.publish(image);

}

void ReadData::imageCallback(const sensor_msgs::ImageConstPtr& msgs){
    std::cout << "image callback" << std::endl;
}

void ReadData::passThrough(pcl::PointCloud<pcl::PointXYZRGB>& cloud, const double sx, const double ex, const double sy, const double ey, const double sz, const double ez){

    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud.makeShared());
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (sx, ex);
    pass.filter (cloud);    
    std::cout << "debug1:" << cloud.size() << std::endl;

    pass.setInputCloud (cloud.makeShared());
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (sy, ey);
    pass.filter (cloud);    
    std::cout << "debug2:" << cloud.size() << std::endl;

    pass.setInputCloud (cloud.makeShared());
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (sz, ez);
    pass.filter (cloud); 
    std::cout << "debug3:" << cloud.size() << std::endl;
}

void ReadData::segmentate(pcl::PointCloud<pcl::PointXYZRGB>& cloud, double threshould) {  
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);  
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);  
    // Create the segmentation object  
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;  
    // Optional  
    seg.setOptimizeCoefficients (true);  
    // Mandatory  
    seg.setModelType (pcl::SACMODEL_PLANE);  
    seg.setMethodType (pcl::SAC_RANSAC);  
    seg.setDistanceThreshold (threshould);  

    seg.setInputCloud (cloud.makeShared ());  
    seg.segment (*inliers, *coefficients);  

    pcl::PointCloud<pcl::PointXYZRGB> out_cloud;

    for (size_t i = 0; i < inliers->indices.size (); ++i) {  
        out_cloud.push_back(cloud.points[inliers->indices[i]]);  
    }  
    cloud = out_cloud;
} 

int main(int argc, char **argv)
{
    std::cout << "start read_data" << std::endl;
    ros::init(argc, argv, "read_data");
    ReadData rd;
    ros::Rate r(10);
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}


