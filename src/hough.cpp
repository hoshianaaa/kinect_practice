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

#include <math.h>

class Hough
{
public:
    Hough();
private:
    ros::NodeHandle nh_;
    ros::Publisher cloud_pub_;
    ros::Publisher hough_plane_cloud_pub_;
    ros::Publisher voxel_cloud_pub_;
    ros::Publisher plane_cloud_pub_;
    ros::Subscriber image_sub_;
    ros::Subscriber cloud_sub_;
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
    std::string cloud_topic_, image_topic_;
    std::string sensor_frame_;
};

Hough::Hough(){
    cloud_topic_ = "camera/depth/points";
    //image_topic_ = "camera/rgb/image_color";
    image_topic_ = "camera/depth/image";
    image_sub_ = nh_.subscribe(image_topic_, 1, &Hough::imageCallback, this);
    cloud_sub_ = nh_.subscribe(cloud_topic_, 1, &Hough::cloudCallback, this);
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("hough_cloud", 1);
    hough_plane_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("hough_plane_cloud", 1);
    voxel_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("voxel_cloud", 1);
    plane_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("plane_cloud", 1);
}

void Hough::imageCallback(const sensor_msgs::ImageConstPtr& msg){
    /*
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

    cv::Mat filtered_img;
    cv::medianBlur(mono8_img, filtered_img, 5);
    cv::imshow("filtered image", filtered_img);

    for (int y=0;y<filtered_img.rows;++y){
        for (int x=0;x<filtered_img.cols;++x){
            std::cout << (int)filtered_img.data[ y * filtered_img.step + x * filtered_img.elemSize()] << std::endl;  
        }
    }
    
    cv::waitKey(1);
    */
}
 

void Hough::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg){

    std::cout << "cloud callback" << std::endl;
    sensor_frame_ = msg->header.frame_id;
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::PointCloud<pcl::PointXYZ> pcl_hough_cloud;
    pcl::fromROSMsg(*msg, pcl_cloud);

    long loop_num = pcl_cloud.width * pcl_cloud.height;
    //long loop_num = 7660;
    for(long i=0;i<loop_num;i+=5){
        //std::cout << loop_num - i << std::endl;
        //int i=7650;

        double x = pcl_cloud.points[i].x;
        double y = pcl_cloud.points[i].y;
        double z = pcl_cloud.points[i].z;

        double rho; 

        for(int theta=0;theta<=180;theta+=10){
            for(int phi=0;phi<=180;phi+=10){
                //theta 90
                //phi 0
                //rho = x  
                double theta_rad = M_PI*theta/180;
                double phi_rad = M_PI*phi/180;
                pcl::PointXYZ p;
                rho = x*std::sin(theta_rad)*std::cos(phi_rad) + y*std::sin(theta_rad)*std::sin(phi_rad) + z*std::cos(theta_rad);
                //std::cout << "theta:" << theta_rad << " phi:" << phi_rad << " " << rho << std::endl;

                p.x = theta/100.0; //0.001 * theta
                p.y = phi/100.0;

                //long box = rho * 1000; //0.001 
                //rho = box / 1000.0;
                p.z = rho;

                    if (!isnan(p.z)){
                        pcl_hough_cloud.push_back(p);
                    }
            }
        }
    }

    pcl::PointCloud<pcl::PointXYZ> pcl_hough_cloud_remove_nan;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(pcl_hough_cloud, pcl_hough_cloud_remove_nan, indices);

    


    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(pcl_hough_cloud_remove_nan, ros_cloud);
    ros_cloud.header.frame_id = sensor_frame_;
    cloud_pub_.publish(ros_cloud);

    //std::cout << "pcl size:" << pcl_hough_cloud_remove_nan.size() << std::endl;

    float resolution = 0.001;
    pcl::octree::OctreePointCloud<pcl::PointXYZ> octree(resolution);
    
    octree.setInputCloud (pcl_hough_cloud_remove_nan.makeShared());
    octree.defineBoundingBox();
    octree.addPointsFromInputCloud();

    std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > voxelIndeices;
    int num = octree.getOccupiedVoxelCenters(voxelIndeices); 

    std::cout << "vox num:" << num << std::endl;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree_search (resolution);

    octree_search.setInputCloud (pcl_hough_cloud_remove_nan.makeShared());
    octree_search.addPointsFromInputCloud ();

    pcl::PointXYZ searchPoint;

    pcl::PointCloud<pcl::PointXYZ> hough_plane_cloud;
    pcl::PointCloud<pcl::PointXYZ> voxel_cloud;
    pcl::PointXYZ maxPoint;

    int max_num = 0;
    for (int i=0;i<num;i++){

        std::vector<int> pointIdxVec;
        pointIdxVec.clear();
        searchPoint.x = voxelIndeices[i].x;
        searchPoint.y = voxelIndeices[i].y;
        searchPoint.z = voxelIndeices[i].z;
        
        voxel_cloud.push_back(searchPoint);

        if (octree_search.voxelSearch (searchPoint, pointIdxVec))
        {
            if (pointIdxVec.size() > max_num){

                max_num = pointIdxVec.size();
                maxPoint = searchPoint;
                std::cout << "x:" << maxPoint.x << std::endl;;
                std::cout << "y:" << maxPoint.y << std::endl;
                std::cout << "z:" << maxPoint.z << std::endl;
                std::cout << "num:" << pointIdxVec.size() << std::endl;

            }
        }
    }

    sensor_msgs::PointCloud2 ros_voxel_cloud;
    pcl::toROSMsg(voxel_cloud, ros_voxel_cloud);
    ros_voxel_cloud.header.frame_id = sensor_frame_;
    voxel_cloud_pub_.publish(ros_voxel_cloud);
    //test

    std::vector<int> pointIdxVec;
    searchPoint.x = 0.9;
    searchPoint.y = 0;
    searchPoint.z = 1;
    octree_search.voxelSearch (searchPoint, pointIdxVec);
    //std::cout << "oct num:"<< pointIdxVec.size() << std::endl;

    hough_plane_cloud.push_back(maxPoint);

    sensor_msgs::PointCloud2 ros_hough_plane_cloud;
    pcl::toROSMsg(hough_plane_cloud, ros_hough_plane_cloud);
    ros_hough_plane_cloud.header.frame_id = sensor_frame_;
    hough_plane_cloud_pub_.publish(ros_hough_plane_cloud);

    pcl::PointCloud<pcl::PointXYZ> plane_cloud;
    pcl::PointXYZ planePoint;

    double rho = maxPoint.z;
    double theta_rad = maxPoint.x * 100 / 180 * M_PI;
    double phi_rad = maxPoint.y * 100 / 180 * M_PI;

    /*
    double rho = 1;
    double theta_rad = M_PI/4;
    double phi_rad = M_PI/4;
    */

    for(long i=0;i<loop_num;i++){

        double x = pcl_cloud.points[i].x;
        double y = pcl_cloud.points[i].y;
        double z = pcl_cloud.points[i].z;

        if((std::abs(x*std::sin(theta_rad)*std::cos(phi_rad) + y*std::sin(theta_rad)*std::sin(phi_rad) + z*std::cos(theta_rad) - rho)) < 0.01){
            planePoint.x = x;
            planePoint.y = y;
            planePoint.z = z;
            plane_cloud.push_back(planePoint);
        }
    }

    sensor_msgs::PointCloud2 ros_plane_cloud;
    pcl::toROSMsg(plane_cloud, ros_plane_cloud);
    ros_plane_cloud.header.frame_id = sensor_frame_;
    plane_cloud_pub_.publish(ros_plane_cloud);
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


