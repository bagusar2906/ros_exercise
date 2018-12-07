#include <iostream>
#include <fstream>
#include <ros/ros.h>
//#include <sensor_msgs/PointCloud.h>
#include <pcl_ros/filters/statistical_outlier_removal.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/filters/radius_outlier_removal.h>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_filtered(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // Container for original & filtered data
    pcl::PCLPointCloud2 pcl_pc2;
    //pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    //pcl::PCLPointCloud2 cloud_filtered;

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(temp_cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filtered);

    sor.setNegative(true);
    sor.filter(*cloud_filtered);

    return cloud_filtered;
}

void statistical_outlier_removal(const PointCloud::ConstPtr &cloud)
{
    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(1);
    sor.setStddevMulThresh(0.9);
    //sor.filter (*cloud_filtered);

    sor.setNegative(true);
    sor.filter(*cloud_filtered);
}

void radius_outlier_removal(const PointCloud::ConstPtr &cloud)
{
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    // build the filter
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(1.5);
    outrem.setMinNeighborsInRadius (2);
    outrem.setNegative(true);
    // apply filter
    outrem.filter (*cloud_filtered);
}

void on_recv_cb(const PointCloud::ConstPtr &msg)
{

    printf("Cloud: width = %d, height = %d\n", msg->width, msg->height);
    //BOOST_FOREACH (const pcl::PointXYZ &pt, msg->points)
    //    printf("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
    //statistical_outlier_removal(msg);
    
    radius_outlier_removal(msg);

    BOOST_FOREACH (const pcl::PointXYZ &pt, cloud_filtered->points)
        printf("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_filter");
    ros::NodeHandle pcl_node;
    ros::Subscriber pcl_sub = pcl_node.subscribe<PointCloud>("pcl_pub_path", 1, on_recv_cb);
    ros::Publisher pcl_pub = pcl_node.advertise<PointCloud>("pcl_outlier_path", 1, false);    
    cloud_filtered->header.frame_id = "map";
    //pcl_pub.publish(cloud_filtered);
    //ros::spin();

    ros::Rate r = ros::Rate(5);
    while (!ros::isShuttingDown())
    {
        ros::spinOnce();
        pcl_pub.publish(cloud_filtered);
        r.sleep();
    }
    
}
