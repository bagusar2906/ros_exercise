#include <iostream>
#include <fstream>
#include <ros/ros.h>
//#include <geometry_msgs/Point32.h>
//#include <sensor_msgs/PointCloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl_nodes/GetNumberOfPoints.h"

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

PointCloud::Ptr _pcl (new PointCloud);

vector<string> split(string str, string token)
{
    vector<string> result;
    while (str.size())
    {
        int index = str.find(token);
        if (index != string::npos)
        {
            result.push_back(str.substr(0, index));
            str = str.substr(index + token.size());
            if (str.size() == 0)
                result.push_back(str);
        }
        else
        {
            result.push_back(str);
            str = "";
        }
    }
    return result;
}

bool GetNumberOfPoints(pcl_nodes::GetNumberOfPointsRequest &request, pcl_nodes::GetNumberOfPointsResponse &response)
{
    ROS_INFO("GetNumberOfPoints()");
    response.count = _pcl->points.size();
    return true;
}

void load_from_file(string filename)
{
    string line;

    ROS_INFO("Loading data: %s", filename.c_str());

    vector<pcl::PointXYZ> points;

    ifstream pctdata(filename);
    if (pctdata.is_open())
    {
        while (getline(pctdata, line))
        {
            cout << line << endl;

            vector<string> data = split(line, ",");
                        
            float x = stof(data[0].c_str());
            float y = stof(data[1].c_str());            
            
            _pcl->points.push_back(pcl::PointXYZ(x, y, 0.0));
        }
        pctdata.close();

        cout << _pcl->points.size() << " points are loaded and publish as point cloud.." << endl;
                
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_publisher");
    ros::NodeHandle pcl_node;
    ros::Publisher pcl_pub = pcl_node.advertise<PointCloud>("pcl_pub_path", 1, true);
    ros::ServiceServer pcl_srv = pcl_node.advertiseService("get_number_of_points", GetNumberOfPoints);


    string filename = argv[1];
    
    _pcl->header.frame_id = "map";
    //std_msgs::Header header = std_msgs::Header();
    //header->stamp = ros::Time(0);
    
    //pcl = sensor_msgs::PointCloud();
    //pcl.header = header;
    load_from_file(filename);
    
    //pcl->points = _points;
    _pcl->height = 1;
    _pcl->width = _pcl->points.size();

    pcl_conversions::toPCL(ros::Time::now(), _pcl->header.stamp);
    pcl_pub.publish(_pcl);
    ros::spin();
    /*
    ros::Rate r = ros::Rate(5);
    while ( ! ros::isShuttingDown()) {
        pct_pub.publish(pct);
        r.sleep();
    }
    */
}
