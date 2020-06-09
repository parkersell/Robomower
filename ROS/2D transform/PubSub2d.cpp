#include <ros/ros.h>
#include <PublisherSubscriber.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <boost/foreach.hpp>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

template<>
void PublisherSubscriber<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>::subscriberCallback(const sensor_msgs::PointCloud2ConstPtr& recievedMsg){
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;
    

    pcl::PCLPointCloud2 pcl_pc2;//defines pcl2
    pcl_conversions::toPCL(*recievedMsg,pcl_pc2);//converts orb cloud to pcl2
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);//defines new pcl ptr type that is more useful
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);//transfers pcl2 to better type
    for (int j=0; j<temp_cloud->points.size(); j++){
       temp_cloud->points[j].z =0;
    }
    sensor_msgs::PointCloud2 output1;
    pcl::toROSMsg(*temp_cloud, output1);//turns output1 into sensormsgs that contains temp_cloud
    pcl_conversions::toPCL(output1, *cloud);//converts it into pcl2 cloud
   
  //BOOST_FOREACH (const pcl::PointXYZ& pt, temp_cloud->points)
    //  ROS_INFO("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z); //prints out each point
      
      

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.1, 0.1, 0.1);
  sor.filter (cloud_filtered);

  sensor_msgs::PointCloud2 output2;
  // Convert to ROS data type
  pcl_conversions::fromPCL(cloud_filtered, output2);
  
  //pcl_conversions::moveFromPCL(cloud_filtered, output);
    
  publisherObject.publish(output2);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pubsub2d");
    while(ros::ok()){
    PublisherSubscriber<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> parrot("echo", "/orb_slam2_mono/map_points", 1);
    ros::spin();
    }
    
}