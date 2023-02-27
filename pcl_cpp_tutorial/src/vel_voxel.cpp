#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/PCLPointCloud2.h>

ros::Publisher voxel_velodyne_data;

using namespace std;
pcl::PointCloud<pcl::PointXYZ>::Ptr src(new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloudmsg2cloud(sensor_msgs::PointCloud2 cloudmsg){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_dst(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::fromROSMsg(cloudmsg, *cloud_dst);
	return cloud_dst;
}

void VelodynePointsCallback(const sensor_msgs::PointCloud2::ConstPtr &velodyne_data){
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr src(new pcl::PointCloud<pcl::PointXYZRGB>);
	*src=*cloudmsg2cloud(*velodyne_data);
	
	/*
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr vox_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
	
	pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
	
	float voxelsize=0.2;
	voxel_filter.setInputCloud(src);
	voxel_filter.setLeafSize(voxelsize, voxelsize, voxelsize);
	voxel_filter.filter(*ptr_filtered);
	
	colorize(*ptr_filtered, vox_colored, {255, 255, 0});
	*/
}

int main(int argc, char **argv){
	ros::init(argc, argv, "Velodyne_Voxelization");
	ros::NodeHandle nh;
	
	ros::Subscriber velodyne_data=nh.subscribe("/velodyne_points", 100, &VelodynePointsCallback);
	//voxel_velodyne_data=nh.advertise<sensor_msgs::PointCloud2.h>("/voxel_velodyne_points", 100);
	
	ros::spin();
	
	while(ros::ok()){
		
	}
	
	return 0;
}
