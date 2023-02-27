#include <iostream>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using namespace std;
template <class T>
void print_pc(pcl::PointCloud<T>& cloud){
	int count=0;
	
	for(const auto& pt: cloud.points){
		cout << count++ << ": ";
		cout << pt.x << ", "<<pt.y << ", "<< pt.z << endl;
	}
}

int main(int argc, char **argv){
	pcl::PointCloud<pcl::PointXYZ> cloud_src;
	
	pcl::PointXYZ point_xyz;
	
	for(int k=0; k<5; ++k){
		/*
		float x=static_cast<float>(rand())/static_cast<float>(RAND_MAX)*5;
		float y=static_cast<float>(rand())/static_cast<float>(RAND_MAX)*5;
		float z=static_cast<float>(rand())/static_cast<float>(RAND_MAX)*5;
		*/
		float x=1.0;
		float y=2.0;
		float z=3.0;
		point_xyz.x=x;
		point_xyz.y=y;
		point_xyz.z=z;
		
		cloud_src.push_back(point_xyz);
	}
	pcl::PointCloud<pcl::PointXYZ> pc_transformed;
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_transformed(new pcl::PointCloud<pcl::PointXYZ>);
	
	Eigen::Matrix4f trans;
	trans<<1, 0, 0, 0.165,
		   0, 1, 0, 0.500,
		   0, 0, 1, -0.320,
		   0, 0, 0, 1;
	pcl::transformPointCloud(cloud_src, *ptr_transformed, trans);
	
	pc_transformed=*ptr_transformed;
	print_pc(pc_transformed);
	
	return 0;
}
