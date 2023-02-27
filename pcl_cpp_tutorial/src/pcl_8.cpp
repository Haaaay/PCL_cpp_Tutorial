#include <iostream>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>
#include <ctime>
#include <string>

using namespace std;

pcl::PointCloud<pcl::PointXYZ>::ConstPtr load_bin(const string &filename){
	FILE*file=fopen(filename.c_str(), "rb");
	if(!file){
		std::cerr<<"Error : failed to load"<<filename<<std::endl;
		return nullptr;
	}
	std::vector<float> buffer(1000000);
	size_t num_points=fread(reinterpret_cast<char*>(buffer.data()), sizeof(float), buffer.size(), file)/4;
	fclose(file);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	cloud->resize(num_points);
	
	for(int i=0; i<num_points; i++){
		auto &pt=cloud->at(i);
		
		pt.x=buffer[i*4];
		pt.y=buffer[i*4+1];
		pt.z=buffer[i*4+2];
	}
	return cloud;
}

int main(int argc, char **argv){
	//load data
	pcl::PointCloud<pcl::PointXYZ>::Ptr src(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr boundary1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr boundary2(new pcl::PointCloud<pcl::PointXYZ>);
	
	*src=*load_bin("/home/maniac/velo_ws/src/pcl_cpp_tutorial/test.bin");
	
	//main
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	int radius=8.0;
	std::vector<int> idxes;
	std::vector<float> sqr_dists;
	
	kdtree.setInputCloud(src);
	//기준점
	pcl::PointXYZ query1(0.0, 0.0, 0.0);
	pcl::PointXYZ query2(20.0, 0.0, 0.0);
	
	kdtree.radiusSearch(query1, radius, idxes, sqr_dists);
	for(const auto &idx: idxes){
		boundary1->points.push_back(src->points[idx]);
	}
	kdtree.radiusSearch(query2, radius, idxes, sqr_dists);
	for(const auto &idx: idxes){
		boundary2->points.push_back(src->points[idx]);
	}
	
	//결과 시각화
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_color(new pcl::PointCloud<pcl::PointXYZRGB>);
	std::uint8_t r(255), g(15), b(15);
	for(const auto &basic_point: src->points){
		pcl::PointXYZRGB point;
		
		point.x=basic_point.x;
		point.y=basic_point.y;
		point.z=basic_point.z;
		std::uint32_t rgb=(static_cast<std::uint32_t>(r)<<16 | static_cast<std::uint32_t>(g)<<8 | static_cast<std::uint32_t>(b));
		point.rgb=*reinterpret_cast<float *>(&rgb);
		src_color->points.push_back(point);
	}
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr boundary_color(new pcl::PointCloud<pcl::PointXYZRGB>);
	
	r=0;
	g=255;
	b=0;
	for(const auto &basic_point: boundary1->points){
		pcl::PointXYZRGB point;
		
		point.x=basic_point.x;
		point.y=basic_point.y;
		point.z=basic_point.z;
		std::uint32_t rgb=(static_cast<std::uint32_t>(r)<<16 | static_cast<std::uint32_t>(g)<<8 | static_cast<std::uint32_t>(b));
		point.rgb=*reinterpret_cast<float *>(&rgb);
		printf("rgb : %d\n", rgb);
		boundary_color->points.push_back(point);
	}
	
	r=0;
	g=0;
	b=255;
	for(const auto &basic_point: boundary2->points){
		pcl::PointXYZRGB point;
		
		point.x=basic_point.x;
		point.y=basic_point.y;
		point.z=basic_point.z;
		std::uint32_t rgb=(static_cast<std::uint32_t>(r)<<16 | static_cast<std::uint32_t>(g)<<8 | static_cast<std::uint32_t>(b));
		point.rgb=*reinterpret_cast<float *>(&rgb);
		boundary_color->points.push_back(point);
	}
	
	pcl::visualization::PCLVisualizer viewer("Simple Cloud Viewer");
	viewer.addPointCloud<pcl::PointXYZRGB>(src_color, "src");
	viewer.addPointCloud<pcl::PointXYZRGB>(boundary_color, "boundaries");
	
	std::uint8_t num_1(10), num_2(20), num_3(30);
	printf("uint8 num_1 : %d\tnum_2 : %d\tnum_3 : %d\n", num_1, num_2, num_3);
	std::uint32_t trans_num=(static_cast<std::uint32_t>(num_1)<<16 | static_cast<std::uint32_t>(num_2)<<8 | static_cast<std::uint32_t>(num_3));
	printf("uint32 num : %d\n", trans_num);
	
	std::float_t num=*reinterpret_cast<float *>(&trans_num);
	printf("num : %f\n", num);	
	
	while(!viewer.wasStopped()){
		viewer.spinOnce();
	}
	
	return 0;
}
