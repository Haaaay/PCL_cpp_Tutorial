#include <iostream>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <string>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

pcl::PointCloud<pcl::PointXYZI>::ConstPtr load_bin(const string &filename){
	FILE *file=fopen(filename.c_str(), "rb");
	if(!file){
		std::cerr<<"Error : failed to load"<<filename<<std::endl;
		return nullptr;
	}
	std::vector<float> buffer(1000000);
	size_t num_points=fread(reinterpret_cast<char*>(buffer.data()), sizeof(float), buffer.size(), file)/4;
	fclose(file);
	
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
	cloud->resize(num_points);
	
	for(int i=0; i<num_points; i++){
		auto &pt=cloud->at(i);
		pt.x=buffer[i*4];
		pt.y=buffer[i*4+1];
		pt.z=buffer[i*4+2];
	}
	return cloud;
}

void colorize(const pcl::PointCloud<pcl::PointXYZI> &pc, pcl::PointCloud<pcl::PointXYZRGB> &pc_colored, const std::vector<int> &color){
	int N=pc.points.size();
	
	pc_colored.clear();
	pcl::PointXYZRGB pt_tmp;
	for(int i=0; i<N; ++i){
		const auto &pt=pc.points[i];
		pt_tmp.x=pt.x;
		pt_tmp.y=pt.y;
		pt_tmp.z=pt.z;
		pt_tmp.r=color[0];
		pt_tmp.g=color[1];
		pt_tmp.b=color[2];
		
		pc_colored.points.emplace_back(pt_tmp);
	}
}

void remove_unintended_parts(const pcl::PointCloud<pcl::PointXYZI>& src, pcl::PointCloud<pcl::PointXYZI>& dst){
	pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_filtered(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PassThrough<pcl::PointXYZI> ptfilter;
	
	*ptr_filtered=src;
	float robot_size=3.0;
	float range_max=8.0;
	
	//part A
	ptfilter.setInputCloud(ptr_filtered);
	ptfilter.setFilterFieldName("y");
	ptfilter.setFilterLimits(-robot_size, robot_size);
	ptfilter.setFilterLimitsNegative(true);
	ptfilter.filter(*ptr_filtered);

	dst=*ptr_filtered;
	
	//part B
	*ptr_filtered=src;
	ptfilter.setInputCloud(ptr_filtered);
	ptfilter.setFilterFieldName("y");
	ptfilter.setFilterLimits(-robot_size, robot_size);
	ptfilter.setFilterLimitsNegative(false);
	ptfilter.filter(*ptr_filtered);
	
	ptfilter.setInputCloud(ptr_filtered);
	ptfilter.setFilterFieldName("x");
	ptfilter.setFilterLimits(0.4, range_max);
	ptfilter.filter(*ptr_filtered);
	
	dst+=*ptr_filtered;
}

int main(int argc, char** argv){
	//Load data
	pcl::PointCloud<pcl::PointXYZI>::Ptr src(new pcl::PointCloud<pcl::PointXYZI>);
	*src=*load_bin("/home/maniac/velo_ws/src/pcl_cpp_tutorial/test.bin");
	
	pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);
	
	remove_unintended_parts(*src, *filtered);
	
	//결과 시각화
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
	
	colorize(*src, *src_colored, {255,0,0});
	colorize(*filtered, *out_colored, {0,255,0});
	
	pcl::visualization::PCLVisualizer viewer1("Raw");
	pcl::visualization::PCLVisualizer viewer2("filtered");
	
	viewer1.addPointCloud<pcl::PointXYZRGB>(src_colored, "src_red");
	viewer2.addPointCloud<pcl::PointXYZRGB>(out_colored, "filtered_green");
	
	while(!viewer1.wasStopped()&&!viewer2.wasStopped()){
		viewer1.spinOnce();
		viewer2.spinOnce();
	}
	
	return 0;
}

