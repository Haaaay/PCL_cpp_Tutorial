#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <vector>
#include <ctime>
#include <string>

using namespace std;

pcl::PointCloud<pcl::PointXYZ>::ConstPtr load_bin(const string &filename) {
	FILE*file = fopen(filename.c_str(), "rb");
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
	//Load data
	pcl::PointCloud<pcl::PointXYZ>::Ptr src(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr boundary(new pcl::PointCloud<pcl::PointXYZ>);
	*src=*load_bin("/home/maniac/velo_ws/src/pcl_cpp_tutorial/test.bin");
	
	//main
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    int N = 1200; // 차랑 주변 크기
    std::vector<int> idxes;
    std::vector<float> sqr_dists;

    kdtree.setInputCloud(src);
    pcl::PointXYZ query(6.0, 6.0, 0.0);

    cout<<query.x <<", "<< query.y<<", "<< query.z<<endl;
    kdtree.nearestKSearch(query, N, idxes, sqr_dists);
	
	for (const auto& idx: idxes){
        boundary->points.push_back(src->points[idx]);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_color(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::uint8_t r(255), g(15), b(15);
    for (const auto &basic_point: src->points) {
        pcl::PointXYZRGB point;
        point.x = basic_point.x;
        point.y = basic_point.y;
        point.z = basic_point.z;
        std::uint32_t rgb = (static_cast<std::uint32_t>(r) << 16 | static_cast<std::uint32_t>(g) << 8 | static_cast<std::uint32_t>(b));
        point.rgb = *reinterpret_cast<float *>(&rgb);
        src_color->points.push_back(point);
    }
	
	//결과 시각화
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr boundary_color(new pcl::PointCloud<pcl::PointXYZRGB>);
    r = 0;
    g = 255;
    b = 0;
    for (const auto &basic_point: boundary->points) {
        pcl::PointXYZRGB point;
        point.x = basic_point.x;
        point.y = basic_point.y;
        point.z = basic_point.z;
        std::uint32_t rgb = (static_cast<std::uint32_t>(r) << 16 | static_cast<std::uint32_t>(g) << 8 | static_cast<std::uint32_t>(b));
        point.rgb = *reinterpret_cast<float *>(&rgb);
        boundary_color->points.push_back(point);
    }
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr seed(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointXYZRGB point;
    point.x = query.x;
    point.y = query.y;
    point.z = query.z;
    g = 0;
    b = 255;
    std::uint32_t rgb = (static_cast<std::uint32_t>(r) << 16 |
                         static_cast<std::uint32_t>(g) << 8 | static_cast<std::uint32_t>(b));
    point.rgb = *reinterpret_cast<float *>(&rgb);
    seed->points.push_back(point);

    pcl::visualization::PCLVisualizer viewer("Simple Cloud Viewer");
    viewer.addPointCloud<pcl::PointXYZRGB>(src_color, "src");
    viewer.addPointCloud<pcl::PointXYZRGB>(boundary_color, "boundary");
    viewer.addPointCloud<pcl::PointXYZRGB>(seed, "seed");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "seed");
    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }
	
}
