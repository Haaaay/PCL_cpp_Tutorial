#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

using namespace std;

template <class T>
void print_pc(pcl::PointCloud<T>& cloud){
	int count=0;
	
	for(const auto& pt: cloud.points){
		cout << count++ << ": ";
		cout << pt.x << ", "<<pt.y << ", "<< pt.z << endl;
	}
	
	cout<<"begin(): ";
	cout<<cloud.begin()->x<<", ";
	cout<<cloud.begin()->y<<", ";
	cout<<cloud.begin()->z<<endl;
	
	cout<<"end() -1: ";
	cout<<(cloud.end()-1)->x<<", ";
	cout<<(cloud.end()-1)->y<<", ";
	cout<<(cloud.end()-1)->z<<endl;
	
	cout<<"front(): ";
	cout<<cloud.front().x<<", ";
	cout<<cloud.front().y<<", ";
	cout<<cloud.front().z<<endl;
	
	cout<<"back(): ";
	cout<<cloud.back().x<<", ";
	cout<<cloud.back().y<<", ";
	cout<<cloud.back().z<<endl;
	
	cout<<"at(1): ";
	cout<<cloud.at(1).x<<", ";
	cout<<cloud.at(1).y<<", ";
	cout<<cloud.at(1).z<<endl;
	
	if(cloud.empty()){
		cout<<"True";
	}
	else{
		cout<<"False";
	}
	cout<<"	|	size of pc: "<<cloud.size()<<endl;
	
	cloud.clear();
	if(cloud.empty()){
		cout<<"True";
	}
	else{
		cout<<"False";
	}
	cout<<"	|	size of pc: "<<cloud.size()<<endl;
}

int main(int argc, char **argv){
	//cloud_init0
	pcl::PointCloud<pcl::PointXYZ> cloud_init0;
	cloud_init0.resize(3);
	
	cloud_init0.points[0].x=1;
	cloud_init0.points[0].y=2;
	cloud_init0.points[0].z=3;
	
	cloud_init0.points[1].x=4;
	cloud_init0.points[1].y=5;
	cloud_init0.points[1].z=6;
	
	cloud_init0.points[2].x=7;
	cloud_init0.points[2].y=8;
	cloud_init0.points[2].z=9;
	cout<<"cloud_init0: "<<endl;
	print_pc(cloud_init0);
	cout<<"============="<<endl;
	
	//cloud_init1
	pcl::PointCloud<pcl::PointXYZ> cloud_init1;
	pcl::PointXYZ point_xyz;
	
	point_xyz.x=1;
	point_xyz.y=2;
	point_xyz.z=3;
	cloud_init1.push_back(point_xyz);
	
	point_xyz.x=4;
	point_xyz.y=5;
	point_xyz.z=6;
	cloud_init1.push_back(point_xyz);
	
	point_xyz.x=7;
	point_xyz.y=8;
	point_xyz.z=9;
	cloud_init1.push_back(point_xyz);
	
	cout<<"cloud_init1: "<<endl;
	print_pc(cloud_init1);
	cout<<"============="<<endl;
	
	//cloud_init2
	pcl::PointCloud<pcl::PointXYZ> cloud_init2;
	cloud_init2.push_back(pcl::PointXYZ(1,2,3));
	cloud_init2.push_back(pcl::PointXYZ(4,5,6));
	cloud_init2.push_back(pcl::PointXYZ(7,8,9));
	cout<<"cloud_init2: "<<endl;
	print_pc(cloud_init2);
	cout<<"============="<<endl;
	
	pcl::PointCloud<pcl::PointXYZ> cloud2;
	cloud2.push_back(pcl::PointXYZ(1,2,3));
	cloud2.push_back(pcl::PointXYZ(4,5,6));
	
	pcl::PointCloud<pcl::PointXYZ> cloud3;
	cloud3.push_back(pcl::PointXYZ(7,8,9));
	cloud3.push_back(pcl::PointXYZ(10,11,12));
	
	cloud2+=cloud3;
	
	cout<<"size: "<<cloud2.size()<<endl;
	print_pc(cloud2);
	
	cloud3.push_back(pcl::PointXYZ(12,13,14));
	cout<<"After: "<<endl;
	print_pc(cloud3);
	
	return 0;
}
