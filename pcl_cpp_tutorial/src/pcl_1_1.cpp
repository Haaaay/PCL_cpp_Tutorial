#include <pcl/point_types.h>
#include <pcl/conversions.h>

using namespace std;

template<class T>
void print_pc(pcl::PointCloud<T> &cloud){
	int count=0;
	for(const auto &pt: cloud.points){
		cout<<count++<<": ";
		cout<<pt.x<<", "<<pt.y<<", "<<pt.z<<endl;
	}
}

template<class T>
void print_pc_via_copy(pcl::PointCloud<T> cloud){
	int count=0;
	for(const auto &pt: cloud.points){
		cout<<count++<<": ";
		cout<<pt.x<<", "<<pt.y<<", "<<pt.z<<endl;
	}
}

void print_ptr(pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud){
	int count=0;
	for(const auto &pt: ptr_cloud->points){
		cout<<count++<<": ";
		cout<<pt.x<<", "<<pt.y<<", "<<pt.z<<endl;
	}
}

void print_address(pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud){
	cout<<ptr_cloud<<endl;
}

void print_address2(pcl::PointCloud<pcl::PointXYZ>::Ptr &ptr_cloud){
	cout<<ptr_cloud<<endl;
}

void add_pc(pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud){
	ptr_cloud->points.emplace_back(pcl::PointXYZ(0, 0, 1));
}

void add_pc2(pcl::PointCloud<pcl::PointXYZ>::Ptr &ptr_cloud){
	ptr_cloud->points.emplace_back(pcl::PointXYZ(1,0,0));
}

void consptr_test(pcl::PointCloud<pcl::PointXYZ>::ConstPtr ptr_cloud){
	pcl::PointCloud<pcl::PointXYZ> pc=*ptr_cloud;
	print_pc(pc);
	print_pc_via_copy(*ptr_cloud);
}

int main(int argc, char **argv){
	pcl::PointCloud<pcl::PointXYZ> cloud2;
	cloud2.push_back(pcl::PointXYZ(1,2,3));
	cloud2.push_back(pcl::PointXYZ(4,5,6));

	pcl::PointCloud<pcl::PointXYZ> cloud3;
	cloud3.push_back(pcl::PointXYZ(7,8,9));
	cloud3.push_back(pcl::PointXYZ(10,11,12));
	
	cloud2+=cloud3;

	pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud2(new pcl::PointCloud<pcl::PointXYZ>);
	*ptr_cloud=cloud2;

	cout<<"Before: "<<endl;
	print_pc(cloud3);

	cloud3=*ptr_cloud;
	cout<<"After: "<<endl;
	print_pc(cloud3);

	ptr_cloud2=ptr_cloud;

	std::cout<<ptr_cloud<<std::endl;
	std::cout<<ptr_cloud2<<std::endl;

	ptr_cloud2->points.emplace_back(pcl::PointXYZ(-1,-2,-3));
	std::cout<<ptr_cloud->points.back().x<<", "<<ptr_cloud->points.back().y<<", "<<ptr_cloud->points.back().z<<endl;
	
	cout<<"-----clouds-----"<<endl;
	print_pc(cloud2);
	print_pc(cloud3);

	cout<<"-----Ptrs-----"<<endl;
	print_ptr(ptr_cloud);
	print_ptr(ptr_cloud2);
	cout<<"--------------"<<endl;

	print_address(ptr_cloud);
	print_address2(ptr_cloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud3(new pcl::PointCloud<pcl::PointXYZ>);
	*ptr_cloud3=*ptr_cloud;

	cout<<"-----add_pc-----"<<endl;
	add_pc(ptr_cloud);
	add_pc2(ptr_cloud);
	print_ptr(ptr_cloud);

	cout<<"----- $Ptr$ = $Ptr$ -----"<<endl;
	print_ptr(ptr_cloud2);

	cout<<"----- *$Ptr$ = *$Ptr$ -----" << endl;
	print_ptr(ptr_cloud3);

	cout<<"--- ConstPtr test ---"<<endl;
	consptr_test(ptr_cloud);

	return 0;
}
