#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

using namespace std;

class DummyClass{
public:
	//Error Case 1
	//pcl::PointCloud<pcl::PointXYZ>::Ptr wrong_member(new pcl::PointCloud<pcl::PointXYZ>);
	
	//Error Case 2
	pcl::PointCloud<pcl::PointXYZ>::Ptr right_member;
	
	DummyClass(){
		right_member.reset(new pcl::PointCloud<pcl::PointXYZ>());
		cout<<"Constructor complete"<<endl;
	}
	~DummyClass() {}
};

int main(){
		DummyClass ptr_test=DummyClass();
		
		pcl::PointCloud<pcl::PointXYZ> cloud;
		cloud.push_back(pcl::PointXYZ(1,2,3));
		cloud.push_back(pcl::PointXYZ(4,5,6));
		
		*ptr_test.right_member=cloud;
		cout<<ptr_test.right_member->points[0].x<<", "<<ptr_test.right_member->points[0].y<<", "<<ptr_test.right_member->points[0].z<<endl;
		
		return 0;
}
