#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

using namespace std;

int main(int argc, char **argv){
	pcl::PointCloud<pcl::PointXYZ> cloud;
	sensor_msgs::PointCloud2 output;
	
	cloud.push_back(pcl::PointXYZ(1,2,3));
	cloud.push_back(pcl::PointXYZ(4,5,6));
	cloud.push_back(pcl::PointXYZ(5,6,7));
	cloud.push_back(pcl::PointXYZ(8,9,10));
	cloud.push_back(pcl::PointXYZ(11,12,13));
	
	pcl::toROSMsg(cloud, output);
	output.header.frame_id="map";
	
	cout<<output.header.frame_id<<endl;
	cout<<output.data[0]<<endl;
	for(int i=0; i<15; i++){
		cout<<output.data[i]<<", ";
	}
	cout<<endl;
}
