#include <pcl/PCLPointCloud2.h>

using namespace std;

int main(int argc, char** argv){
	boost::shared_ptr<vector<float>> vec_ptr(new vector<float>());
	vector<float> data={1.1,2.2,3.3,4.4,5.5};

	cout<<vec_ptr<<endl;

	*vec_ptr=data;
	for(int i=0; i<vec_ptr->size(); ++i){
		cout<<(*vec_ptr)[i]<<", ";
	}
	cout<<endl;

	cout<<vec_ptr<<endl;

	boost::shared_ptr<vector<int>> vec_ptr2;
	vec_ptr2.reset(new vector<int>(3,5));
	for(int i=0; i<vec_ptr2->size(); ++i){
		cout<<(*vec_ptr2)[i]<<", ";
	}
	cout<<endl;

	return 0;
}
