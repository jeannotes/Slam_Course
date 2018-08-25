#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

using namespace std;

int main(){
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.width = 5;
    cloud.height =1;
    cloud.points.resize(cloud.width * cloud.height);

    for(size_t i = 0; i< cloud.points.size(); i++){
        cloud.points[i].x = i;
        cloud.points[i].y = i;
        cloud.points[i].z = i;
    }
    pcl::io::savePCDFileASCII("test1.pcd", cloud);
    // load pcd
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile("test1.pcd", *cloud_) == -1){
        cout << "over";
    }

    for(size_t i = 0; i<cloud_->points.size(); i++){
        cout << cloud_->points[i] << endl;
    }

    pcl::visualization::CloudViewer viewer("viewer");
    viewer.showCloud(cloud_);
    while(!viewer.wasStopped()){}
}
