#include <pcl-1.6/pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl-1.6/pcl/io/io.h>
#include <pcl-1.6/pcl/io/pcd_io.h>
#include<pcl-1.6/pcl/point_cloud.h>
#include<pcl-1.6/pcl/point_types.h>
#include<cmath>
    
int user_data;
    
void 
viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor (1.0, 0.5, 1.0);
    pcl::PointXYZ o;
    o.x = 1.0;
    o.y = 0;
    o.z = 0;
    viewer.addSphere (o, 0.25, "sphere", 0);
    std::cout << "i only run once" << std::endl;
    
}
    
void 
viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
{
    static unsigned count = 0;
    std::stringstream ss;
    ss << "Once per viewer loop: " << count++;
    viewer.removeShape ("text", 0);
    viewer.addText (ss.str(), 200, 300, "text", 0);
    
    //FIXME: possible race condition here:
    user_data++;
}
    
int main ()
{
    pcl::PointCloud<pcl::PointXYZRGBA> *cloud;
    cloud= new pcl::PointCloud<pcl::PointXYZRGBA>();
    //pcl::io::loadPCDFile ("capture0001.pcd", *cloud);
   pcl::io::loadPCDFile("milk_cartoon.pcd", *cloud);
   std::vector<pcl::PointXYZRGBA> mypoints;
   std::vector<pcl::PointXYZRGBA>:: iterator pitr;

//for(pitr=cloud->points.begin();pitr!=cloud->points.end();pitr++)
//std::cout<<*pitr;
//std::vector<cloud->points>:: iterator pitr;
//convvert normal pointer to the boost pointer

//to display each points
for(int i=0;i<cloud->points.size();i++)
{
mypoints.push_back(cloud->points[i]);
//if(std::isinf(!cloud->points[i].z)&&std::isnan(!cloud->points[i].z))
if(!isnan(cloud->points[i].z))  
std::cout<<cloud->points[i].z;
}

//for(pitr=mypoints.begin();pitr!=mypoints.end();pitr++)
//std::cout<<*pitr;

   //changing raw pointer to the boost pointer  
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA> > cloud_ptr(cloud);
    pcl::visualization::CloudViewer viewer("Cloud Viewer");

    //obtaining raw pointer from the shared boost pointer
    pcl::PointCloud<pcl::PointXYZRGBA> *rawptr = cloud_ptr.get();

   //blocks until the cloud is actually rendered
    viewer.showCloud(rawptr);
    
    //use the following functions to get access to the underlying more advanced/powerful
    //PCLVisualizer
    
    //This will only get called once
    viewer.runOnVisualizationThreadOnce (viewerOneOff);
    
    //This will get called once per visualization iteration
    viewer.runOnVisualizationThread (viewerPsycho);
    while (!viewer.wasStopped ())
    {
    //you can also do cool processing here
    //FIXME: Note that this is running in a separate thread from viewerPsycho
    //and you should guard against race conditions yourself...
    user_data++;
    }
    return 0;
}

