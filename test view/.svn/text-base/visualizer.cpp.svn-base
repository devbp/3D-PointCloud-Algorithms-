#include <iostream>
 //#include <pcl-1.6/pcl/io/io.h>
#include <pcl-1.7/pcl/io/pcd_io.h>
 #include <pcl-1.7/pcl/point_types.h>
#include <pcl-1.7/pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <pcl-1.7/pcl/visualization/point_picking_event.h>
#include <pcl-1.7/pcl/visualization/mouse_event.h>


pcl::PointCloud<pcl::PointXYZ> pickedpoint;





void mouseclick_callback(const pcl::visualization::MouseEvent& event ,void* viewer )
{
int x;
//x=event.getX();
//std::cout<<x<<endl;

}


void pointpick_callback (const pcl::visualization::PointPickingEvent& event, void* viewer)
{

std::cout<<"hello I am inside point pick call back function"<<endl;

  if (event.getPointIndex () == -1)
   {std::cout<<"could not get the points";
   // return;
    }

  pcl::PointXYZ current_point;
 
  event.getPoint(current_point.x, current_point.y, current_point.z);
   std::cout<<current_point.x;
   pickedpoint.push_back(current_point);
  
  std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;


}



int main(int arg, char **argv)
{
 //std::cout<<"hello"<<std::flush;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//pcl::io::loadPCDFile ("capture0001.pcd", *cloud);
 if( pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) == -1 )
 {
   std::cout << "Cloud reading failed." << std::endl;
  return (-1);
  
 }

//std::cout<<"total number of points"<<cloud->points.size()<<std::endl;
 boost::shared_ptr<pcl::visualization::PCLVisualizer>  v(new pcl::visualization::PCLVisualizer("3d"));

//pcl::visualization::MouseEvent mouse_event();

pcl::PointCloud<pcl::PointXYZ>::Ptr pickedpoint(new pcl::PointCloud<pcl::PointXYZ>);
  
v->setBackgroundColor (0, 0, 0);
v->addPointCloud(cloud,"milkcartoon");
v->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "milkcartoon");
v->addCoordinateSystem (1.0);
//boost::function<void (const pcl::visualization::PointPickingEvent)> f =
  //    boost::bind (&pointpick_callback, _1, cloud, &new_cloud_available_flag);
//boost::function<void (pcl::visualization::PointPickingEvent)> f = boost::bind(&pointpick_callback,event);
boost::signals2::connection sig =v->registerPointPickingCallback(pointpick_callback,(void*)&v);

//boost::function<void (pcl::visualization::MouseEvent)> mcb = boost::bind//(&mouseclick_callback,mouse_event);
boost::signals2::connection sig2=v->registerMouseCallback(mouseclick_callback,(void*)&v);
v->updatePointCloud(pickedpoint,"clicked_points");
// Draw clicked points in red:
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red (pickedpoint, 255, 0, 0);
//v->removePointCloud("clicked_points");
  v->addPointCloud(pickedpoint, red, "clicked_points");
  v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "clicked_points");
//const std::string s ="ITWM";
v->addText("ITWM Fraunhofer",100,100,"str");

if(sig.connected())
std::cout<<"signal connected"<<endl;
v->initCameraParameters ();

while (!v->wasStopped ())
 {

    v->spinOnce(1);
 //boost::this_thread::sleep(boost::posix_time::microseconds (10));   
}
return 0;



}







