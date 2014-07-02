//#include <pcl-1.6/pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl-1.6/pcl/io/io.h>
#include <pcl-1.6/pcl/io/pcd_io.h>
#include<pcl-1.6/pcl/point_cloud.h>
#include<pcl-1.6/pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include<pcl/sample_consensus/ransac.h>
#include<pcl/sample_consensus/sac_model_plane.h>
#include<pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include<cmath>
#include <boost/thread/thread.hpp>
    
    
int main ()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("milk_cartoon.pcd", *cloud_xyz);
       //changing raw pointer to the boost pointer  
    std::vector<int> inliers;
Eigen::Vector4f sensorposition;
sensorposition=cloud_xyz->sensor_origin_;
std::cout<<sensorposition;
  pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud_xyz));
  // pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZ> (cloud_xyz));  
   pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
   ransac.setDistanceThreshold (0.01);
   ransac.computeModel();
   ransac.getInliers(inliers);

   pcl::copyPointCloud<pcl::PointXYZ>(*cloud_xyz, inliers, *final);
   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new    pcl::visualization::PCLVisualizer ("3D Viewer"));
   viewer->setBackgroundColor (0, 0, 0);
  //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_ptr);
   viewer->addPointCloud<pcl::PointXYZ> (final, "sample cloud");  
//viewer->addPointCloud<pcl::PointXYZRGB> (cloud_ptr, rgb, "sample cloud");
   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
 // viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud_ptr, cloud_normals, 10, 0.05, "normals");  
   viewer->addCoordinateSystem (1.0);
   viewer->initCameraParameters ();
  while (!viewer->wasStopped ())
   {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
   }


    
}

