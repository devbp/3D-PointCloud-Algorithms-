//#include <pcl-1.6/pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl-1.6/pcl/io/io.h>
#include <pcl-1.6/pcl/io/pcd_io.h>
#include<pcl-1.6/pcl/point_cloud.h>
#include<pcl-1.6/pcl/point_types.h>

#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include<cmath>
#include <boost/thread/thread.hpp>
    
    
int main ()
{
    pcl::PointCloud<pcl::PointXYZRGBA> *cloud;
    cloud= new pcl::PointCloud<pcl::PointXYZRGBA>();
    //pcl::io::loadPCDFile ("capture0001.pcd", *cloud);
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::io::loadPCDFile("milk_cartoon.pcd", *cloud_xyzrgb);
   pcl::io::loadPCDFile("milk_cartoon.pcd", *cloud);
   std::vector<pcl::PointXYZRGB> mypoints;
  //changing raw pointer to the boost pointer  
    
boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA> > cloud_ptr(cloud);
  
pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);

/*
    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
    ne.setInputCloud (cloud_ptr);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());
  ne.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.01);
  // Compute the features
  ne.compute (*cloud_normals);*/

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud_ptr);
  //viewer->addPointCloud<pcl::PointXYZRGB> (cloud_ptr, "sample cloud");  
  //viewer->addPointCloud<pcl::PointXYZRGBA> (cloud_ptr, rgb, "sample cloud");
  
 //viewer->addPointCloudNormals<pcl::PointXYZRGBA, pcl::Normal> (cloud_ptr, cloud_normals, 10, 0.05, "normals");  

//convert point cloud to polygon mesh
 pcl::toROSMsg(*cloud_ptr, mesh->cloud);


for (size_t i=0; i<mesh->polygons.size(); ++i) {
    for (size_t j=0; j<mesh->polygons[i].vertices.size(); ++j) {
      std::cout << mesh->polygons[i].vertices[j] << std::endl;
    }
  } 

 viewer->addPolygonMesh(*mesh, "mesh display");
viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	//addPolygon (const typename pcl::PointCloud< PointT >::ConstPtr &cloud, const std::string &id="polygon", int viewport=0)
	//viewer->addPolygon<pcl::PointXYZRGBA> (cloud_xyzrgb,244,255,212,"polygon",0) ;
viewer->addCoordinateSystem (0.0);
  viewer->initCameraParameters ();
 while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }



    
}

