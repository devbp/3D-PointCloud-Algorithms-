#include"display.h"

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);


void displaycloud()
{
 //Convert a pcl::PointCloud<T> object to a PointCloud2 binary data blob. 
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new    pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_ptr);
//display cloud  
//viewer->addPointCloud<pcl::PointXYZRGBA> (cloud_ptr, "sample cloud"); 
//display mesh
//display cloud mesh
  viewer->addPolygonMesh(*mesh,"cloud mesh"); 
 //viewer->addPointCloud<pcl::PolygonMesh> (mesh, "another cloud");
 //viewer->addPointCloud<pcl::PointXYZ> (Final, "registered cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.2, "sample cloud");
 // viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud_ptr, cloud_normals, 10, 0.05, "normals");  
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
   while (!viewer->wasStopped ())
    {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

}


void converttoPLY()
{
pcl::PolygonMesh::Ptr mesh1(new pcl::PolygonMesh);

/*
void 	toROSMsg (const pcl::PointCloud< PointT > &cloud, sensor_msgs::PointCloud2 &msg)
 	Convert a pcl::PointCloud<T> object to a PointCloud2 binary data blob. 
*/
 pcl::toROSMsg (*cloud_ptr, mesh1->cloud);
/*
 int 	savePLYFileASCII (const std::string &file_name, const pcl::PointCloud< PointT > &cloud)
 	Templated version for saving point cloud data to a PLY file containing a specific given cloud format.*/ 

//pcl::io::savePLYFileASCII ("test2.ply", *cloud_ptr);
  pcl::io::savePLYFileBinary("test2.ply", *cloud_ptr);
}


 void loadcloud()
{  
/*
int 	loadPCDFile (const std::string &file_name, sensor_msgs::PointCloud2 &cloud)
 	Load a PCD v.6 file into a templated PointCloud type. */
      //laod the polygon file

    //pcl::io::loadPCDFile("capture0001.pcd",mesh->cloud);
   pcl::io::loadPolygonFile("scene0070.ply", *mesh);
/*if ( pcl::io::loadPolygonFile <pcl::PolygonMesh> ("capture0001.pcd", *cloud_in) == -1)
  {
    std::cout << "Cloud reading failed." << std::endl;
    return (-1);
  }*/
   pcl::fromROSMsg( mesh->cloud, *cloud_ptr );
  // pcl::PolygonMesh::Ptr mesh1(new pcl::PolygonMesh);
/*
void 	toROSMsg (const pcl::PointCloud< PointT > &cloud, sensor_msgs::PointCloud2 &msg)
 	Convert a pcl::PointCloud<T> object to a PointCloud2 binary data blob. 
*/
 	//pcl::toROSMsg (*cloud_ptr, mesh1->cloud);
 	//Convert a pcl::PointCloud<T> object to a PointCloud2 binary data blob. 

}
