#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/min_cut_segmentation.h>

int main (int argc, char** argv)
{
  pcl::PointCloud <pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud <pcl::PointXYZ>);
  if ( pcl::io::loadPCDFile <pcl::PointXYZ> (argv[1], *cloud) == -1 )
  {
    std::cout << "Cloud reading failed." << std::endl;
    return (-1);
  }

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  //pass.setInputCloud (cloud);
  //pass.setFilterFieldName ("x");
  //pass.setFilterLimits (0.0, 1.0);
  //pass.filter (*indices);
 //pass.filter(*filtered_cloud);
   pcl::MinCutSegmentation<pcl::PointXYZ> seg;
   seg.setInputCloud (cloud);
  
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr foreground_points(new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointXYZ point;
  point.x = 2.4;
  point.y = -1.55;
  point.z = 0.57;
  foreground_points->points.push_back(point);
  seg.setForegroundPoints(foreground_points);
  seg.setSigma (0.25);
  seg.setRadius (3.0433856);
  seg.setNumberOfNeighbours (14);
  seg.setSourceWeight (0.8);

  std::vector <pcl::PointIndices> clusters;
  seg.extract (clusters);

  std::cout << "Maximum flow is " << seg.getMaxFlow () << std::endl;

  std::cout<<"Number of cluster ="<<clusters.size()<<std::endl;
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = seg.getColoredCloud ();

  pcl::visualization::CloudViewer viewer ("Cluster viewer");
  viewer.showCloud(colored_cloud);

 //pcl::visualization::CloudViewer viewer2 ("Filtered Cloud");
  //viewer2.showCloud(filtered_cloud);

while (!viewer.wasStopped ())
  {

  }
 /*
  while (!viewer2.wasStopped ())
  {
  }*/
 return (0);

}
