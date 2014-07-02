#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h>

/*
typedef boost::shared_ptr<const std::vector<int> > pcl::IndicesConstPtr


*/

int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_segment1(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr segment_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if ( pcl::io::loadPCDFile <pcl::PointXYZ> (argv[1], *cloud) == -1)
  {
    std::cout << "Cloud reading failed." << std::endl;
    return (-1);
  }

  pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);

 //pcl::search::Search<pcl::PointXYZ>::Ptr tree = (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud);
  normal_estimator.setKSearch (50);
  normal_estimator.compute (*normals);


  pcl::ExtractIndices<pcl::PointXYZ> extract;

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  //pass.setInputCloud (cloud);
  //pass.setFilterFieldName ("z");
  //pass.setFilterLimits (0.0, 1.0);
  //pass.filter (*indices);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  reg.setMinClusterSize (5000);
  reg.setMaxClusterSize (10000000);
  reg.setSearchMethod (tree);
  //reg.setNumberOfNeighbours (10);
reg.setNumberOfNeighbours(50);  
reg.setInputCloud (cloud);
  //reg.setIndices (indices);
  reg.setInputNormals (normals);
  //reg.setSmoothnessThreshold (7.0 / 180.0 * M_PI);
  //reg.setCurvatureThreshold (1.0);
   reg.setSmoothnessThreshold(3.0/ 180.0 * M_PI);
reg.setCurvatureThreshold (1.5);
  std::vector <pcl::PointIndices> clusters;

  reg.extract (clusters);


  std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
  std::cout << "First cluster has " << clusters[0].indices.size () << " points." << endl;
  std::cout << "These are the indices of the points of the initial" <<
    std::endl << "cloud that belong to the first cluster:" << std::endl;
  int counter = 0;
 
std::vector<int> pointindices;
int i=0;

while( i<clusters[1].indices.size())
{
pointindices.push_back(clusters[1].indices[i]);
i++;
}

pcl::copyPointCloud<pcl::PointXYZ>(*cloud, pointindices, *cloud_segment1);

  while ( counter <clusters[0].indices.size ())
  {
//inliers->indices.push_back()
indices->push_back(clusters[0].indices[counter]);
    //std::cout << clusters[1].indices[counter] << std::endl;
  counter++;
  }
  extract.setInputCloud(cloud);
  extract.setNegative(false);
  extract.setIndices(indices);
  extract.filter(*segment_cloud);
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  pcl::visualization::CloudViewer viewer1 ("Segment");
  //viewer1.showCloud(cloud_segment1);
 viewer1.showCloud(colored_cloud);

while (!viewer1.wasStopped ())
  {
  }


  return (0);
}
