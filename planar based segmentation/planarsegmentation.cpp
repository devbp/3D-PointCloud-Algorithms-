#include<iostream>
#include<pcl-1.7/pcl/ModelCoefficients.h>
#include<pcl-1.7/pcl/io/pcd_io.h>
#include<pcl-1.7/pcl/sample_consensus/method_types.h>
#include<pcl-1.7/pcl/sample_consensus/model_types.h>
#include<pcl-1.7/pcl/segmentation/sac_segmentation.h>
#include<pcl-1.7/pcl/filters/extract_indices.h>
#include<pcl-1.7/pcl/visualization/pcl_visualizer.h>

int main(int argc, char** argv)
{
pcl::PCDReader reader;
pcl::PointCloud<pcl::PointXYZ>:: Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
pcl::ExtractIndices<pcl::PointXYZ> extract;
reader.read(argv[1], *cloud_ptr);
pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
seg.setOptimizeCoefficients (true);
  // Mandatory
seg.setModelType (pcl::SACMODEL_PLANE);
seg.setMethodType (pcl::SAC_RANSAC);
seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud_ptr);
  seg.segment (*inliers, *coefficients);

extract.setInputCloud(cloud_ptr->makeShared());
extract.setIndices(inliers);
extract.setNegative(false);

pcl::PointCloud<pcl::PointXYZ>::Ptr segmented_plane(new pcl::PointCloud<pcl::PointXYZ>);
extract.filter(*segmented_plane);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return (-1);
  }

  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));


viewer->addPointCloud<pcl::PointXYZ> (segmented_plane, "segment");

  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (0.0);
  viewer->initCameraParameters ();

while (!viewer->wasStopped ())
   {
    viewer->spinOnce (100);

   }


return 0;

}

