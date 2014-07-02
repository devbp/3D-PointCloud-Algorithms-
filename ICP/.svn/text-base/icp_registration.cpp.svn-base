#include <iostream>
#include <pcl-1.7/pcl/io/pcd_io.h>
#include <pcl-1.7/pcl/point_types.h>
#include <pcl-1.7/pcl/registration/icp.h>
#include <pcl-1.7/pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl-1.7/pcl/visualization/point_cloud_color_handlers.h>
#include <pcl-1.7/pcl/registration/correspondence_estimation.h>
#include<pcl-1.7/pcl/registration/registration.h>
#include <pcl-1.7/pcl/correspondence.h>
#include <pcl-1.7/pcl/registration/transformation_estimation_svd.h>
#include <pcl-1.7/pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl-1.7/pcl/features/normal_3d.h>
#include <pcl-1.7/pcl/registration/ia_ransac.h>
#include <pcl-1.7/pcl/features/fpfh.h>

void NormalCalculation(pcl::PointCloud <pcl::PointXYZ>::Ptr cloud_ptr, pcl::PointCloud <pcl::Normal>::Ptr &normals)
{
  pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new   pcl::search::KdTree<pcl::PointXYZ>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud_ptr);
  normal_estimator.setKSearch (50);
  normal_estimator.compute (*normals);

}


void FeatureEstimation(pcl::PointCloud <pcl::PointXYZ>::Ptr cloud_ptr,pcl::PointCloud <pcl::Normal>::Ptr normals, pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs)
{
 pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
  fpfh.setInputCloud (cloud_ptr);
  fpfh.setInputNormals (normals);
  
  // Create an empty kdtree representation, and pass it to the FPFH estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  fpfh.setSearchMethod (tree);
  // Output datasets
 
  // Use all neighbors in a sphere of radius 5cm
  // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
  fpfh.setRadiusSearch (0.05);

  // Compute the features
  fpfh.compute (*fpfhs);

}



int  main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

if ( pcl::io::loadPCDFile <pcl::PointXYZ> (argv[1], *cloud_in) == -1)
  {
    std::cout << "Cloud reading failed." << std::endl;
    return (-1);
  }


pcl::PointCloud<pcl::FPFHSignature33>::Ptr sourcefpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());
pcl::PointCloud<pcl::FPFHSignature33>::Ptr targetfpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());
pcl::PointCloud <pcl::Normal>::Ptr sourcenormal(new pcl::PointCloud <pcl::Normal>());
pcl::PointCloud <pcl::Normal>::Ptr targetnormal(new pcl::PointCloud <pcl::Normal>());

/*
if ( pcl::io::loadPCDFile <pcl::PointXYZ> ("capture0002.pcd", *cloud_out) == -1)
  {
    std::cout << "Cloud reading failed." << std::endl;
    return (-1);
  }
*/

/*
  // Fill in the CloudIn data
  cloud_in->width    = 5;
  cloud_in->height   = 1;
  cloud_in->is_dense = false;
  cloud_in->points.resize (cloud_in->width * cloud_in->height);
  for (size_t i = 0; i < cloud_in->points.size (); ++i)
  {
    cloud_in->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_in->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_in->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }
  std::cout << "Saved " << cloud_in->points.size () << " data points to input:"
      << std::endl;
  for (size_t i = 0; i < cloud_in->points.size (); ++i) std::cout << "    " <<
      cloud_in->points[i].x << " " << cloud_in->points[i].y << " " <<
      cloud_in->points[i].z << std::endl;
  *cloud_out = *cloud_in;
  std::cout << "size:" << cloud_out->points.size() << std::endl;
  for (size_t i = 0; i < cloud_in->points.size (); ++i)
    cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;
  std::cout << "Transformed " << cloud_in->points.size () << " data points:"
      << std::endl;
  for (size_t i = 0; i < cloud_out->points.size (); ++i)
    std::cout << "    " << cloud_out->points[i].x << " " <<
      cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;
*/

*cloud_out = *cloud_in;

if ( pcl::io::loadPCDFile <pcl::PointXYZ> (argv[2], *cloud_out) == -1)
  {
    std::cout << "Cloud reading failed." << std::endl;
    return (-1);
  }


NormalCalculation(cloud_in,sourcenormal);
NormalCalculation(cloud_out,targetnormal);
FeatureEstimation(cloud_in,sourcenormal,sourcefpfhs);
FeatureEstimation(cloud_out,targetnormal,targetfpfhs);


//finding the correspondences between the features
pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33,pcl::FPFHSignature33> est;
est.setInputSource (sourcefpfhs);
est.setInputTarget (targetfpfhs);
//est.setSourceFeatures (sourcefpfhs);
//est.setTargetFeatures (targetfpfhs);

//pcl::CorrespondencesPtr correspondences(new pcl::Correspondences ());
//boost::shared_ptr<pcl::Correspondences> correspondences(new pcl::Correspondences ());
pcl::Correspondences correspondences;
est.determineCorrespondences (correspondences);



/*
for (size_t i = 0; i < cloud_in->points.size (); ++i)
{    cloud_out->points[i].x = cloud_in->points[i].x + 0.01f;
cloud_out->points[i].y = cloud_in->points[i].y + 0.01f;

}

*/

//find the correspondence





Eigen::Matrix4f transform;
transform<<1,0.1,0,10.0,
           0,1,0,20.5,
           0,0,1,14.5,
            0,0,0,1;
std::cout<<transform;
std::cout<<"Pose of 1st scan"<<cloud_in->sensor_origin_;
//pcl::transformPointCloud(*cloud_out, *cloud_out, transform);
std::cout<<"Pose of 2nd scan"<<cloud_out->sensor_origin_;


/*
//finding the correspondences between points
pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> est;
est.setInputSource (cloud_in);
est.setInputTarget (cloud_out);
//pcl::CorrespondencesPtr correspondences(new pcl::Correspondences ());
//boost::shared_ptr<pcl::Correspondences> correspondences(new pcl::Correspondences ());
pcl::Correspondences correspondences;
est.determineCorrespondences (correspondences);
*/

//find the transformation
Eigen::Matrix4f transformation;
pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> svd;
svd.estimateRigidTransformation(*cloud_in, *cloud_out, correspondences, transformation);
pcl::PointCloud<pcl::PointXYZ>::Ptr Final (new pcl::PointCloud<pcl::PointXYZ>);
std::cout<<transformation;

pcl::Correspondences bestcorrespondences;


//reject false correspondence
pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> sac;
sac.setInputSource(cloud_in);
sac.setInputTarget(cloud_out);
sac.setInlierThreshold(3.0);
sac.setMaximumIterations(50);
//sac.setInputCorrespondences(correspondences);
sac.getRemainingCorrespondences(correspondences,bestcorrespondences);
//sac.getCorrespondences(inliers);
Eigen::Matrix4f bestransformation = sac.getBestTransformation();

std::cout<<bestransformation;


pcl::transformPointCloud(*cloud_in, *Final, bestransformation);

*Final=*Final+*cloud_out;



/*
pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
sac_ia.setNumberOfSamples (2);
sac_ia.setMinSampleDistance (0.0001);
sac_ia.setCorrespondenceRandomness (3);
sac_ia.setMaxCorrespondenceDistance (1.0);
sac_ia.setMaximumIterations (20);
sac_ia.setInputSource (cloud_in);
sac_ia.setInputTarget (cloud_out);
sac_ia.setSourceFeatures (sourcefpfhs);
sac_ia.setTargetFeatures (targetfpfhs);
//sac_ia.align (aligned_source);
Eigen::Matrix4f transformation = sac_ia.getFinalTransformation ();
std::cout<<transformation;

*/

/*
  std::cout<<"size of source"<<cloud_in->points.size()<<std::endl;
  std::cout<<"size of target"<<cloud_out->points.size()<<std::endl;
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(cloud_out);
  icp.setInputTarget(cloud_in);
// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)


  icp.setMaxCorrespondenceDistance (icp.getMaxCorrespondenceDistance () - 0.01);
//Set the maximum number of iterations (criterion 1)
  icp.setMaximumIterations (19);
// Set the transformation epsilon (criterion 2)
 // icp.setTransformationEpsilon ();
// Set the euclidean distance difference epsilon (criterion 3)
  icp.setEuclideanFitnessEpsilon (0.5);

  pcl::PointCloud<pcl::PointXYZ>::Ptr Final (new pcl::PointCloud<pcl::PointXYZ>);
  icp.align(*Final);
*Final=*Final+*cloud_in;
std::cout<<"Pose of registered cloud"<<Final->sensor_origin_;
std::cout<<"size of registered cloud"<<Final->points.size()<<std::endl;
  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;

  std::cout << icp.getFinalTransformation() << std::endl;
*/


 boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new 
 pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_ptr);
 pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorhandler(cloud_out,0.0,0.0,255.0);
 viewer->addPointCloud<pcl::PointXYZ> (cloud_out, colorhandler, "sample cloud");  
 viewer->addPointCloud<pcl::PointXYZ> (cloud_in, "another cloud");
 viewer->addCorrespondences<pcl::PointXYZ> (cloud_in,cloud_out,bestcorrespondences,"correspondences");
 pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorhandler1(Final,0.0,255.0,0.0);
 viewer->addPointCloud<pcl::PointXYZ> (Final,colorhandler1, "registered cloud");
 viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.1, "registered cloud");
 // viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud_ptr, cloud_normals, 10, 0.05, "normals");  
 viewer->addCoordinateSystem (1.0);
 viewer->initCameraParameters ();
 while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }


 return (0);
}
